"""
single_leg_test.py
==================
Standalone 3-DOF single-leg test for Argos.
No ROS — drives the PCA9685 directly over I2C from the Raspberry Pi.

Joints tested:
    Channel HIP_CH   → theta_1    hip abductor   (lateral, from Dingo)
    Channel TOP_CH   → theta_top  upper leg       (top servo, direct drive)
    Channel BOT_CH   → theta_bot  lower leg       (bottom servo, bell crank)

Install once on the Pi:
    pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-busdevice

Usage:
    python3 single_leg_test.py              # interactive menu
    python3 single_leg_test.py --sweep      # automated arc sweep
    python3 single_leg_test.py --goto 0 -0.061 -0.200   # body-frame (x,y,z) metres
    python3 single_leg_test.py --sag 0 200              # sagittal (x_mm, y_mm)
    python3 single_leg_test.py --raw 3 -48 79           # raw angles θ1 θtop θbot (deg)
    python3 single_leg_test.py --sim        # simulation only, no hardware
    python3 single_leg_test.py --math       # IK/FK self-test only

╔══════════════════════════════════════════════════════════════════╗
║  EDIT THE GEOMETRY BLOCK BELOW TO MATCH YOUR HARDWARE           ║
╚══════════════════════════════════════════════════════════════════╝
"""

import sys
import time
import argparse
import numpy as np
from math import pi, acos, atan2, sqrt, sin, cos, fmod, asin, radians, degrees

# ══════════════════════════════════════════════════════════════════════════════
#  ▼▼▼  HARDWARE CONFIG — EDIT THESE  ▼▼▼
# ══════════════════════════════════════════════════════════════════════════════

PCA9685_I2C_ADDRESS = 0x40   # default; check your board's solder bridges

# PCA9685 channel assignments
HIP_CH = 2    # hip abductor servo    (θ_1)
TOP_CH = 1    # upper leg servo       (θ_top, direct drive)
BOT_CH = 0    # lower leg servo       (θ_bot, bell crank)

# PWM pulse-width limits (µs) — from your servo datasheet
PWM_MIN_US = 370     # pulse at   0° command
PWM_MAX_US = 2400    # pulse at 180° command
PWM_FREQ   = 50      # Hz (standard servo)

# ── Servo mounting corrections ────────────────────────────────────────────────
# After first power-up, if a servo moves in the wrong direction flip its
# MULTIPLIER (+1 → -1).  If neutral is off, adjust OFFSET (degrees).
# Procedure: run  --raw 0 0 0  and check physical leg position.

HIP_OFFSET     =  0.0   # degrees
HIP_MULTIPLIER = +1     # +1 or -1

TOP_OFFSET     =  0.0   # degrees
TOP_MULTIPLIER = +1     # +1 or -1

BOT_OFFSET     =  0.0   # degrees
BOT_MULTIPLIER = +1     # +1 or -1

# ── Which leg is this? (affects hip abductor direction) ───────────────────────
# 0=FR, 1=FL, 2=RR, 3=RL
LEG_INDEX = 0

# ══════════════════════════════════════════════════════════════════════════════
#  ▼▼▼  LEG GEOMETRY — EDIT THESE TO MATCH YOUR HARDWARE  ▼▼▼
# ══════════════════════════════════════════════════════════════════════════════
#
#  All distances in METRES.  See Config.py for measurement guide.
#
PARAMS = dict(
    # ── Hip abductor ────────────────────────────────────────────────────────
    L1_hip = 0.05162,         # m  coxa link length (hip pivot → femur pivot)
    phi    = radians(73.917), # rad coxa angle from horizontal

    # ── Upper and lower leg ─────────────────────────────────────────────────
    L1     = 0.130,           # m  upper leg: top servo shaft → knee hinge
    L2     = 0.150,           # m  lower leg: knee hinge → foot tip

    # ── Bell crank ──────────────────────────────────────────────────────────
    rbl    = 0.035,           # m  bell crank left  arm (→ Rod 2)
    rbr    = 0.050,           # m  bell crank right arm (← Rod 1)
    abc    = radians(60.0),   # rad angle between arms (right arm is abc CW from left)

    # ── Rods ────────────────────────────────────────────────────────────────
    Lr1    = 0.065,           # m  Rod 1: bot servo horn → BC right arm
    Lr2    = 0.120,           # m  Rod 2: BC left arm → lower leg pivot

    # ── Bottom servo horn ────────────────────────────────────────────────────
    rh     = 0.035,           # m  horn arm radius (shaft → Rod 1 pin)

    # ── Knee pivot offset ────────────────────────────────────────────────────
    dko    = 0.030,           # m  Rod 2 pivot: distance above knee along lower leg
)

# Lateral offset of this leg from body centre (used for body-frame commands)
LEG_LATERAL_OFFSET = -0.061  # m  FR/RR = -0.061 (right), FL/RL = +0.061 (left)

# ▲▲▲  END OF GEOMETRY TO EDIT  ▲▲▲

# Safe motion parameters
STEPS_PER_MOVE = 30       # interpolation steps between positions
STEP_DELAY_S   = 0.018    # seconds per step (~55 Hz)
STARTUP_SAG    = (0, 200) # (x_mm, y_mm) safe starting position in sagittal plane


# ══════════════════════════════════════════════════════════════════════════════
#  KINEMATICS  (self-contained, no external imports)
# ══════════════════════════════════════════════════════════════════════════════

def _point_to_rad(p1, p2):
    return (atan2(p2, p1) + 2*pi) % (2*pi)

def _rotx(a):
    c,s=cos(a),sin(a); return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def _ci(P1, r1, P2, r2):
    dx,dy=P2[0]-P1[0],P2[1]-P1[1]; d=sqrt(dx*dx+dy*dy)
    if d<1e-9 or d>r1+r2+1e-9 or d<abs(r1-r2)-1e-9: return None,None
    a=(r1*r1-r2*r2+d*d)/(2*d); h=sqrt(max(0,r1*r1-a*a))
    mx=P1[0]+a*dx/d; my=P1[1]+a*dy/d; px,py=-dy/d,dx/d
    return np.array([mx+h*px,my+h*py]), np.array([mx-h*px,my-h*py])

def hip_ik(r_foot, leg_index, P):
    """
    Hip abductor IK.  r_foot is body-frame foot position (x,y,z) metres.
    Returns (theta_1_deg, x_sag_m, y_sag_m).
    """
    is_right = 1 if leg_index in (0, 2) else 0
    x,y,z = float(r_foot[0]),float(r_foot[1]),float(r_foot[2])
    if is_right: y=-y
    R1=pi/2-P['phi']; rf=_rotx(-R1)@np.array([x,y,z]); x,y,z=rf
    len_A=sqrt(y*y+z*z)
    a1=_point_to_rad(y,z); a2=asin(sin(P['phi'])*P['L1_hip']/len_A); a3=pi-a2-P['phi']
    t1=a1+a3
    if t1>=2*pi: t1=fmod(t1,2*pi)
    offset=np.array([0.,P['L1_hip']*cos(t1),P['L1_hip']*sin(t1)])
    translated=np.array([x,y,z])-offset
    R2=t1+P['phi']-pi/2; sag=_rotx(-R2)@translated; x_,_,z_=sag
    return degrees(t1), x_, -z_

def sag_ik(x_m, y_m, P):
    """
    Sagittal coupled IK.  x_m=forward(m), y_m=downward(m).
    Returns (theta_top_deg, theta_bot_deg) or None.
    """
    L1,L2=P['L1'],P['L2']; O=np.zeros(2)
    d=sqrt(x_m*x_m+y_m*y_m); mr=(L1+L2)*0.99
    if d>mr: x_m,y_m=x_m*mr/d,y_m*mr/d; d=mr
    if d<abs(L1-L2)*1.01: return None
    ck=max(-1,min(1,(L1**2+L2**2-d**2)/(2*L1*L2))); ki=acos(ck)
    al=atan2(x_m,y_m); cb=max(-1,min(1,(L1**2+d**2-L2**2)/(2*L1*d))); tt=al-acos(cb)
    knee=np.array([L1*sin(tt),L1*cos(tt)])
    tl=tt+(pi-ki); r2=knee-P['dko']*np.array([sin(tl),cos(tl)])
    A,B=_ci(O,P['rbl'],r2,P['Lr2'])
    if A is None: return None
    bcl=A if A[0]<=B[0] else B
    phl=atan2(bcl[0],bcl[1]); phr=phl-P['abc']
    bcr=np.array([P['rbr']*sin(phr),P['rbr']*cos(phr)])
    A,B=_ci(O,P['rh'],bcr,P['Lr1'])
    if A is None: return None
    horn=A if A[0]>=B[0] else B
    return degrees(tt), degrees(atan2(horn[0],horn[1]))

def sag_fk(tt_d, tb_d, P):
    """Sagittal FK → (x_mm, y_mm) or None."""
    tt,tb=radians(tt_d),radians(tb_d); L1,L2=P['L1'],P['L2']; O=np.zeros(2)
    knee=np.array([L1*sin(tt),L1*cos(tt)])
    horn=np.array([P['rh']*sin(tb),P['rh']*cos(tb)])
    A,B=_ci(O,P['rbr'],horn,P['Lr1'])
    if A is None: return None
    def bclx(c): return sin(atan2(c[0],c[1])+P['abc'])
    bcr=A if bclx(A)<bclx(B) else B
    phr=atan2(bcr[0],bcr[1]); phl=phr+P['abc']
    bcl=np.array([P['rbl']*sin(phl),P['rbl']*cos(phl)])
    A,B=_ci(knee,P['dko'],bcl,P['Lr2'])
    if A is None: return None
    def diff(c):
        ld=knee-c; d=(atan2(ld[0],ld[1])-tt)%(2*pi)
        return d-2*pi if d>pi else d
    dA,dB=diff(A),diff(B)
    rod2=A if 0<dA<pi else (B if 0<dB<pi else (A if abs(dA-pi/2)<abs(dB-pi/2) else B))
    ld=(knee-rod2)/np.linalg.norm(knee-rod2); foot=knee+L2*ld
    return foot[0]*1000, foot[1]*1000

def full_ik_from_sag(x_mm, y_mm, lateral_m=None):
    """
    Full 3-DOF IK from a sagittal foot target.
    lateral_m: lateral body-frame offset (defaults to LEG_LATERAL_OFFSET).
    Returns (t1_deg, tt_deg, tb_deg) or None.
    """
    if lateral_m is None:
        lateral_m = LEG_LATERAL_OFFSET
    # Build body-frame foot position (z=-y_sag, x=x_sag)
    r = np.array([x_mm/1000.0, lateral_m, -y_mm/1000.0])
    t1_deg, x_sag, y_sag = hip_ik(r, LEG_INDEX, PARAMS)
    sol = sag_ik(x_sag, y_sag, PARAMS)
    if sol is None:
        return None
    return t1_deg, sol[0], sol[1]


# ══════════════════════════════════════════════════════════════════════════════
#  PWM CONVERSION
# ══════════════════════════════════════════════════════════════════════════════

def _ik_to_servo_us(joint_angle_deg, offset_deg, multiplier):
    """
    IK angle (degrees, measured from vertical, CW positive)
    → servo command angle (0–180°) → PWM pulse width (µs).
    Neutral mapping: 0° IK → 90° command → 1500 µs.
    """
    cmd = 90.0 + multiplier * joint_angle_deg + offset_deg
    cmd = max(0.0, min(180.0, cmd))
    return PWM_MIN_US + (cmd / 180.0) * (PWM_MAX_US - PWM_MIN_US)

def _us_to_duty(pulse_us):
    """Pulse width (µs) → PCA9685 duty cycle integer (0–65535)."""
    period_us = 1_000_000.0 / PWM_FREQ
    return int(round(pulse_us / period_us * 65535))


# ══════════════════════════════════════════════════════════════════════════════
#  HARDWARE INTERFACE
# ══════════════════════════════════════════════════════════════════════════════

_pca = None

def init_hardware():
    global _pca
    try:
        import board, busio
        from adafruit_pca9685 import PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        _pca = PCA9685(i2c, address=PCA9685_I2C_ADDRESS)
        _pca.frequency = PWM_FREQ
        print(f"  PCA9685 OK  (addr=0x{PCA9685_I2C_ADDRESS:02X}  "
              f"channels: hip={HIP_CH} top={TOP_CH} bot={BOT_CH})")
        return True
    except Exception as e:
        print(f"  [WARN] PCA9685 unavailable ({e})")
        print(f"         Running in simulation mode.\n")
        return False

def _set_pulse(ch, pulse_us):
    if _pca is None: return
    _pca.channels[ch].duty_cycle = _us_to_duty(pulse_us)

def send_angles(t1_deg, tt_deg, tb_deg, verbose=True):
    """Drive all three servos to the given IK angles."""
    hip_us = _ik_to_servo_us(t1_deg, HIP_OFFSET, HIP_MULTIPLIER)
    top_us = _ik_to_servo_us(tt_deg, TOP_OFFSET, TOP_MULTIPLIER)
    bot_us = _ik_to_servo_us(tb_deg, BOT_OFFSET, BOT_MULTIPLIER)
    _set_pulse(HIP_CH, hip_us)
    _set_pulse(TOP_CH, top_us)
    _set_pulse(BOT_CH, bot_us)
    if verbose:
        print(f"    θ_1  ={t1_deg:+7.2f}°  ch{HIP_CH}  {hip_us:.0f}µs")
        print(f"    θ_top={tt_deg:+7.2f}°  ch{TOP_CH}  {top_us:.0f}µs")
        print(f"    θ_bot={tb_deg:+7.2f}°  ch{BOT_CH}  {bot_us:.0f}µs")

def move_to_sag(x_mm, y_mm, from_angles=None, verbose=True):
    """
    Move foot to sagittal (x_mm, y_mm) with smooth interpolation.
    from_angles: (t1, tt, tb) in degrees — current position.
    Returns (t1, tt, tb) on success, None on failure.
    """
    sol = full_ik_from_sag(x_mm, y_mm)
    if sol is None:
        print(f"  [WARN] ({x_mm:.0f}, {y_mm:.0f}) mm unreachable — skipped")
        return None
    t1t, ttt, tbt = sol

    if verbose:
        fk = sag_fk(ttt, tbt, PARAMS)
        fk_str = f"({fk[0]:.1f}, {fk[1]:.1f})mm" if fk else "?"
        print(f"\n  Target: ({x_mm:.0f}, {y_mm:.0f})mm  FK check: {fk_str}")

    if from_angles is None or _pca is None:
        send_angles(t1t, ttt, tbt, verbose=verbose)
        return t1t, ttt, tbt

    t1s, tts, tbs = from_angles
    for step in range(1, STEPS_PER_MOVE + 1):
        f = step / STEPS_PER_MOVE
        send_angles(t1s + f*(t1t-t1s), tts + f*(ttt-tts), tbs + f*(tbt-tbs), verbose=False)
        time.sleep(STEP_DELAY_S)

    if verbose:
        send_angles(t1t, ttt, tbt, verbose=True)
    return t1t, ttt, tbt

def release():
    if _pca is None: return
    for ch in [HIP_CH, TOP_CH, BOT_CH]:
        _pca.channels[ch].duty_cycle = 0
    print("  Servos released.")


# ══════════════════════════════════════════════════════════════════════════════
#  TESTS
# ══════════════════════════════════════════════════════════════════════════════

def test_math():
    print("\n── IK/FK self-test ─────────────────────────────────────────────")
    print("  Sagittal IK/FK (pure 2-DOF coupled bell-crank):")
    sag_tests = [
        (  0.000,  0.200, "Nominal"),
        (  0.040,  0.190, "Fwd +40mm"),
        (  0.080,  0.170, "Wide fwd"),
        ( -0.040,  0.190, "Back -40mm"),
        ( -0.080,  0.160, "Wide back"),
        (  0.000,  0.170, "Raised"),
        (  0.000,  0.230, "Extended"),
    ]
    print(f"  {'Pose':>16}   {'θ_top':>8}  {'θ_bot':>8}  FK err (mm)")
    print("  " + "─"*52)
    all_ok = True
    for x_m, y_m, name in sag_tests:
        sol = sag_ik(x_m, y_m, PARAMS)
        if sol is None:
            print(f"  {name:<16}  UNREACHABLE"); continue
        tt, tb = sol
        fk = sag_fk(tt, tb, PARAMS)
        # FK returns mm; convert target to mm for comparison
        x_mm, y_mm = x_m*1000, y_m*1000
        err = sqrt((fk[0]-x_mm)**2+(fk[1]-y_mm)**2) if fk else float('nan')
        ok  = "✓" if err < 0.001 else f"✗{err:.4f}mm"
        if err >= 0.001: all_ok = False
        print(f"  {name:<16}   {tt:+7.3f}°  {tb:+7.3f}°  {ok}")

    print(f"\n  Hip abductor IK (lateral solve for leg {LEG_INDEX}):")
    hip_tests = [
        (  0.000, -0.061, -0.200, "Nominal, FR"),
        (  0.050, -0.061, -0.190, "Fwd 50mm, FR"),
        ( -0.040, -0.061, -0.190, "Back 40mm, FR"),
    ]
    print(f"  {'Pose':>18}   {'θ_1':>7}  {'x_sag':>9}  {'y_sag':>9}")
    print("  " + "─"*52)
    for bx,by,bz,name in hip_tests:
        r = np.array([bx,by,bz])
        t1,xs,ys = hip_ik(r, LEG_INDEX, PARAMS)
        print(f"  {name:<18}   {t1:+6.2f}°  {xs*1000:+8.2f}mm  {ys*1000:+8.2f}mm")

    print(f"\n  {'Sagittal all passed ✓' if all_ok else 'FAILURES ✗'}")
    return all_ok

def test_sweep(sim_only=False):
    print("\n── Sweep test ──────────────────────────────────────────────────")
    waypoints = [
        (  0, 200), ( 40, 190), ( 80, 170), ( 80, 200),
        ( 40, 210), (  0, 210), (-40, 210), (-80, 200),
        (-80, 170), (-40, 190), (  0, 200),
    ]
    current = None
    if not sim_only and _pca is not None:
        print("  Moving to start …")
        current = move_to_sag(*waypoints[0], verbose=True)
        time.sleep(0.5)

    print(f"\n  {'Step':>4}  {'x':>6}  {'y':>6}  {'θ_1':>7}  {'θ_top':>8}  {'θ_bot':>8}  FK err")
    print("  " + "─"*62)
    for i,(x,y) in enumerate(waypoints):
        sol = full_ik_from_sag(x, y)
        if sol is None:
            print(f"  {i+1:>4}  ({x:+4.0f},{y:4.0f})  UNREACHABLE"); continue
        t1,tt,tb = sol
        fk = sag_fk(tt, tb, PARAMS)
        # Compare FK to post-hip sagittal target (not raw mm input)
        r = np.array([x/1000.0, LEG_LATERAL_OFFSET, -y/1000.0])
        _, xs, ys = hip_ik(r, LEG_INDEX, PARAMS)
        err = sqrt((fk[0]-xs*1000)**2+(fk[1]-ys*1000)**2) if fk else float('nan')
        ok  = "✓" if err < 0.001 else f"✗{err:.3f}mm"
        print(f"  {i+1:>4}  ({x:+4.0f},{y:4.0f})  {t1:+6.2f}°  {tt:+7.2f}°  {tb:+7.2f}°  {ok}")
        if not sim_only and _pca is not None:
            current = move_to_sag(x, y, from_angles=current, verbose=False)
            time.sleep(0.3)

    if not sim_only and _pca is not None:
        print("\n  Sweep done. Releasing in 2 s …")
        time.sleep(2); release()

def interactive(sim_only=False):
    print("\n── Interactive mode ────────────────────────────────────────────")
    print("  Commands:")
    print("    sag <x> <y>      sagittal target mm       e.g.  sag 0 200")
    print("    body <x> <y> <z> body-frame target (m)    e.g.  body 0 -0.061 -0.200")
    print("    raw <t1> <tt> <tb>  raw angles (deg)      e.g.  raw 3 -48 79")
    print("    ik  <x> <y>      print IK only (no move)  e.g.  ik 40 190")
    print("    fk  <tt> <tb>    print FK only             e.g.  fk -48 79")
    print("    sweep            run arc sweep")
    print("    neutral          return to nominal (0, 200)")
    print("    release          servos go limp")
    print("    q / quit         exit")
    print()

    current = None
    if not sim_only and _pca is not None:
        print("  Moving to neutral …")
        current = move_to_sag(*STARTUP_SAG, verbose=True)
        time.sleep(0.5)

    while True:
        try:
            raw = input("  > ").strip()
        except (KeyboardInterrupt, EOFError):
            break
        if not raw: continue
        parts = raw.split(); cmd = parts[0].lower()

        if cmd in ('q','quit','exit'):
            break

        elif cmd == 'sag' and len(parts) == 3:
            try:
                x,y = float(parts[1]),float(parts[2])
                current = move_to_sag(x, y, from_angles=current, verbose=True)
            except ValueError: print("  Usage: sag <x_mm> <y_mm>")

        elif cmd == 'body' and len(parts) == 4:
            try:
                bx,by,bz = float(parts[1]),float(parts[2]),float(parts[3])
                r = np.array([bx, by, bz])
                t1_d, x_s, y_s = hip_ik(r, LEG_INDEX, PARAMS)
                sol = sag_ik(x_s, y_s, PARAMS)
                if sol is None:
                    print("  Unreachable"); continue
                tt_d, tb_d = sol
                print(f"  IK: θ_1={t1_d:+.2f}°  θ_top={tt_d:+.2f}°  θ_bot={tb_d:+.2f}°")
                if not sim_only:
                    if current: current = move_to_sag(x_s*1000, y_s*1000, from_angles=current, verbose=False)
                    send_angles(t1_d, tt_d, tb_d, verbose=True)
                    current = (t1_d, tt_d, tb_d)
            except ValueError: print("  Usage: body <x_m> <y_m> <z_m>")

        elif cmd == 'raw' and len(parts) == 4:
            try:
                t1,tt,tb = float(parts[1]),float(parts[2]),float(parts[3])
                send_angles(t1, tt, tb, verbose=True)
                fk = sag_fk(tt, tb, PARAMS)
                if fk: print(f"  FK foot (sagittal): ({fk[0]:.2f}, {fk[1]:.2f}) mm")
                current = (t1, tt, tb)
            except ValueError: print("  Usage: raw <t1_deg> <tt_deg> <tb_deg>")

        elif cmd == 'ik' and len(parts) == 3:
            try:
                x,y = float(parts[1]),float(parts[2])
                sol = full_ik_from_sag(x, y)
                if sol:
                    t1,tt,tb = sol
                    fk = sag_fk(tt, tb, PARAMS)
                    print(f"  IK: θ_1={t1:+.2f}°  θ_top={tt:+.2f}°  θ_bot={tb:+.2f}°")
                    if fk: print(f"  FK check: ({fk[0]:.4f}, {fk[1]:.4f})mm  "
                                 f"err={sqrt((fk[0]-x)**2+(fk[1]-y)**2):.6f}mm")
                else:
                    print(f"  ({x},{y})mm unreachable")
            except ValueError: print("  Usage: ik <x_mm> <y_mm>")

        elif cmd == 'fk' and len(parts) == 3:
            try:
                tt,tb = float(parts[1]),float(parts[2])
                fk = sag_fk(tt, tb, PARAMS)
                if fk: print(f"  FK: θ_top={tt:.2f}°  θ_bot={tb:.2f}°  →  ({fk[0]:.2f}, {fk[1]:.2f})mm")
                else:   print("  FK: infeasible")
            except ValueError: print("  Usage: fk <theta_top_deg> <theta_bot_deg>")

        elif cmd == 'sweep':
            test_sweep(sim_only=sim_only); current = None

        elif cmd == 'neutral':
            current = move_to_sag(*STARTUP_SAG, from_angles=current, verbose=True)

        elif cmd == 'release':
            release(); current = None

        else:
            print(f"  Unknown: {raw}")

    if not sim_only and _pca is not None:
        print("\n  Releasing …"); release()


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="Argos 3-DOF single-leg test")
    ap.add_argument('--sim',    action='store_true', help="Simulation only (no I2C)")
    ap.add_argument('--math',   action='store_true', help="IK/FK self-test and exit")
    ap.add_argument('--sweep',  action='store_true', help="Run arc sweep and exit")
    ap.add_argument('--sag',    nargs=2, type=float, metavar=('X','Y'),
                    help="Move sagittal foot to (X Y)mm and exit")
    ap.add_argument('--goto',   nargs=3, type=float, metavar=('X','Y','Z'),
                    help="Move body-frame foot to (X Y Z)m and exit")
    ap.add_argument('--raw',    nargs=3, type=float, metavar=('T1','TT','TB'),
                    help="Send raw angles (θ1 θtop θbot degrees) and exit")
    args = ap.parse_args()

    print("=" * 62)
    print("  Argos 3-DOF Single-Leg Test")
    print(f"  Leg index   : {LEG_INDEX}  "
          f"({'FR' if LEG_INDEX==0 else 'FL' if LEG_INDEX==1 else 'RR' if LEG_INDEX==2 else 'RL'})")
    print(f"  Channels    : hip={HIP_CH}  top={TOP_CH}  bot={BOT_CH}")
    print(f"  L1_hip={PARAMS['L1_hip']*1000:.1f}mm  L_upper={PARAMS['L1']*1000:.1f}mm  "
          f"L_lower={PARAMS['L2']*1000:.1f}mm")
    print(f"  BC left={PARAMS['rbl']*1000:.1f}mm  right={PARAMS['rbr']*1000:.1f}mm  "
          f"α={degrees(PARAMS['abc']):.0f}°")
    print(f"  Rod1={PARAMS['Lr1']*1000:.1f}mm  Rod2={PARAMS['Lr2']*1000:.1f}mm  "
          f"horn={PARAMS['rh']*1000:.1f}mm  dko={PARAMS['dko']*1000:.1f}mm")
    print("=" * 62)

    if not test_math():
        print("\n  IK self-test FAILED — check PARAMS. Aborting.")
        sys.exit(1)

    if args.math:
        return

    sim_only = args.sim
    if not sim_only:
        sim_only = not init_hardware()

    if args.sag:
        x, y = args.sag
        print(f"\n  Moving to sagittal ({x:.0f}, {y:.0f}) mm …")
        move_to_sag(x, y, verbose=True)
        if not sim_only: time.sleep(2); release()

    elif args.goto:
        bx,by,bz = args.goto
        r = np.array([bx,by,bz])
        t1d, xs, ys = hip_ik(r, LEG_INDEX, PARAMS)
        sol = sag_ik(xs, ys, PARAMS)
        if sol:
            tt,tb = sol
            print(f"  Body ({bx:.3f},{by:.3f},{bz:.3f})m → "
                  f"θ1={t1d:+.2f}° θtop={tt:+.2f}° θbot={tb:+.2f}°")
            send_angles(t1d, tt, tb, verbose=True)
            if not sim_only: time.sleep(2); release()
        else:
            print("  Unreachable")

    elif args.raw:
        t1,tt,tb = args.raw
        send_angles(t1, tt, tb, verbose=True)
        if not sim_only: time.sleep(2); release()

    elif args.sweep:
        test_sweep(sim_only=sim_only)

    else:
        interactive(sim_only=sim_only)


if __name__ == '__main__':
    main()
