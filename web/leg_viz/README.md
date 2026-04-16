# Argos — Single Leg Visualization

Standalone 3D web viz for the Argos quadruped's 3-DOF leg. Mirrors the
single-leg bench test in `ros2_ws/argos_control/single_leg_test.py` — same
joint conventions, same geometry, same 90/90/90 servo = neutral stand
reference.

## Usage

### Simulator only (no hardware)

Double-click `index.html`, or serve the directory:

```
cd web/leg_viz
python -m http.server 8000
# then open http://localhost:8000 in a browser
```

Sliders and presets drive the FK in the browser — no robot involvement.

### Driving the real robot from the Pi

Copy this directory to the Pi and run `server.py` — a tiny stdlib HTTP
server that serves the page AND exposes a `/api/servo` endpoint that
writes directly to the PCA9685.

```
scp -r web/leg_viz pi@argos-pi:~/
ssh pi@argos-pi
cd ~/leg_viz
python3 server.py                   # default port 8000
# or:  python3 server.py --sim      # log requests without touching I2C
```

Then, from any browser on the same network, open
`http://argos-pi:8000`, tick **Drive robot** in the top-right, and the
servos will follow the sliders / presets / gait animations. Use
**Release** to zero the duty cycles (servos go limp).

The server uses only the Python stdlib plus `adafruit-pca9685`
(`board`, `busio`) — same packages `single_leg_test.py` already needs.
No Flask, no pip install beyond what's there.

ES modules + an `importmap` pull Three.js 0.160 from unpkg — the first
load needs internet; subsequent loads work offline from cache.

## Controls

- **Orbit** — drag in the 3D view.
- **Sliders** — three joint sliders on the right. Control-space degrees
  on the left, equivalent servo degrees on the right. Ranges clamped to
  the limits in `Config.py`.
- **Preset buttons** (top bar):
  - **Stand** — servo 90/90/90, the neutral pose.
  - **Crouch** — foot lifted toward the body.
  - **Stretch** — near-max leg extension.
  - **Walk** — slow swing/stance cycle (~1.6 s period).
  - **Trot** — fast swing/stance cycle (~0.75 s period).

Dragging any slider cancels a running gait animation.

## How the joints map

| Slider | Range (ctrl) | Servo at 0 | Direction |
|---|---|---|---|
| `theta_1` (hip abductor) | −45° .. +45° | 90° | +1° ctrl → 89° servo (inverted, `HIP_MULTIPLIER = -1`) |
| `theta_top` (upper leg) | −75° .. +10° | 90° | +1° ctrl → 91° servo |
| `theta_bot` (bell-crank) | −20° .. +80° | 90° | +1° ctrl → 91° servo |

90/90/90 servo ⇢ 0/0/0 control ⇢ foot resting ~165 mm below the hip
(the `default_z_ref` stand height from `Config.py`).

## What's rendered

All linkage segments are computed every frame from the same forward
kinematics used on the robot (`_sagittal_fk_state` in `Kinematics.py`):

- Coxa (body hip → femur pivot)
- Femur (pivot → knee)
- Tibia (knee → foot)
- Bell-crank (two 90°-apart arms)
- Bottom servo horn
- Rod 1 (horn → bell-crank right)
- Rod 2 (bell-crank left → tibia attach point above the knee)

If a slider combination makes the bell-crank linkage infeasible (the two
rod circles don't intersect), the last valid frame stays on screen and
the overlay flags it.

## Files

- `index.html` — page shell, sliders, preset buttons, Drive-robot toggle
- `viz.js` — Three.js scene + animation loop + throttled robot POSTs
- `kinematics.js` — pure FK port of `Kinematics.py` (no IK, no ROS)
- `server.py` — optional HTTP bridge for driving the real servos from the Pi

## API (server.py)

| Method | Path | Body | Effect |
|---|---|---|---|
| GET | `/api/status` | — | `{connected, sim}` |
| POST | `/api/servo` | `{hip, top, bot}` (degrees, 0–270) | sets PWM on ch 2 / 1 / 0 |
| POST | `/api/release` | — | zeroes every duty cycle |

The viz throttles POSTs to 20 Hz and skips sends when the pose hasn't
moved, so a running Walk/Trot animation produces ~20 POSTs per second,
not 60.

## Verifying against the robot math

Run the Python self-test to get ground-truth angles/foot positions:

```
python ros2_ws/argos_control/Kinematics.py
```

Drag the sliders to the same `theta_top` / `theta_bot` and compare the
foot position readout (bottom-left overlay) to the Python output. They
should agree to <1 mm.
