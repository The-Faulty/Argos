// Top-level app shell.
//
// Layout follows the "one big leg + four mini overview" pattern the user
// explicitly chose over andy-servo's multi-card grid:
//
//   ┌───── Header: status pill, settings gear, Disconnect ─────┐
//   ├─── ModeSelector chips + Rotate L/R ──────────────────────┤
//   ├──── Main ────────────────────────┬── Sidebar ────────────┤
//   │  LegDetail (selected leg)        │  RobotOverview        │
//   │                                  │  WalkingPanel         │
//   │  active-mode direct panel        │  IMU  /  Gas          │
//   │  (JointPanel / FootXYZ / Servo)  │  StancePresets        │
//   │                                  │  AnimationPlayer      │
//   │                                  │  TelemetryRecorder    │
//   └──────────────────────────────────┴───────────────────────┘
//
// The useRosbridge hook owns the WS and reduces incoming telemetry into
// a flat `data` object that every component consumes. The useCommand hook
// is the only source of server-side side effects. Local app state here
// is limited to: active leg, whether the settings drawer is open, and
// last-error banner toggle.

import { useCallback, useEffect, useRef, useState } from "react";

import AnimationPlayer from "./components/AnimationPlayer.jsx";
import CameraPanel from "./components/CameraPanel.jsx";
import FootXYZPanel from "./components/FootXYZPanel.jsx";
import GasPanel from "./components/GasPanel.jsx";
import ImuPanel from "./components/ImuPanel.jsx";
import JointPanel from "./components/JointPanel.jsx";
import LegDetail, { leg_angle_map } from "./components/LegDetail.jsx";
import ModeSelector from "./components/ModeSelector.jsx";
import RobotOverview from "./components/RobotOverview.jsx";
import ServoPanel from "./components/ServoPanel.jsx";
import SettingsDrawer from "./components/SettingsDrawer.jsx";
import StancePresets from "./components/StancePresets.jsx";
import ThermalPanel from "./components/ThermalPanel.jsx";
import TelemetryRecorder from "./components/TelemetryRecorder.jsx";
import WalkingPanel from "./components/WalkingPanel.jsx";

import { ActiveLegProvider, useActiveLeg } from "./hooks/useActiveLeg.jsx";
import { useCommand } from "./hooks/useCommand.js";
import { useRosbridge } from "./hooks/useRosbridge.js";
import {
  DEFAULT_ROTATE_SETTINGS,
  DEFAULT_STANCE,
  JOINT_NAMES,
  LEG_IDS,
  LEG_ORIGINS,
} from "../shared/robot-config.js";

export default function App() {
  return (
    <ActiveLegProvider>
      <AppBody />
    </ActiveLegProvider>
  );
}

function AppBody() {
  const { data } = useRosbridge();
  const cmd = useCommand();
  const { activeLeg, setActiveLeg } = useActiveLeg();
  const [settingsOpen, setSettingsOpen] = useState(false);

  // Shared optimistic preview for the active leg so LegDetail (foot drag)
  // and JointPanel (sliders) move in lock-step without a /joint_states
  // round-trip. Shape: { leg: "FR", angles: [coxa, femur, tibia] } (rad).
  // Cleared on activeLeg change so the new focal leg always starts from
  // its true telemetry.
  const [previewAngles, setPreviewAngles] = useState(null);
  useEffect(() => setPreviewAngles(null), [activeLeg]);

  // Joystick auto-mode-switch dedupe. The WalkingPanel heartbeat re-emits
  // the held twist every 100 ms; until the Pi telemetry confirms the mode
  // flipped, our `mode` state still reads the old value and the conditional
  // below would re-fire `setMode("trot")` on every heartbeat. The ref
  // gates that — true while a switch is in flight, cleared on stick release
  // OR on a real mode transition (the useEffect below).
  const trotRequested = useRef(false);

  // LegDetail emits a window event when its in-header selector changes,
  // since that subcomponent lives outside the provider tree by design.
  useEffect(() => {
    function onPick(event) {
      const next = event.detail;
      if (["FR", "FL", "RR", "RL"].includes(next)) setActiveLeg(next);
    }
    window.addEventListener("argos:set-active-leg", onPick);
    return () => window.removeEventListener("argos:set-active-leg", onPick);
  }, [setActiveLeg]);

  const wsConnected = Boolean(data.connected?.ws);
  const serialConnected = Boolean(data.connected?.serial);
  const sensorsConnected = Boolean(data.connected?.sensors);
  // The dashboard is "connected" enough to send commands once the browser↔Pi
  // socket is up and the ESP32 serial link is open. Sensors are optional —
  // camera/IMU/thermal can be missing without blocking servo control.
  const connected = wsConnected && serialConnected;
  const rotateSettings = data.rotate_settings || DEFAULT_ROTATE_SETTINGS;

  // Direct-mode flags drive which optional extra panel is shown under the
  // focal leg view. The joint-angle panel is always rendered; these flags
  // are only for Foot XYZ / Servo which stay gated to their explicit modes.
  const mode = data.mode || "idle";
  const isDirectFoot = mode === "direct_foot_xyz";
  const isDirectServo = mode === "direct_servo_angles";

  // Real Pi-side mode transition → drop the in-flight setMode gate so the
  // next "stick from non-trot" push is allowed to issue a fresh setMode.
  useEffect(() => {
    trotRequested.current = false;
  }, [mode]);

  // Joint-slider dispatcher used by the "always-on" JointPanel sitting right
  // under LegDetail. Same auto-switch pattern as the foot drag: flipping to
  // direct_joint_angles on first interaction is what makes the bridge
  // actually consume the command. If we're already in that mode, setMode
  // is a fast no-op on the Pi.
  const sendJointAnglesForActiveLeg = useCallback(
    (angles) => {
      if (mode !== "direct_joint_angles") {
        cmd.setMode("direct_joint_angles").catch(() => {});
      }
      cmd.sendJointAngles(angles).catch(() => {});
    },
    [cmd, mode],
  );

  const sendFootForActiveLegDrag = useCallback(
    ({ x, z, leg, angles }) => {
      // Called from LegDetail's SVG drag. Andy-servo parity: dragging the
      // foot always moves the leg, even if the user hasn't flipped to a
      // direct panel yet. LegDetail's SVG is driven by the bell-crank angle
      // solve, so use those solved joint angles as the command path. Sending
      // the SVG's local display millimeters as body-frame foot targets puts z
      // near the hip instead of near the stance depth and IK refuses it.
      if (Array.isArray(angles) && angles.length === 3) {
        if (mode !== "direct_joint_angles") {
          cmd.setMode("direct_joint_angles").catch(() => {});
        }
        const next = readFullJointArray(data.joint_states);
        const legIndex = LEG_IDS.indexOf(leg);
        if (legIndex >= 0) {
          for (let i = 0; i < 3; i++) next[legIndex * 3 + i] = angles[i];
          cmd.sendJointAngles(next).catch(() => {});
        }
        return;
      }

      // Fallback for older drag payloads that only include an x/z target.
      if (mode !== "direct_foot_xyz") {
        cmd.setMode("direct_foot_xyz").catch(() => {});
      }
      // Synthesize a 4-row foot matrix. Andy-servo's drag is sagittal-only
      // (we only know X and Z from the SVG), so each non-dragged leg holds
      // its proper per-leg stance position from robot-config (front legs
      // forward, rear legs back, left legs +Y, right legs -Y). Sending
      // [0.1, 0.1, ...] for every leg used to fail IK silently for FR/RR/RL
      // because those legs are physically not at +X/+Y.
      const safe = LEG_IDS.map((legId) => {
        const stance = DEFAULT_STANCE[legId];
        if (legId !== leg) return stance;
        const legIndex = LEG_IDS.indexOf(legId);
        const hipOriginX = legIndex >= 0 ? LEG_ORIGINS[0][legIndex] : 0;
        const hipOriginZ = legIndex >= 0 ? LEG_ORIGINS[2][legIndex] : 0;
        // Active leg: LegDetail drag reports hip-local sagittal X/Z in mm.
        // Convert back to body frame before sending the 4-leg target matrix,
        // while keeping Y at the leg's own stance half-width since the drag
        // UI does not provide lateral input.
        return [
          hipOriginX + x / 1000,
          stance[1],
          hipOriginZ + z / 1000,
        ];
      });
      cmd.sendFootTargets(safe).catch(() => {});
    },
    [cmd, data.joint_states, mode],
  );

  return (
    <div className="app">
      <TopBar
        connected={connected}
        wsConnected={wsConnected}
        serialConnected={serialConnected}
        sensorsConnected={sensorsConnected}
        mode={mode}
        gaitMode={data.gait_mode?.data}
        onOpenSettings={() => setSettingsOpen(true)}
        onDisconnect={() => cmd.disconnect().catch(() => {})}
      />

      <ModeSelector
        mode={mode}
        connected={connected}
        onSetMode={(m) => cmd.setMode(m).catch(() => {})}
        onDisconnect={() => cmd.disconnect().catch(() => {})}
      />

      <main className="app__main">
        <section className="app__focus">
          <div className="app__leg-workspace">
            <LegDetail
              activeLeg={activeLeg}
              jointState={data.joint_states}
              mode={mode}
              previewAngles={previewAngles}
              onPreviewAngles={setPreviewAngles}
              onFootDrag={sendFootForActiveLegDrag}
            />

            <div className="app__leg-controls">
              {/* Always-on joint sliders for the active leg. On first drag the
                  dispatcher auto-switches to direct_joint_angles so the bridge
                  consumes the command; no user mode-flip required. */}
              <JointPanel
                activeLeg={activeLeg}
                jointState={data.joint_states}
                previewAngles={previewAngles}
                onPreviewAngles={setPreviewAngles}
                onSendAngles={sendJointAnglesForActiveLeg}
              />

              {/* Mode-specific extras (shown only when the user has explicitly
                  entered that direct mode from the chip strip). */}
              {(isDirectFoot || isDirectServo) && (
                <div className="app__focus-panel">
                  {isDirectFoot && (
                    <FootXYZPanel
                      activeLeg={activeLeg}
                      enabled={connected}
                      onSendFootTargets={(matrix) => cmd.sendFootTargets(matrix).catch(() => {})}
                    />
                  )}
                  {isDirectServo && (
                    <ServoPanel
                      activeLeg={activeLeg}
                      jointState={data.joint_states}
                      enabled={connected}
                      onSendServoAngles={(angles_deg) =>
                        cmd.sendServoAngles(angles_deg).catch(() => {})
                      }
                    />
                  )}
                </div>
              )}
            </div>
          </div>
        </section>

        <aside className="app__sidebar">
          <RobotOverview
            activeLeg={activeLeg}
            onSelectLeg={setActiveLeg}
            jointState={data.joint_states}
          />
          <WalkingPanel
            mode={mode}
            rotateIncrement={rotateSettings.rotate_increment_deg}
            stepLength={data.gait_params?.delta_x_mm}
            swingTimeMs={data.gait_params?.swing_time_ms}
            onTwist={async (x, y, yaw) => {
              // Auto-switch to trot on first non-zero input (only crawl/trot
              // consume the twist; from idle/stand/crouch the planner would
              // silently drop it). The trotRequested ref makes this idempotent
              // across the WalkingPanel heartbeat — without it, every 100 ms
              // re-emit while mode telemetry hasn't caught up was firing a
              // fresh `setMode("trot")` HTTP call. The ref is cleared on a
              // real Pi-side mode transition (useEffect on `mode` above) and
              // on stick release (below).
              const moving = x !== 0 || y !== 0 || yaw !== 0;
              if (
                moving &&
                mode !== "trot" &&
                mode !== "crawl" &&
                !trotRequested.current
              ) {
                trotRequested.current = true;
                try { await cmd.setMode("trot"); } catch { /* fall through */ }
              }
              if (!moving) trotRequested.current = false;
              cmd.sendTwist(x, y, yaw).catch(() => {});
            }}
            onZeroTwist={() => {
              trotRequested.current = false;
              cmd.sendTwist(0, 0, 0).catch(() => {});
            }}
            onRotate={(dir, deg) => cmd.rotate(dir, deg).catch(() => {})}
            onGaitParam={(name, value) => cmd.setGaitParam(name, value).catch(() => {})}
          />
          <StancePresets
            stances={data.stances}
            enabled={connected}
            onSave={(name) => cmd.saveStance(name).catch(() => {})}
            onPlay={(name) => cmd.playStance(name).catch(() => {})}
          />
        </aside>
      </main>

      {/* Footer row: read-only telemetry, media, playback, and recording. */}
      <section className="app__footer">
        <div className="app__footer-col">
          <ImuPanel imu={data.imu} />
          <GasPanel gas={data.gas} />
        </div>
        <div className="app__footer-col">
          <CameraPanel />
          <ThermalPanel thermal={data.thermal} />
        </div>
        <div className="app__footer-col">
          <AnimationPlayer
            enabled={connected}
            onPlay={(name) =>
              cmd.sendCommand({ type: "animation_play", name }).catch(() => {})
            }
            onStop={() => cmd.sendCommand({ type: "animation_stop" }).catch(() => {})}
            onSendFootTargets={(matrix) => cmd.sendFootTargets(matrix).catch(() => {})}
          />
        </div>
        <div className="app__footer-col">
          <TelemetryRecorder
            recording={data.recording}
            onStart={() => cmd.startRecording().catch(() => {})}
            onStop={() => cmd.stopRecording().catch(() => {})}
            onList={() => cmd.listRecordings()}
          />
        </div>
      </section>

      <SettingsDrawer
        open={settingsOpen}
        onClose={() => setSettingsOpen(false)}
        rotateSettings={rotateSettings}
        onSaveRotateSettings={(v) => cmd.setRotateSettings(v).catch(() => {})}
        onSaveStabilizerParams={(v) => cmd.setStabilizerParams(v).catch(() => {})}
        onSaveGaitParam={(n, v) => cmd.setGaitParam(n, v).catch(() => {})}
      />

      {cmd.lastError && (
        <div className="app__error-banner" role="alert">
          <span>{cmd.lastError}</span>
        </div>
      )}
    </div>
  );
}

function readFullJointArray(jointState) {
  const bag = leg_angle_map(jointState);
  return JOINT_NAMES.map((name) => {
    const rad = bag[name];
    return Number.isFinite(rad) ? rad : 0;
  });
}

function TopBar({ connected, wsConnected, serialConnected, sensorsConnected, mode, gaitMode, onOpenSettings, onDisconnect }) {
  return (
    <header className="app__topbar">
      <div className="app__brand">
        <h1>Argos</h1>
        <span className={`pill${connected ? " is-ok" : " is-bad"}`}>
          {wsConnected ? "pi ✓" : "pi ✕"} · {serialConnected ? "esp ✓" : "esp ✕"} · {sensorsConnected ? "sensors ✓" : "sensors ✕"}
        </span>
        <span className="pill">mode {mode}</span>
        {gaitMode && <span className="pill">gait {gaitMode}</span>}
      </div>
      <div className="app__topbar-actions">
        <button type="button" className="ghost-btn" onClick={onOpenSettings}>
          ⚙ Settings
        </button>
        <button type="button" className="danger-btn" onClick={onDisconnect}>
          🛑 Disconnect
        </button>
      </div>
    </header>
  );
}
