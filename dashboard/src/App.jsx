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

import { useCallback, useEffect, useState } from "react";

import AnimationPlayer from "./components/AnimationPlayer.jsx";
import FootXYZPanel from "./components/FootXYZPanel.jsx";
import GasPanel from "./components/GasPanel.jsx";
import ImuPanel from "./components/ImuPanel.jsx";
import JointPanel from "./components/JointPanel.jsx";
import LegDetail from "./components/LegDetail.jsx";
import ModeSelector from "./components/ModeSelector.jsx";
import RobotOverview from "./components/RobotOverview.jsx";
import ServoPanel from "./components/ServoPanel.jsx";
import SettingsDrawer from "./components/SettingsDrawer.jsx";
import StancePresets from "./components/StancePresets.jsx";
import TelemetryRecorder from "./components/TelemetryRecorder.jsx";
import WalkingPanel from "./components/WalkingPanel.jsx";

import { ActiveLegProvider, useActiveLeg } from "./hooks/useActiveLeg.jsx";
import { useCommand } from "./hooks/useCommand.js";
import { useRosbridge } from "./hooks/useRosbridge.js";
import { DEFAULT_ROTATE_SETTINGS } from "../shared/robot-config.js";

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
  const rosConnected = Boolean(data.connected?.ros);
  const connected = wsConnected && rosConnected;
  const rotateSettings = data.rotate_settings || DEFAULT_ROTATE_SETTINGS;

  // Direct-mode flags drive which optional extra panel is shown under the
  // focal leg view. The joint-angle panel is always rendered; these flags
  // are only for Foot XYZ / Servo which stay gated to their explicit modes.
  const mode = data.mode || "idle";
  const isDirectFoot = mode === "direct_foot_xyz";
  const isDirectServo = mode === "direct_servo_angles";

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
    ({ x, z, leg }) => {
      // Called from LegDetail's SVG drag. Andy-servo parity: dragging the
      // foot always moves the leg, even if the user hasn't flipped to the
      // direct-foot panel yet. Auto-enter direct_foot_xyz on first drag
      // so the bridge actually consumes the command. Subsequent drags in
      // the same mode skip this (cheap early-return in setMode for the
      // already-active mode).
      if (mode !== "direct_foot_xyz") {
        cmd.setMode("direct_foot_xyz").catch(() => {});
      }
      // Synthesize a 4-row foot matrix. Andy-servo's drag is sagittal-only
      // (we only know X and Z from the SVG), so Y holds the default stance
      // half-width. Non-dragged legs pass through a safe stance default —
      // the bridge clamps any that stray outside the envelope, so it's
      // safe to send them verbatim.
      const DEFAULT_STANCE = [0.1, 0.1, -0.189];
      const safe = ["FR", "FL", "RR", "RL"].map((legId) => {
        if (legId !== leg) return DEFAULT_STANCE;
        // Convert mm (LegDetail frame) → meters (bridge frame). Preserve
        // Y at stance half-width; the sagittal drag doesn't give us Y.
        const yHalf = legId === "FR" || legId === "RR" ? -0.1 : 0.1;
        return [x / 1000, yHalf, z / 1000];
      });
      cmd.sendFootTargets(safe).catch(() => {});
    },
    [cmd, mode],
  );

  return (
    <div className="app">
      <TopBar
        connected={connected}
        wsConnected={wsConnected}
        rosConnected={rosConnected}
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
          <LegDetail
            activeLeg={activeLeg}
            jointState={data.joint_states}
            mode={mode}
            previewAngles={previewAngles}
            onPreviewAngles={setPreviewAngles}
            onFootDrag={sendFootForActiveLegDrag}
          />

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
            onTwist={(x, y, yaw) => cmd.sendTwist(x, y, yaw).catch(() => {})}
            onZeroTwist={() => cmd.sendTwist(0, 0, 0).catch(() => {})}
            onRotate={(dir, deg) => cmd.rotate(dir, deg).catch(() => {})}
            onGaitParam={(name, value) => cmd.setGaitParam(name, value).catch(() => {})}
          />
        </aside>
      </main>

      {/* Footer row: read-only telemetry + playback controls. Kept out of
          the sidebar so RobotOverview + WalkingPanel breathe, and grouped
          into 3 columns (telemetry / playback / recording) so related
          panels sit next to each other. */}
      <section className="app__footer">
        <div className="app__footer-col">
          <ImuPanel imu={data.imu} />
          <GasPanel gas={data.gas} />
        </div>
        <div className="app__footer-col">
          <StancePresets
            stances={data.stances}
            enabled={connected}
            onSave={(name) => cmd.saveStance(name).catch(() => {})}
            onPlay={(name) => cmd.playStance(name).catch(() => {})}
          />
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
        servoOverrides={data.servo_overrides}
        jointLimits={data.joint_limits}
        rotateSettings={rotateSettings}
        servoSpeed={data.servo_speed}
        servoUpdateRate={data.servo_update_rate}
        onSaveServoOverrides={(v) => cmd.setServoOverrides(v).catch(() => {})}
        onSaveJointLimits={(v) => cmd.setJointLimits(v).catch(() => {})}
        onSaveRotateSettings={(v) => cmd.setRotateSettings(v).catch(() => {})}
        onSaveStabilizerParams={(v) => cmd.setStabilizerParams(v).catch(() => {})}
        onSaveGaitParam={(n, v) => cmd.setGaitParam(n, v).catch(() => {})}
        onSaveServoSpeed={(v) => cmd.setServoSpeedLimits(v).catch(() => {})}
        onSaveServoUpdateRate={(v) => cmd.setServoUpdateRate(v).catch(() => {})}
      />

      {cmd.lastError && (
        <div className="app__error-banner" role="alert">
          <span>{cmd.lastError}</span>
        </div>
      )}
    </div>
  );
}

function TopBar({ connected, wsConnected, rosConnected, mode, gaitMode, onOpenSettings, onDisconnect }) {
  return (
    <header className="app__topbar">
      <div className="app__brand">
        <h1>Argos</h1>
        <span className={`pill${connected ? " is-ok" : " is-bad"}`}>
          {wsConnected ? "pi ✓" : "pi ✕"} · {rosConnected ? "ros ✓" : "ros ✕"}
        </span>
        <span className="pill">mode {mode}</span>
        {gaitMode && <span className="pill">/gait_mode {gaitMode}</span>}
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
