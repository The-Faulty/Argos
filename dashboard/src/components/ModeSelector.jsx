// Top-bar mode chips + red Disconnect button.
//
// Clicking a chip posts `set_mode` to the Pi. The Disconnect button lives
// next door because it's a distinct hazard class — losing a mode chip
// in a muscle-memory click during a demo shouldn't be confusable with
// pulling power. We style it red + always keep it outside the chip row.

import { MODE_OPTIONS } from "../../shared/robot-config.js";

const CHIP_LABELS = {
  idle: "Idle",
  stand: "Stand",
  crouch: "Crouch",
  extend: "Extend",
  crawl: "Crawl",
  trot: "Trot",
  direct_foot_xyz: "Foot XYZ",
  direct_joint_angles: "Joint",
  direct_servo_angles: "Servo",
  animation_playback: "Animation",
};

export default function ModeSelector({ mode, onSetMode, onDisconnect, connected }) {
  return (
    <div className="mode-selector">
      <div className="mode-selector__chips" role="radiogroup" aria-label="Gait mode">
        {MODE_OPTIONS.map((name) => (
          <button
            key={name}
            type="button"
            role="radio"
            aria-checked={mode === name}
            className={`mode-chip${mode === name ? " is-active" : ""}`}
            disabled={!connected}
            onClick={() => onSetMode(name)}
          >
            {CHIP_LABELS[name] ?? name}
          </button>
        ))}
      </div>

      <button
        type="button"
        className="mode-selector__disconnect"
        onClick={onDisconnect}
        disabled={!connected}
        title="Publish /release_servos=true and /estop=true"
      >
        Disconnect
      </button>
    </div>
  );
}
