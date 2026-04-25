// IMU telemetry panel.
//
// Shows roll/pitch/yaw as a small artificial-horizon style dial + numeric
// readouts. The quaternion is converted server-side (pi_server/quaternion.js)
// so the browser just reads `imu._euler_rad`.
//
// The frame_id on these messages is `imu_link` (ESP32 board frame), NOT
// `base_link` — the note stays in the panel so operators don't confuse
// a tilted board with a tilted body.

const RAD2DEG = 180.0 / Math.PI;

export default function ImuPanel({ imu }) {
  const euler = imu?._euler_rad;
  const roll  = Number.isFinite(euler?.roll)  ? euler.roll  * RAD2DEG : null;
  const pitch = Number.isFinite(euler?.pitch) ? euler.pitch * RAD2DEG : null;
  const yaw   = Number.isFinite(euler?.yaw)   ? euler.yaw   * RAD2DEG : null;

  return (
    <section className="imu-panel">
      <header>
        <h3>IMU</h3>
        <span className="muted">frame_id: imu_link (ESP32)</span>
      </header>

      <div className="imu-panel__dial">
        <HorizonDial roll={roll ?? 0} pitch={pitch ?? 0} />
      </div>

      <dl className="imu-panel__readout">
        <div><dt>roll</dt><dd>{fmt(roll)}°</dd></div>
        <div><dt>pitch</dt><dd>{fmt(pitch)}°</dd></div>
        <div><dt>yaw</dt><dd>{fmt(yaw)}°</dd></div>
      </dl>
    </section>
  );
}

function HorizonDial({ roll, pitch }) {
  // Very simple artificial horizon: a circular window with a line tilted
  // by `roll` and offset by `pitch`. Not trying to compete with a real avionics
  // dial — just a visual gestalt so the user sees body orientation at a glance.
  const SIZE = 160;
  const CX = SIZE / 2;
  const CY = SIZE / 2;
  const R = SIZE / 2 - 6;
  const pitchPx = (pitch / 45) * (R * 0.6);
  return (
    <svg width={SIZE} height={SIZE} viewBox={`0 0 ${SIZE} ${SIZE}`}>
      <clipPath id="horizonClip">
        <circle cx={CX} cy={CY} r={R} />
      </clipPath>
      <g clipPath="url(#horizonClip)">
        <rect x={0} y={0} width={SIZE} height={CY + pitchPx}
              fill="#60a5fa" transform={`rotate(${roll} ${CX} ${CY})`} />
        <rect x={0} y={CY + pitchPx} width={SIZE} height={SIZE - (CY + pitchPx)}
              fill="#a16207" transform={`rotate(${roll} ${CX} ${CY})`} />
        <line
          x1={CX - R} x2={CX + R}
          y1={CY + pitchPx} y2={CY + pitchPx}
          stroke="#fff" strokeWidth={2}
          transform={`rotate(${roll} ${CX} ${CY})`}
        />
      </g>
      <circle cx={CX} cy={CY} r={R} stroke="#111" strokeWidth={2} fill="none" />
      <circle cx={CX} cy={CY} r={3} fill="#111" />
    </svg>
  );
}

function fmt(n) {
  return Number.isFinite(n) ? n.toFixed(1) : "—";
}
