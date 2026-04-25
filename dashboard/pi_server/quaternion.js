// Minimal quaternion → Euler (roll, pitch, yaw) conversion.
//
// ROS 2 IMU messages expose `orientation` as {x, y, z, w}. The dashboard's
// IMU panel shows roll/pitch/yaw in degrees; we'd rather convert once on the
// server and broadcast pre-resolved Euler than ship the full quaternion to
// every connected browser.
//
// Convention: ZYX intrinsic (yaw, pitch, roll), matching
// transforms3d.euler.quat2euler's default. This is the same convention
// `ros_support.euler_from_imu` uses on the Python side, so if the Pi bridge
// and this server disagree on roll/pitch/yaw conventions, that's a bug.

// Returns [roll, pitch, yaw] in radians. Returns zeros if the quaternion
// is unpopulated (|q| near 0) — matches the Python fallback behavior.
export function euler_from_quaternion(q) {
  if (!q) return [0, 0, 0];
  const x = Number(q.x) || 0;
  const y = Number(q.y) || 0;
  const z = Number(q.z) || 0;
  const w = Number(q.w) || 0;
  const norm = Math.hypot(x, y, z, w);
  if (norm < 1e-9) return [0, 0, 0];
  const nx = x / norm;
  const ny = y / norm;
  const nz = z / norm;
  const nw = w / norm;

  // roll (x-axis rotation)
  const sinr_cosp = 2 * (nw * nx + ny * nz);
  const cosr_cosp = 1 - 2 * (nx * nx + ny * ny);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation) — clamped to avoid NaN at gimbal singularities
  const sinp = 2 * (nw * ny - nz * nx);
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * (Math.PI / 2)
    : Math.asin(sinp);

  // yaw (z-axis rotation)
  const siny_cosp = 2 * (nw * nz + nx * ny);
  const cosy_cosp = 1 - 2 * (ny * ny + nz * nz);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return [roll, pitch, yaw];
}
