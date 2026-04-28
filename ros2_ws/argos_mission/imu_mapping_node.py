"""Fuse /imu/data_raw and /imu/mag into /imu/filtered using a Madgwick filter.

Publishes:
  /imu/filtered        (sensor_msgs/Imu)        — orientation-stamped output
  /imu/euler           (geometry_msgs/Vector3)   — roll/pitch/yaw in degrees
                                                   for easy dashboard display
Subscribes:
  /imu/data_raw        (sensor_msgs/Imu)         — accel + gyro from ESP32
  /imu/mag             (sensor_msgs/MagneticField) — magnetometer from ESP32

The Madgwick algorithm runs entirely in Python (no external C library required)
so it works in the same Docker/devcontainer environment as the rest of the
mission stack. If you later want higher throughput, swap the _MadgwickFilter
class for the `imufusion` or `ahrs` pip package — the node interface stays the same.

Add to ros2_ws/argos_mission/ alongside gas_mapping_node.py and register in
setup.py:
    'imu_fusion_node = argos_mission.imu_fusion_node:main'

Then wire into mission_stack.launch.py the same way gas_mapping_node is wired.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from .mission_contract import TOPICS


# ---------------------------------------------------------------------------
# Pure-Python Madgwick AHRS
# ---------------------------------------------------------------------------

class _MadgwickFilter:
    """Minimal Madgwick AHRS implementation (no external deps).

    Quaternion convention: q = [w, x, y, z].

    References:
        Madgwick, S. (2010). An efficient orientation filter for inertial
        and inertial/magnetic sensor arrays. Report x-io and University of
        Bristol.
    """

    def __init__(self, beta: float = 0.1):
        # beta is the algorithm gain — controls how aggressively the
        # accelerometer/magnetometer correction pulls the estimate.
        # Larger beta = faster convergence but noisier steady state.
        # 0.033 is Madgwick's empirical default; 0.1 is a reasonable
        # starting point for a slow-moving robot.
        self.beta = beta
        # Start with identity quaternion (no rotation)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # ------------------------------------------------------------------
    def update(
        self,
        gyro_xyz: np.ndarray,   # rad/s
        accel_xyz: np.ndarray,  # m/s² (will be normalised internally)
        mag_xyz: np.ndarray,    # any unit (will be normalised internally)
        dt: float,              # seconds since last call
    ) -> np.ndarray:
        """Run one filter step; returns updated quaternion [w, x, y, z]."""
        q = self.q
        gx, gy, gz = gyro_xyz
        ax, ay, az = accel_xyz
        mx, my, mz = mag_xyz

        # --- Normalise accel (skip if near-zero to avoid divide-by-zero) ---
        a_norm = math.sqrt(ax * ax + ay * ay + az * az)
        if a_norm < 1e-10:
            # Can't use accel this tick — gyro-only integration
            self.q = self._integrate_gyro(q, gyro_xyz, dt)
            return self.q
        ax /= a_norm; ay /= a_norm; az /= a_norm

        # --- Normalise mag ---
        m_norm = math.sqrt(mx * mx + my * my + mz * mz)
        if m_norm < 1e-10:
            # Magnetometer reading unusable — fall back to accel-only gradient
            self.q = self._gradient_accel_only(q, gyro_xyz,
                                               np.array([ax, ay, az]), dt)
            return self.q
        mx /= m_norm; my /= m_norm; mz /= m_norm

        # --- Auxiliary variables to avoid repeated arithmetic ---
        q1, q2, q3, q4 = q   # w, x, y, z
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _2q4 = 2.0 * q4
        _2q1q3 = _2q1 * q3
        _2q3q4 = _2q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4

        # --- Reference direction of Earth's magnetic field ---
        hx = (mx * (q1q1 + q2q2 - q3q3 - q4q4)
              + 2.0 * my * (q2q3 - q1q4)
              + 2.0 * mz * (q2q4 + q1q3))
        hy = (2.0 * mx * (q2q3 + q1q4)
              + my * (q1q1 - q2q2 + q3q3 - q4q4)
              + 2.0 * mz * (q3q4 - q1q2))
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = (-2.0 * mx * (q2q4 - q1q3)
                + 2.0 * my * (q3q4 + q1q2)
                + mz * (q1q1 - q2q2 - q3q3 + q4q4))
        _4bx = 2.0 * _2bx
        _4bz = 2.0 * _2bz

        # --- Gradient descent step (objective function + Jacobian) ---
        s1 = (-_2q3 * (2.0 * q2q4 - _2q1q3 - ax)
              + _2q2 * (2.0 * q1q2 + _2q3q4 - ay)
              - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s2 = (_2q4 * (2.0 * q2q4 - _2q1q3 - ax)
              + _2q1 * (2.0 * q1q2 + _2q3q4 - ay)
              - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
              + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s3 = (-_2q1 * (2.0 * q2q4 - _2q1q3 - ax)
              + _2q4 * (2.0 * q1q2 + _2q3q4 - ay)
              - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
              + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        s4 = (_2q2 * (2.0 * q2q4 - _2q1q3 - ax)
              + _2q3 * (2.0 * q1q2 + _2q3q4 - ay)
              + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
              + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
              + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz))

        # Normalise gradient
        s = np.array([s1, s2, s3, s4])
        s_norm = np.linalg.norm(s)
        if s_norm > 1e-10:
            s /= s_norm

        # --- Rate of change of quaternion from gyroscope ---
        qdot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
             q1 * gx + q3 * gz - q4 * gy,
             q1 * gy - q2 * gz + q4 * gx,
             q1 * gz + q2 * gy - q3 * gx,
        ]) - self.beta * s

        q = q + qdot * dt
        self.q = q / np.linalg.norm(q)
        return self.q

    # ------------------------------------------------------------------
    def _integrate_gyro(self, q, gyro_xyz, dt):
        gx, gy, gz = gyro_xyz
        q1, q2, q3, q4 = q
        qdot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
             q1 * gx + q3 * gz - q4 * gy,
             q1 * gy - q2 * gz + q4 * gx,
             q1 * gz + q2 * gy - q3 * gx,
        ])
        q = q + qdot * dt
        return q / np.linalg.norm(q)

    def _gradient_accel_only(self, q, gyro_xyz, accel_unit, dt):
        """Simplified gradient step using only accel (no mag)."""
        ax, ay, az = accel_unit
        q1, q2, q3, q4 = q
        s1 = -2.0 * q3 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) + 2.0 * q2 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay)
        s2 =  2.0 * q4 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) + 2.0 * q1 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay) - 4.0 * q2 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az)
        s3 = -2.0 * q1 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) + 2.0 * q4 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay) - 4.0 * q3 * (1.0 - 2.0 * q2 * q2 - 2.0 * q3 * q3 - az)
        s4 =  2.0 * q2 * (2.0 * q2 * q4 - 2.0 * q1 * q3 - ax) + 2.0 * q3 * (2.0 * q1 * q2 + 2.0 * q3 * q4 - ay)
        s = np.array([s1, s2, s3, s4])
        s_norm = np.linalg.norm(s)
        if s_norm > 1e-10:
            s /= s_norm
        gx, gy, gz = gyro_xyz
        qdot = 0.5 * np.array([
            -q2 * gx - q3 * gy - q4 * gz,
             q1 * gx + q3 * gz - q4 * gy,
             q1 * gy - q2 * gz + q4 * gx,
             q1 * gz + q2 * gy - q3 * gx,
        ]) - self.beta * s
        q = np.array([q1, q2, q3, q4]) + qdot * dt
        return q / np.linalg.norm(q)

    # ------------------------------------------------------------------
    @staticmethod
    def quaternion_to_euler(q: np.ndarray):
        """Convert [w, x, y, z] to (roll, pitch, yaw) in radians."""
        w, x, y, z = q
        roll  = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw   = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class ImuFusionNode(Node):
    """Fuses raw IMU + magnetometer into an orientation estimate."""

    def __init__(self):
        super().__init__("imu_fusion_node")

        # --- Parameters ---
        self.declare_parameter("imu_raw_topic",    TOPICS.imu_raw)
        self.declare_parameter("imu_mag_topic",    TOPICS.imu_mag)
        self.declare_parameter("imu_filtered_topic", TOPICS.imu_filtered)
        self.declare_parameter("imu_euler_topic",  TOPICS.imu_euler)
        self.declare_parameter("output_frame",     "imu_link")
        # Madgwick beta gain — increase if orientation drifts, decrease if noisy
        self.declare_parameter("madgwick_beta",    0.1)
        # Assumed sensor rate; used as fallback dt when timestamps are zero/bad
        self.declare_parameter("nominal_rate_hz",  100.0)

        imu_raw_topic     = self.get_parameter("imu_raw_topic").value
        imu_mag_topic     = self.get_parameter("imu_mag_topic").value
        imu_filtered_topic = self.get_parameter("imu_filtered_topic").value
        imu_euler_topic   = self.get_parameter("imu_euler_topic").value
        self.output_frame = self.get_parameter("output_frame").value
        beta              = float(self.get_parameter("madgwick_beta").value)
        self.nominal_dt   = 1.0 / float(self.get_parameter("nominal_rate_hz").value)

        # --- Filter ---
        self.filter = _MadgwickFilter(beta=beta)
        self._last_imu_stamp_ns: int | None = None

        # --- Latest mag reading (updated independently of IMU) ---
        self._latest_mag = np.zeros(3, dtype=np.float64)

        # --- Publishers ---
        self.filtered_pub = self.create_publisher(Imu,     imu_filtered_topic, 10)
        self.euler_pub    = self.create_publisher(Vector3, imu_euler_topic,    10)

        # --- Subscribers ---
        # Magnetometer comes in on its own topic at up to 100 Hz — just cache it.
        self.create_subscription(MagneticField, imu_mag_topic,
                                 self._mag_callback, 10)
        # IMU drives the filter: every accel+gyro message triggers one update.
        self.create_subscription(Imu, imu_raw_topic,
                                 self._imu_callback, 10)

        self.get_logger().info(
            f"ImuFusionNode started | beta={beta:.4f} | "
            f"raw={imu_raw_topic} mag={imu_mag_topic} "
            f"→ filtered={imu_filtered_topic}"
        )

    # ------------------------------------------------------------------
    def _mag_callback(self, msg: MagneticField):
        self._latest_mag = np.array([
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])

    # ------------------------------------------------------------------
    def _imu_callback(self, msg: Imu):
        # --- Compute dt ---
        stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec))
        if self._last_imu_stamp_ns is None or stamp_ns == 0:
            dt = self.nominal_dt
        else:
            dt = (stamp_ns - self._last_imu_stamp_ns) * 1e-9
            # Sanity clamp: reject implausible dt values (clock jump, reboot, etc.)
            if dt <= 0.0 or dt > 1.0:
                dt = self.nominal_dt
        self._last_imu_stamp_ns = stamp_ns

        # --- Pack sensor vectors ---
        gyro  = np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        mag   = self._latest_mag.copy()  # snapshot; mag callback runs async

        # --- Run Madgwick ---
        q = self.filter.update(gyro, accel, mag, dt)  # [w, x, y, z]

        # --- Publish /imu/filtered ---
        out = Imu()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = self.output_frame

        out.orientation.w = float(q[0])
        out.orientation.x = float(q[1])
        out.orientation.y = float(q[2])
        out.orientation.z = float(q[3])

        # Pass through raw accel + gyro unchanged so downstream consumers
        # (e.g. robot_localization) have everything they need in one message.
        out.linear_acceleration = msg.linear_acceleration
        out.angular_velocity    = msg.angular_velocity

        # Covariance: -1 on orientation signals "unknown" in the raw message;
        # we now have an estimate, so zero it out (unknown variance, not absent).
        # Tune the diagonal values after bench-testing if you feed this into EKF.
        out.orientation_covariance      = [0.0] * 9
        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)

        self.filtered_pub.publish(out)

        # --- Publish /imu/euler (degrees, for dashboard telemetry) ---
        roll, pitch, yaw = _MadgwickFilter.quaternion_to_euler(q)
        euler = Vector3()
        euler.x = math.degrees(roll)
        euler.y = math.degrees(pitch)
        euler.z = math.degrees(yaw)
        self.euler_pub.publish(euler)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()