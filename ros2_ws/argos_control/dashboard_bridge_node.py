"""Translate dashboard intents into the existing Argos ROS control topics.

This node is the Pi-side bridge for the Node dashboard server (talking via
rosbridge_websocket). It does NOT own real-time control — it just turns
dashboard messages into /joint_command/raw publishes so foothold_checker and
safety_node keep enforcing joint limits. The gait_planner_node listens to the
same /control_mode topic and stops publishing raw commands while we are in
manual mode, so both sources never fight for the bus at once.

Why /joint_command/raw and not /joint_command: the latter is the MCU output
from joint_command_publisher_node — publishing there would bypass the safety
pipeline and is unsafe.
"""

import json
import os
import tempfile
import threading
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .Config import Configuration
from .Kinematics import four_legs_inverse_kinematics
from .ros_support import JOINT_NAMES, LEG_ORDER, TOPICS, joint_state_from_positions


# Persistent state lives under ~/.argos so the dashboard survives reboots.
ARGOS_STATE_DIR = Path(os.path.expanduser("~/.argos"))
SERVO_OVERRIDES_PATH = ARGOS_STATE_DIR / "servo_overrides.json"
STANCES_PATH = ARGOS_STATE_DIR / "stances.json"
SERVO_SPEED_PATH = ARGOS_STATE_DIR / "servo_speed_limits.json"
JOINT_LIMITS_PATH = ARGOS_STATE_DIR / "joint_limits.json"

# Map from joint-row suffix to the (3, 4, 2) joint_limits_per_leg_rad row index.
_ROW_INDEX = {"coxa": 0, "femur": 1, "tibia": 2}
# Map from LEG_ORDER id → matrix column index.
_LEG_INDEX = {leg_id: idx for idx, leg_id in enumerate(LEG_ORDER)}

# Per-joint fallback speed ceiling (deg/s in servo-horn space), keyed by the
# joint-row suffix. Mirrors DEFAULT_SERVO_SPEED_LIMIT_DEG_PER_SEC in
# dashboard/shared/robot-config.js — keep in sync.
_DEFAULT_SPEED_LIMITS_DEG_PER_SEC = {"coxa": 180.0, "femur": 240.0, "tibia": 300.0}
_DEG_TO_RAD = np.pi / 180.0

# Fallback stance library — written to disk the first time the node runs so
# the dashboard always has something to play. Angles are radians, ordered by
# ros_support.JOINT_NAMES.
DEFAULT_STANCES = {
    "crouch": [0.0] * 12,
    "stand": [0.0, -0.10, 0.30] * 4,
}


class DashboardBridgeNode(Node):
    """Bridges dashboard websocket traffic into the Argos control topics."""

    def __init__(self):
        super().__init__("dashboard_bridge_node")

        self.config = Configuration()
        self.control_mode = "auto"

        # Lock guards both the override table and its file on disk so writes
        # from /dashboard/servo_overrides don't race with reads during IK.
        self._override_lock = threading.Lock()
        self.servo_overrides = self._load_servo_overrides()
        self._log_override_table()

        # Per-leg user joint limits (radians). Loaded from disk; live-updated
        # by /dashboard/joint_limits. Structure: {joint_name: (min_rad, max_rad)}.
        # These can only TIGHTEN the Config.py defaults — the coupled bell-crank
        # clamp via config.clamp_joint_matrix still runs after this to enforce
        # the hardware envelope.
        self._limits_lock = threading.Lock()
        self.joint_limits_rad = self._load_joint_limits()
        self._log_joint_limits()

        # Per-joint speed ceilings (rad/s). Loaded from disk on boot, updated
        # live by /dashboard/servo_speed_limits. Publish path tracks
        # _last_published_positions + timestamp and clamps deltas per-joint
        # before publishing; a zero ceiling disables rate-limiting for that
        # joint.
        self._speed_lock = threading.Lock()
        self.servo_speed_limits_rad_per_sec = self._load_speed_limits()
        self._last_published_positions: np.ndarray | None = None
        self._last_published_ns: int | None = None
        self._log_speed_limits()

        # Publisher into the regular safety pipeline
        self.raw_joint_pub = self.create_publisher(
            JointState, TOPICS.joint_command_raw, 10
        )

        # Subscribers
        self.create_subscription(
            String, TOPICS.control_mode, self._control_mode_callback, 10
        )
        self.create_subscription(
            PoseArray,
            TOPICS.dashboard_foot_targets,
            self._foot_targets_callback,
            10,
        )
        self.create_subscription(
            JointState,
            TOPICS.dashboard_joint_angles,
            self._joint_angles_callback,
            10,
        )
        self.create_subscription(
            String,
            TOPICS.dashboard_servo_overrides,
            self._servo_overrides_callback,
            10,
        )
        self.create_subscription(
            String,
            TOPICS.dashboard_joint_limits,
            self._joint_limits_callback,
            10,
        )
        self.create_subscription(
            String, TOPICS.dashboard_stance_play, self._stance_play_callback, 10
        )
        self.create_subscription(
            String,
            TOPICS.dashboard_servo_speed_limits,
            self._servo_speed_limits_callback,
            10,
        )

        self.get_logger().info(
            "dashboard_bridge_node ready (manual-mode publishes /joint_command/raw)."
        )

    # ── State persistence ────────────────────────────────────────────────

    def _load_servo_overrides(self) -> dict:
        """Read the override table from disk, creating an empty file if missing."""
        ARGOS_STATE_DIR.mkdir(parents=True, exist_ok=True)
        if not SERVO_OVERRIDES_PATH.exists():
            self._atomic_write_json(SERVO_OVERRIDES_PATH, {})
            return {}
        try:
            with SERVO_OVERRIDES_PATH.open("r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else {}
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().warning(
                f"Could not read {SERVO_OVERRIDES_PATH}: {exc}; starting empty."
            )
            return {}

    def _load_stances(self) -> dict:
        """Read the stance library from disk, seeding defaults if missing."""
        ARGOS_STATE_DIR.mkdir(parents=True, exist_ok=True)
        if not STANCES_PATH.exists():
            self._atomic_write_json(STANCES_PATH, DEFAULT_STANCES)
            return dict(DEFAULT_STANCES)
        try:
            with STANCES_PATH.open("r", encoding="utf-8") as f:
                data = json.load(f)
            return data if isinstance(data, dict) else dict(DEFAULT_STANCES)
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().warning(
                f"Could not read {STANCES_PATH}: {exc}; using defaults."
            )
            return dict(DEFAULT_STANCES)

    def _load_joint_limits(self) -> dict:
        """Read the per-joint user limits from disk.

        File format: `{joint_name: {"min_rad": float, "max_rad": float}}`.
        Missing joints fall back to `config.joint_limits_per_leg_rad` at apply
        time, so an empty file is valid (and means "use firmware defaults").
        """
        ARGOS_STATE_DIR.mkdir(parents=True, exist_ok=True)
        if not JOINT_LIMITS_PATH.exists():
            self._atomic_write_json(JOINT_LIMITS_PATH, {})
            return {}
        try:
            with JOINT_LIMITS_PATH.open("r", encoding="utf-8") as f:
                data = json.load(f)
        except (json.JSONDecodeError, OSError) as exc:
            self.get_logger().warning(
                f"Could not read {JOINT_LIMITS_PATH}: {exc}; starting empty."
            )
            return {}
        if not isinstance(data, dict):
            return {}
        return self._normalize_joint_limits(data)

    @staticmethod
    def _normalize_joint_limits(raw: dict) -> dict:
        """Coerce a dashboard joint_limits payload into {name: (lo, hi)} radians.

        Accepts both `{"min_rad": x, "max_rad": y}` (preferred) and
        `{"min_deg": x, "max_deg": y}` (what the dashboard Settings drawer
        sends today) so the ROS side is tolerant of either unit.
        """
        out: dict[str, tuple[float, float]] = {}
        for name, entry in raw.items():
            if name not in JOINT_NAMES or not isinstance(entry, dict):
                continue
            lo = entry.get("min_rad")
            hi = entry.get("max_rad")
            if lo is None and "min_deg" in entry:
                try:
                    lo = float(entry["min_deg"]) * _DEG_TO_RAD
                except (TypeError, ValueError):
                    lo = None
            if hi is None and "max_deg" in entry:
                try:
                    hi = float(entry["max_deg"]) * _DEG_TO_RAD
                except (TypeError, ValueError):
                    hi = None
            if lo is None or hi is None:
                continue
            try:
                lo_f, hi_f = float(lo), float(hi)
            except (TypeError, ValueError):
                continue
            if lo_f > hi_f:
                lo_f, hi_f = hi_f, lo_f
            out[name] = (lo_f, hi_f)
        return out

    @staticmethod
    def _atomic_write_json(path: Path, payload: dict) -> None:
        """Write JSON via tempfile + os.replace so a crash can't truncate the file."""
        path.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(
            prefix=f".{path.name}.", dir=str(path.parent)
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, sort_keys=True)
                f.flush()
                try:
                    os.fsync(f.fileno())
                except OSError:
                    # fsync may not work on all filesystems — best-effort only
                    pass
            os.replace(tmp_path, path)
        except Exception:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise

    def _log_override_table(self) -> None:
        if not self.servo_overrides:
            self.get_logger().info("Override table: empty (all joints default).")
            return
        lines = [f"  {name}: {entry}" for name, entry in self.servo_overrides.items()]
        self.get_logger().info("Override table:\n" + "\n".join(lines))

    def _log_joint_limits(self) -> None:
        if not self.joint_limits_rad:
            self.get_logger().info("Joint limits: using firmware defaults.")
            return
        lines = [
            f"  {name}: [{lo / _DEG_TO_RAD:+.1f}, {hi / _DEG_TO_RAD:+.1f}] deg"
            for name, (lo, hi) in self.joint_limits_rad.items()
        ]
        self.get_logger().info("Joint limits (per-joint overrides):\n" + "\n".join(lines))

    def _load_speed_limits(self) -> dict:
        """Read the speed-limit table (deg/s per joint) and convert to rad/s.

        Missing joints fall back to _DEFAULT_SPEED_LIMITS_DEG_PER_SEC by row.
        """
        table_deg: dict = {}
        if SERVO_SPEED_PATH.exists():
            try:
                with SERVO_SPEED_PATH.open("r", encoding="utf-8") as f:
                    raw = json.load(f)
                if isinstance(raw, dict):
                    table_deg = raw
            except (json.JSONDecodeError, OSError) as exc:
                self.get_logger().warning(
                    f"Could not read {SERVO_SPEED_PATH}: {exc}; using defaults."
                )

        result: dict[str, float] = {}
        for name in JOINT_NAMES:
            if name in table_deg:
                try:
                    deg_per_sec = float(table_deg[name])
                except (TypeError, ValueError):
                    deg_per_sec = 0.0
            else:
                row = name.rsplit("_", 2)[-2] if "_" in name else ""
                deg_per_sec = _DEFAULT_SPEED_LIMITS_DEG_PER_SEC.get(row, 0.0)
            result[name] = max(0.0, deg_per_sec) * _DEG_TO_RAD
        return result

    def _log_speed_limits(self) -> None:
        if not self.servo_speed_limits_rad_per_sec:
            return
        # Summarize by joint row to keep the log short.
        rows: dict[str, list[float]] = {"coxa": [], "femur": [], "tibia": []}
        for name, rad_per_sec in self.servo_speed_limits_rad_per_sec.items():
            for row in rows:
                if name.endswith(f"_{row}_joint"):
                    rows[row].append(rad_per_sec / _DEG_TO_RAD)
                    break
        summary = ", ".join(
            f"{row}={sum(vals) / len(vals):.0f} deg/s"
            for row, vals in rows.items()
            if vals
        )
        self.get_logger().info(f"Servo speed limits: {summary}")

    # ── Envelope clamp (per-leg user limits + coupled bell-crank) ────────

    def _clamp_envelope(self, positions: np.ndarray) -> np.ndarray:
        """Reshape (12,) → (3, 4), apply user limits, then coupled clamp.

        Order matters: user per-joint limits can only TIGHTEN the window, then
        `config.clamp_joint_matrix` enforces the hardware coupled bell-crank
        envelope so we never command a physically infeasible triple. This
        keeps `safety_node`'s guarantees intact for direct-joint-angle mode,
        which otherwise would bypass the coupled check (safety_node sits on
        the pre-mux channel; the dashboard speaks post-mux).
        """
        flat = np.asarray(positions, dtype=float).reshape(-1)
        if flat.size != len(JOINT_NAMES):
            raise ValueError(
                f"Expected {len(JOINT_NAMES)} joint values, got {flat.size}"
            )

        # (3 rows × 4 legs). JOINT_NAMES is leg-major (FR_coxa, FR_femur,
        # FR_tibia, FL_coxa, ...) so matrix[row, leg] = flat[leg*3 + row].
        matrix = np.zeros((3, 4), dtype=float)
        for idx, name in enumerate(JOINT_NAMES):
            leg_id, row_name, _ = name.split("_", 2)
            row = _ROW_INDEX[row_name]
            leg = _LEG_INDEX[leg_id]
            matrix[row, leg] = flat[idx]

        # Apply user per-joint tightening (radians). Absent joints fall back
        # to the Config.py defaults baked into joint_limits_per_leg_rad.
        with self._limits_lock:
            user_limits = dict(self.joint_limits_rad)
        fw_limits = self.config.joint_limits_per_leg_rad  # (3, 4, 2)
        for idx, name in enumerate(JOINT_NAMES):
            leg_id, row_name, _ = name.split("_", 2)
            row = _ROW_INDEX[row_name]
            leg = _LEG_INDEX[leg_id]
            fw_lo, fw_hi = float(fw_limits[row, leg, 0]), float(fw_limits[row, leg, 1])
            if name in user_limits:
                u_lo, u_hi = user_limits[name]
                lo = max(fw_lo, u_lo)
                hi = min(fw_hi, u_hi)
            else:
                lo, hi = fw_lo, fw_hi
            if lo > hi:
                mid = 0.5 * (lo + hi)
                lo = hi = mid
            matrix[row, leg] = float(np.clip(matrix[row, leg], lo, hi))

        # Final coupled bell-crank envelope (hardware truth).
        matrix = self.config.clamp_joint_matrix(matrix)

        # Flatten back in JOINT_NAMES order.
        out = np.zeros(len(JOINT_NAMES), dtype=float)
        for idx, name in enumerate(JOINT_NAMES):
            leg_id, row_name, _ = name.split("_", 2)
            row = _ROW_INDEX[row_name]
            leg = _LEG_INDEX[leg_id]
            out[idx] = matrix[row, leg]
        return out

    # ── Overrides ────────────────────────────────────────────────────────

    def _apply_overrides(self, positions: np.ndarray) -> np.ndarray:
        """Apply per-joint invert/offset from the override table.

        Formula per joint: invert ? -a + offset : a + offset (radians).
        """
        with self._override_lock:
            overrides = dict(self.servo_overrides)
        out = np.asarray(positions, dtype=float).copy()
        for idx, name in enumerate(JOINT_NAMES):
            entry = overrides.get(name)
            if not entry:
                continue
            invert = bool(entry.get("invert", False))
            offset = float(entry.get("offset_rad", 0.0))
            value = -out[idx] if invert else out[idx]
            out[idx] = value + offset
        return out

    # ── Subscriber callbacks ─────────────────────────────────────────────

    def _control_mode_callback(self, msg: String):
        mode = msg.data.strip().lower()
        if mode not in {"auto", "manual"}:
            return
        if mode == self.control_mode:
            return
        self.control_mode = mode
        self.get_logger().info(f"control_mode -> {mode}")

    def _foot_targets_callback(self, msg: PoseArray):
        """Run IK on four foot positions and publish the resulting joint angles."""
        if self.control_mode != "manual":
            return
        if len(msg.poses) != len(LEG_ORDER):
            self.get_logger().warning(
                f"foot_targets expected {len(LEG_ORDER)} poses, got {len(msg.poses)}"
            )
            return
        foot_matrix = np.zeros((3, len(LEG_ORDER)), dtype=float)
        for leg_index, pose in enumerate(msg.poses):
            foot_matrix[0, leg_index] = float(pose.position.x)
            foot_matrix[1, leg_index] = float(pose.position.y)
            foot_matrix[2, leg_index] = float(pose.position.z)
        try:
            angles = four_legs_inverse_kinematics(foot_matrix, self.config)
        except ValueError as exc:
            self.get_logger().warning(f"IK miss on foot target: {exc}")
            return

        # Flatten (3, 4) -> (12,) in JOINT_NAMES order
        positions = np.zeros(12, dtype=float)
        for leg_index in range(4):
            for row in range(3):
                positions[leg_index * 3 + row] = float(angles[row, leg_index])

        clamped = self._clamp_envelope(positions)
        adjusted = self._apply_overrides(clamped)
        self._publish_raw(adjusted)

    def _joint_angles_callback(self, msg: JointState):
        if self.control_mode != "manual":
            return
        if len(msg.position) != len(JOINT_NAMES):
            self.get_logger().warning(
                f"joint_angles expected {len(JOINT_NAMES)} values, got {len(msg.position)}"
            )
            return
        if msg.name:
            index_by_name = {name: idx for idx, name in enumerate(msg.name)}
            try:
                positions = np.asarray(
                    [msg.position[index_by_name[name]] for name in JOINT_NAMES],
                    dtype=float,
                )
            except KeyError as exc:
                self.get_logger().warning(
                    f"joint_angles missing joint name {exc}"
                )
                return
        else:
            positions = np.asarray(msg.position, dtype=float)

        clamped = self._clamp_envelope(positions)
        adjusted = self._apply_overrides(clamped)
        self._publish_raw(adjusted)

    def _servo_overrides_callback(self, msg: String):
        """Update the in-memory override table and persist to disk atomically."""
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid servo_overrides JSON: {exc}")
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("servo_overrides payload must be an object")
            return

        with self._override_lock:
            self.servo_overrides = payload
            try:
                self._atomic_write_json(SERVO_OVERRIDES_PATH, payload)
            except OSError as exc:
                self.get_logger().error(
                    f"Could not persist servo overrides: {exc}"
                )
                return
        self._log_override_table()

    def _joint_limits_callback(self, msg: String):
        """Update the per-joint user-limit table and persist to disk atomically.

        Accepts either radian (`min_rad`/`max_rad`) or degree
        (`min_deg`/`max_deg`) payloads — the dashboard Settings drawer today
        sends degrees. These limits can only TIGHTEN the Config.py defaults.
        """
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid joint_limits JSON: {exc}")
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("joint_limits payload must be an object")
            return

        normalized = self._normalize_joint_limits(payload)
        with self._limits_lock:
            self.joint_limits_rad = normalized
            # Persist in radians so the ROS side is the source of truth; the
            # Pi server and browser both round-trip through /api/settings/
            # joint_limits, so this file is a recovery snapshot, not the live
            # read path.
            snapshot = {
                name: {"min_rad": lo, "max_rad": hi}
                for name, (lo, hi) in normalized.items()
            }
            try:
                self._atomic_write_json(JOINT_LIMITS_PATH, snapshot)
            except OSError as exc:
                self.get_logger().error(
                    f"Could not persist joint limits: {exc}"
                )
                return
        self._log_joint_limits()

    def _servo_speed_limits_callback(self, msg: String):
        """Update the per-joint speed ceiling from the dashboard.

        Payload: JSON object `{joint_name: deg_per_sec}`. Missing joints keep
        their current ceiling. A zero value disables rate-limiting for that
        joint (lets the full slider-delta through in one publish).
        """
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid servo_speed_limits JSON: {exc}")
            return
        if not isinstance(payload, dict):
            self.get_logger().warning("servo_speed_limits payload must be an object")
            return

        updated: dict[str, float] = {}
        with self._speed_lock:
            self.servo_speed_limits_rad_per_sec = dict(self.servo_speed_limits_rad_per_sec)
            for name, deg_per_sec in payload.items():
                if name not in JOINT_NAMES:
                    continue
                try:
                    value = max(0.0, float(deg_per_sec))
                except (TypeError, ValueError):
                    continue
                self.servo_speed_limits_rad_per_sec[name] = value * _DEG_TO_RAD
                updated[name] = value
            snapshot_deg = {
                name: rate / _DEG_TO_RAD
                for name, rate in self.servo_speed_limits_rad_per_sec.items()
            }
            try:
                self._atomic_write_json(SERVO_SPEED_PATH, snapshot_deg)
            except OSError as exc:
                self.get_logger().error(
                    f"Could not persist servo speed limits: {exc}"
                )
        if updated:
            self.get_logger().info(
                "Updated speed limits: "
                + ", ".join(f"{k}={v:.0f} deg/s" for k, v in updated.items())
            )

    def _clamp_by_speed(self, positions: np.ndarray) -> np.ndarray:
        """Rate-limit the per-joint delta since the last publish.

        The dashboard bridge is the last software hop before safety_node and
        the firmware; clamping here prevents a big slider jump from translating
        into a single 100 Hz step that the servos physically can't follow.
        """
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._last_published_positions is None
            or self._last_published_ns is None
            or self._last_published_positions.shape != positions.shape
        ):
            self._last_published_positions = positions.copy()
            self._last_published_ns = now_ns
            return positions

        dt = (now_ns - self._last_published_ns) / 1e9
        if dt <= 0.0:
            return self._last_published_positions.copy()

        with self._speed_lock:
            limits = dict(self.servo_speed_limits_rad_per_sec)

        prev = self._last_published_positions
        clamped = positions.copy()
        for idx, name in enumerate(JOINT_NAMES):
            limit = limits.get(name, 0.0)
            if limit <= 0.0:
                continue
            max_step = limit * dt
            delta = float(positions[idx] - prev[idx])
            if delta > max_step:
                clamped[idx] = prev[idx] + max_step
            elif delta < -max_step:
                clamped[idx] = prev[idx] - max_step

        self._last_published_positions = clamped.copy()
        self._last_published_ns = now_ns
        return clamped

    def _stance_play_callback(self, msg: String):
        """Look up a named stance and publish its 12-angle vector."""
        stance_name = msg.data.strip()
        if not stance_name:
            return
        stances = self._load_stances()
        angles = stances.get(stance_name)
        if angles is None:
            self.get_logger().warning(f"Unknown stance '{stance_name}'")
            return
        if len(angles) != len(JOINT_NAMES):
            self.get_logger().warning(
                f"Stance '{stance_name}' has {len(angles)} angles, expected {len(JOINT_NAMES)}"
            )
            return
        clamped = self._clamp_envelope(np.asarray(angles, dtype=float))
        adjusted = self._apply_overrides(clamped)
        self._publish_raw(adjusted)

    # ── Publish helper ───────────────────────────────────────────────────

    def _publish_raw(self, positions: np.ndarray) -> None:
        rate_limited = self._clamp_by_speed(np.asarray(positions, dtype=float))
        msg = joint_state_from_positions(
            self.get_clock().now().to_msg(), rate_limited
        )
        self.raw_joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
