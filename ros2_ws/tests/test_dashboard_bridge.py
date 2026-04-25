"""Integration test for argos_control.dashboard_bridge_node.

Two cases:

1. `test_foot_targets_publish_produces_matching_joint_state` — publishes
   control_mode=manual and a PoseArray of four reachable feet, then asserts
   the resulting /joint_command/raw JointState matches
   `four_legs_inverse_kinematics` after the coupled bell-crank clamp.
   Tolerance is 3e-3 rad (one sweep step of _sagittal_ik; see
   dashboard/tests/kinematics.test.js for the rationale).

2. `test_joint_angles_clamped_to_coupled_envelope` — publishes a wildly
   out-of-envelope direct joint command and verifies the bridge clamps it
   inline (this is the Phase E safety guarantee: safety_node lives on the
   pre-mux channel and never sees dashboard-originated commands, so the
   bridge is the only clamp point for direct joint-angle inputs).

Gracefully skips if rclpy or the argos_control package cannot be imported
or initialized — expected when the ROS environment is not sourced.
"""

from pathlib import Path
import sys
import threading
import time

import pytest


# Allow importing argos_control as a package when running from ros2_ws/
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


# ── Optional-dependency guard ────────────────────────────────────────────
# pytest.importorskip would emit a "skip" the moment any of these fail.
rclpy = pytest.importorskip("rclpy")
geometry_msgs_msg = pytest.importorskip("geometry_msgs.msg")
sensor_msgs_msg = pytest.importorskip("sensor_msgs.msg")
std_msgs_msg = pytest.importorskip("std_msgs.msg")

try:
    import numpy as np
    from argos_control.Config import Configuration
    from argos_control.Kinematics import four_legs_inverse_kinematics
    from argos_control.dashboard_bridge_node import DashboardBridgeNode
    from argos_control.ros_support import (
        JOINT_NAMES,
        LEG_ORDER,
        TOPICS,
        default_foot_locations,
    )
except Exception as exc:  # pragma: no cover
    pytest.skip(
        f"argos_control stack not importable ({exc}); skipping dashboard-bridge test",
        allow_module_level=True,
    )

PoseArray = geometry_msgs_msg.PoseArray
Pose = geometry_msgs_msg.Pose
JointState = sensor_msgs_msg.JointState
StringMsg = std_msgs_msg.String


# ── Fixtures ─────────────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def rclpy_context():
    """Initialize rclpy once per module; skip cleanly if the RMW is missing."""
    try:
        rclpy.init()
    except Exception as exc:  # pragma: no cover
        pytest.skip(f"rclpy.init() failed ({exc}); skipping dashboard-bridge test")
    yield
    try:
        rclpy.shutdown()
    except Exception:
        pass


# ── Helpers ──────────────────────────────────────────────────────────────

def _make_pose_array(config) -> PoseArray:
    """Build a 4-pose body-frame stance at the default crouch-to-stand height."""
    feet = default_foot_locations(config)  # (3, 4), body frame, FR/FL/RR/RL
    msg = PoseArray()
    msg.header.frame_id = "base_link"
    for i in range(len(LEG_ORDER)):
        p = Pose()
        p.position.x = float(feet[0, i])
        p.position.y = float(feet[1, i])
        p.position.z = float(feet[2, i])
        p.orientation.w = 1.0
        msg.poses.append(p)
    return msg


def _expected_positions(config) -> np.ndarray:
    """Python-reference angles for the stance above, flattened JOINT_NAMES order.

    The bridge applies `config.clamp_joint_matrix` to the IK output before
    publishing, so the reference path must do the same to stay in lock-step.
    """
    feet = default_foot_locations(config)
    angles = four_legs_inverse_kinematics(feet, config)
    angles = config.clamp_joint_matrix(angles)
    out = np.zeros(12, dtype=float)
    for leg_index in range(len(LEG_ORDER)):
        for row in range(3):
            out[leg_index * 3 + row] = float(angles[row, leg_index])
    return out


# ── The test ─────────────────────────────────────────────────────────────

def test_foot_targets_publish_produces_matching_joint_state(rclpy_context):
    config = Configuration()
    expected = _expected_positions(config)

    # Bring the node up.
    bridge_node = DashboardBridgeNode()
    helper_node = rclpy.create_node("dashboard_bridge_test_helper")

    # Subscribe to /joint_command/raw — we capture the first JointState emitted
    # by the bridge in response to our PoseArray.
    received = {"msg": None}
    latch = threading.Event()

    def _on_joint_state(msg):
        received["msg"] = msg
        latch.set()

    helper_node.create_subscription(
        JointState, TOPICS.joint_command_raw, _on_joint_state, 10
    )

    control_mode_pub = helper_node.create_publisher(
        StringMsg, TOPICS.control_mode, 10
    )
    foot_pub = helper_node.create_publisher(
        PoseArray, TOPICS.dashboard_foot_targets, 10
    )

    # Spin both nodes on background threads so callbacks fire asynchronously.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(helper_node)

    stop = threading.Event()

    def _spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        # Give the pub/sub pair a moment to discover each other before
        # publishing. rclpy's discovery is fast on the default RMW but not
        # instant — 200 ms is a conservative bound.
        time.sleep(0.2)

        # Flip the bridge into manual mode.
        mode_msg = StringMsg()
        mode_msg.data = "manual"
        control_mode_pub.publish(mode_msg)
        time.sleep(0.1)  # let the mode change settle before the next publish

        foot_pub.publish(_make_pose_array(config))

        # Wait up to 1 s for the JointState echo.
        assert latch.wait(timeout=1.0), (
            "Did not receive /joint_command/raw within 1 s after publishing "
            "/dashboard/foot_targets in manual mode."
        )
    finally:
        stop.set()
        spin_thread.join(timeout=1.0)
        bridge_node.destroy_node()
        helper_node.destroy_node()

    msg = received["msg"]
    assert msg is not None
    assert list(msg.name) == list(JOINT_NAMES), (
        "JointState name list must match ros_support.JOINT_NAMES order"
    )
    assert len(msg.position) == len(JOINT_NAMES)

    got = np.asarray(msg.position, dtype=float)
    diffs = np.abs(got - expected)
    # 3e-3 rad (~0.17°): one sweep step of _sagittal_ik's 0.1° refinement.
    # Any real algorithmic drift is orders of magnitude above this.
    assert np.all(diffs < 3e-3), (
        "Bridge IK output diverges from Python IK reference:\n"
        f"  max abs diff = {diffs.max():.2e} rad\n"
        f"  got      = {got.tolist()}\n"
        f"  expected = {expected.tolist()}"
    )


# ── Second case: direct joint-angle clamp ────────────────────────────────

def test_joint_angles_clamped_to_coupled_envelope(rclpy_context):
    """A direct joint-angle command outside the envelope must be clamped.

    Publishes a JointState with intentionally out-of-envelope femur + tibia
    values and asserts the bridge republishes something that falls inside
    `config.clamp_joint_matrix`'s feasible set. This is the only safety
    gate for direct joint-angle commands — `safety_node` sits on the pre-mux
    channel and never sees dashboard output.
    """
    config = Configuration()

    bridge_node = DashboardBridgeNode()
    helper_node = rclpy.create_node("dashboard_bridge_clamp_helper")

    received = {"msg": None}
    latch = threading.Event()

    def _on_joint_state(msg):
        received["msg"] = msg
        latch.set()

    helper_node.create_subscription(
        JointState, TOPICS.joint_command_raw, _on_joint_state, 10
    )
    control_mode_pub = helper_node.create_publisher(
        StringMsg, TOPICS.control_mode, 10
    )
    joint_pub = helper_node.create_publisher(
        JointState, TOPICS.dashboard_joint_angles, 10
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge_node)
    executor.add_node(helper_node)

    stop = threading.Event()

    def _spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        time.sleep(0.2)
        mode_msg = StringMsg()
        mode_msg.data = "manual"
        control_mode_pub.publish(mode_msg)
        time.sleep(0.1)

        # Intentionally extreme: femur + tibia cranked outside the coupled
        # envelope on every leg.
        js = JointState()
        js.name = list(JOINT_NAMES)
        positions = []
        for name in JOINT_NAMES:
            if name.endswith("_femur_joint"):
                positions.append(2.0)   # ~115°, far above top_hi
            elif name.endswith("_tibia_joint"):
                positions.append(-2.0)  # incompatible bot for that top
            else:
                positions.append(0.0)
        js.position = positions
        joint_pub.publish(js)

        assert latch.wait(timeout=1.0), (
            "No /joint_command/raw echo within 1 s after publishing "
            "/dashboard/joint_angles"
        )
    finally:
        stop.set()
        spin_thread.join(timeout=1.0)
        bridge_node.destroy_node()
        helper_node.destroy_node()

    msg = received["msg"]
    assert msg is not None
    got = np.asarray(msg.position, dtype=float)

    # Reference: compute what the bridge is supposed to produce by running
    # the same clamp path against the raw request.
    raw = np.zeros((3, 4), dtype=float)
    for leg_idx, leg in enumerate(LEG_ORDER):
        for row_idx, row in enumerate(("coxa", "femur", "tibia")):
            name = f"{leg}_{row}_joint"
            if name.endswith("_femur_joint"):
                raw[row_idx, leg_idx] = 2.0
            elif name.endswith("_tibia_joint"):
                raw[row_idx, leg_idx] = -2.0
    clamped = config.clamp_joint_matrix(raw)
    expected = np.zeros(12, dtype=float)
    for leg_idx in range(len(LEG_ORDER)):
        for row in range(3):
            expected[leg_idx * 3 + row] = float(clamped[row, leg_idx])

    diffs = np.abs(got - expected)
    assert np.all(diffs < 1e-6), (
        "Bridge did not apply clamp_joint_matrix on direct joint-angle input:\n"
        f"  max abs diff = {diffs.max():.2e} rad\n"
        f"  got      = {got.tolist()}\n"
        f"  expected = {expected.tolist()}"
    )
    # Sanity: the clamped femur must be inside JOINT_LIMITS_RAD[1].
    femur_lo, femur_hi = config.JOINT_LIMITS_RAD[1]
    for leg_idx in range(4):
        femur_val = got[leg_idx * 3 + 1]
        assert femur_lo - 1e-9 <= femur_val <= femur_hi + 1e-9, (
            f"Leg {LEG_ORDER[leg_idx]} femur {femur_val} outside "
            f"[{femur_lo}, {femur_hi}]"
        )
