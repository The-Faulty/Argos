// Typed wrappers around roslibjs for the Pi-side dashboard server.
//
// roslibjs's Topic/Service constructors take freeform strings for names and
// message types — easy to typo, and every subscriber ends up repeating the
// same `new ROSLIB.Topic({...})` boilerplate. This module centralizes all
// topic metadata + gives the server a single place to swap the underlying
// ros lib (e.g. if we move off rosbridge later). The SetParameters wrapper
// hides the ROS 2 parameter-type dance behind a plain JS object.

import ROSLIB from "roslib";

// Topic names mirror ros2_ws/argos_control/ros_support.py::TOPICS. A mismatch
// here is a silent bug (no error — you just never get messages), so keep the
// strings identical to the Python side. If TOPICS is edited, update both.
export const TOPIC_NAMES = {
  cmd_vel: "/cmd_vel",
  estop: "/estop",
  gait_mode: "/gait_mode",
  release_servos: "/release_servos",
  control_mode: "/control_mode",
  joint_states: "/joint_states",
  joint_command_raw: "/joint_command/raw",
  imu_raw: "/imu/data_raw",
  gas: "/gas",
  // Dashboard-originated topics, consumed by dashboard_bridge_node.
  dashboard_foot_targets: "/dashboard/foot_targets",
  dashboard_joint_angles: "/dashboard/joint_angles",
  dashboard_stance_play: "/dashboard/stance_play",
  dashboard_servo_overrides: "/dashboard/servo_overrides",
  dashboard_joint_limits: "/dashboard/joint_limits",
  dashboard_servo_speed_limits: "/dashboard/servo_speed_limits",
  dashboard_servo_update_rate_hz: "/dashboard/servo_update_rate_hz",
};

const MSG_TYPES = {
  Twist: "geometry_msgs/msg/Twist",
  PoseArray: "geometry_msgs/msg/PoseArray",
  Bool: "std_msgs/msg/Bool",
  String: "std_msgs/msg/String",
  Float32: "std_msgs/msg/Float32",
  Float32MultiArray: "std_msgs/msg/Float32MultiArray",
  JointState: "sensor_msgs/msg/JointState",
  Imu: "sensor_msgs/msg/Imu",
};

export class RosBridgeClient {
  constructor({ url = "ws://localhost:9090" } = {}) {
    this.url = url;
    this.ros = new ROSLIB.Ros({ url });
    this.connected = false;
    this._listeners = new Set();
    this._publishers = new Map();
    this._subscribers = new Map();

    this.ros.on("connection", () => {
      this.connected = true;
      this._emit({ event: "connected" });
    });
    this.ros.on("error", (error) => {
      this.connected = false;
      this._emit({ event: "error", error: String(error?.message ?? error) });
    });
    this.ros.on("close", () => {
      this.connected = false;
      this._emit({ event: "closed" });
    });
  }

  onStatus(listener) {
    this._listeners.add(listener);
    return () => this._listeners.delete(listener);
  }

  _emit(status) {
    for (const listener of this._listeners) {
      try {
        listener(status);
      } catch {
        /* never let a listener break another */
      }
    }
  }

  _getPublisher(name, messageType) {
    const key = `${name}::${messageType}`;
    if (this._publishers.has(key)) return this._publishers.get(key);
    const topic = new ROSLIB.Topic({ ros: this.ros, name, messageType });
    this._publishers.set(key, topic);
    return topic;
  }

  _getSubscriber(name, messageType) {
    const key = `${name}::${messageType}`;
    if (this._subscribers.has(key)) return this._subscribers.get(key);
    const topic = new ROSLIB.Topic({ ros: this.ros, name, messageType });
    this._subscribers.set(key, topic);
    return topic;
  }

  // ─── Publishers ─────────────────────────────────────────────────────────

  publishTwist({ x = 0, y = 0, z = 0, wx = 0, wy = 0, wz = 0 } = {}) {
    const topic = this._getPublisher(TOPIC_NAMES.cmd_vel, MSG_TYPES.Twist);
    topic.publish(
      new ROSLIB.Message({
        linear: { x, y, z },
        angular: { x: wx, y: wy, z: wz },
      }),
    );
  }

  publishFootTargets(matrix4x3) {
    // matrix4x3: FR/FL/RR/RL rows of [x, y, z] meters, body frame.
    const poses = matrix4x3.map(([x, y, z]) => ({
      position: { x, y, z },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    }));
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_foot_targets, MSG_TYPES.PoseArray);
    topic.publish(
      new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "base_link" },
        poses,
      }),
    );
  }

  publishJointAnglesRad(jointNames, positionsRad) {
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_joint_angles, MSG_TYPES.JointState);
    topic.publish(
      new ROSLIB.Message({
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: "base_link" },
        name: jointNames,
        position: positionsRad,
        velocity: [],
        effort: [],
      }),
    );
  }

  publishEstop(value) {
    const topic = this._getPublisher(TOPIC_NAMES.estop, MSG_TYPES.Bool);
    topic.publish(new ROSLIB.Message({ data: Boolean(value) }));
  }

  publishReleaseServos(value) {
    const topic = this._getPublisher(TOPIC_NAMES.release_servos, MSG_TYPES.Bool);
    topic.publish(new ROSLIB.Message({ data: Boolean(value) }));
  }

  publishGaitMode(name) {
    const topic = this._getPublisher(TOPIC_NAMES.gait_mode, MSG_TYPES.String);
    topic.publish(new ROSLIB.Message({ data: String(name) }));
  }

  publishControlMode(name) {
    const topic = this._getPublisher(TOPIC_NAMES.control_mode, MSG_TYPES.String);
    topic.publish(new ROSLIB.Message({ data: String(name) }));
  }

  publishStancePlay(name) {
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_stance_play, MSG_TYPES.String);
    topic.publish(new ROSLIB.Message({ data: String(name) }));
  }

  publishServoOverrides(overridesObj) {
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_servo_overrides, MSG_TYPES.String);
    topic.publish(new ROSLIB.Message({ data: JSON.stringify(overridesObj ?? {}) }));
  }

  publishJointLimits(limitsObj) {
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_joint_limits, MSG_TYPES.String);
    topic.publish(new ROSLIB.Message({ data: JSON.stringify(limitsObj ?? {}) }));
  }

  publishServoSpeedLimits(limitsObj) {
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_servo_speed_limits, MSG_TYPES.String);
    topic.publish(new ROSLIB.Message({ data: JSON.stringify(limitsObj ?? {}) }));
  }

  publishServoUpdateRate(hz) {
    const topic = this._getPublisher(TOPIC_NAMES.dashboard_servo_update_rate_hz, MSG_TYPES.Float32);
    topic.publish(new ROSLIB.Message({ data: Number(hz) }));
  }

  // ─── Subscribers ─────────────────────────────────────────────────────────

  subscribeJointStates(cb) {
    return this._subscribe(TOPIC_NAMES.joint_states, MSG_TYPES.JointState, cb);
  }
  subscribeImu(cb) {
    return this._subscribe(TOPIC_NAMES.imu_raw, MSG_TYPES.Imu, cb);
  }
  subscribeGas(cb) {
    return this._subscribe(TOPIC_NAMES.gas, MSG_TYPES.Float32, cb);
  }
  subscribeGaitMode(cb) {
    return this._subscribe(TOPIC_NAMES.gait_mode, MSG_TYPES.String, cb);
  }
  subscribeControlMode(cb) {
    return this._subscribe(TOPIC_NAMES.control_mode, MSG_TYPES.String, cb);
  }

  _subscribe(name, messageType, cb) {
    const topic = this._getSubscriber(name, messageType);
    topic.subscribe(cb);
    return () => topic.unsubscribe(cb);
  }

  // ─── ROS 2 parameter service ─────────────────────────────────────────────
  // Every ROS 2 node exposes `<node_name>/set_parameters` with the message
  // type rcl_interfaces/srv/SetParameters. We convert JS values into ROS
  // ParameterValue variants (type 1=bool, 2=int, 3=double, 4=string, 5=bytes,
  // 6..9=arrays). Only bool/int/double/string show up in practice.
  async setParameter(nodeName, paramName, value) {
    const service = new ROSLIB.Service({
      ros: this.ros,
      name: `${nodeName}/set_parameters`,
      serviceType: "rcl_interfaces/srv/SetParameters",
    });
    const parameterValue = jsValueToRos2ParameterValue(value);
    const request = new ROSLIB.ServiceRequest({
      parameters: [{ name: paramName, value: parameterValue }],
    });
    return new Promise((resolve, reject) => {
      service.callService(
        request,
        (response) => {
          const result = response?.results?.[0];
          if (!result || result.successful === false) {
            reject(new Error(result?.reason || `Failed to set ${paramName}`));
            return;
          }
          resolve(result);
        },
        (error) => reject(new Error(String(error))),
      );
    });
  }

  close() {
    try {
      this.ros.close();
    } catch {
      /* roslibjs throws if already closed */
    }
  }
}

export function jsValueToRos2ParameterValue(value) {
  if (typeof value === "boolean") {
    return { type: 1, bool_value: value, integer_value: 0, double_value: 0.0, string_value: "",
             byte_array_value: [], bool_array_value: [], integer_array_value: [],
             double_array_value: [], string_array_value: [] };
  }
  if (typeof value === "number") {
    if (Number.isInteger(value)) {
      return { type: 2, bool_value: false, integer_value: value, double_value: 0.0, string_value: "",
               byte_array_value: [], bool_array_value: [], integer_array_value: [],
               double_array_value: [], string_array_value: [] };
    }
    return { type: 3, bool_value: false, integer_value: 0, double_value: value, string_value: "",
             byte_array_value: [], bool_array_value: [], integer_array_value: [],
             double_array_value: [], string_array_value: [] };
  }
  if (typeof value === "string") {
    return { type: 4, bool_value: false, integer_value: 0, double_value: 0.0, string_value: value,
             byte_array_value: [], bool_array_value: [], integer_array_value: [],
             double_array_value: [], string_array_value: [] };
  }
  throw new Error(`Unsupported parameter value type: ${typeof value}`);
}
