"""Score candidate footholds against aligned depth and suggest safe landings."""

from dataclasses import dataclass
import sys

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Int32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from transforms3d.quaternions import quat2mat
from visualization_msgs.msg import Marker, MarkerArray

from .ros_support import (
    LEG_ORDER,
    TOPICS,
    int32_multiarray_from_values,
    matrix_from_pose_array,
    pose_array_from_matrix,
    values_from_int32_multiarray,
)


@dataclass
class FootholdResult:
    safe: bool
    adjusted_point: np.ndarray
    support_count: int
    height_error: float
    height_std: float
    height_span: float
    score: float
    reason: str


class FootholdCheckerNode(Node):
    def __init__(self):
        super().__init__("foothold_checker_node")

        self.declare_parameter("candidate_topic", TOPICS.foothold_candidates)
        self.declare_parameter("contact_modes_topic", TOPICS.foothold_contact_modes)
        self.declare_parameter("adjusted_topic", TOPICS.foothold_adjusted)
        self.declare_parameter("status_topic", TOPICS.foothold_status)
        self.declare_parameter("safe_topic", TOPICS.foothold_safe)
        self.declare_parameter("marker_topic", TOPICS.foothold_markers)
        self.declare_parameter(
            "depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic", "/camera/camera/aligned_depth_to_color/camera_info"
        )
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("search_radius_m", 0.04)
        self.declare_parameter("search_step_m", 0.02)
        self.declare_parameter("support_radius_m", 0.025)
        self.declare_parameter("patch_radius_px", 3)
        self.declare_parameter("min_support_samples", 5)
        self.declare_parameter("max_height_error_m", 0.03)
        self.declare_parameter("max_height_std_m", 0.012)
        self.declare_parameter("max_height_span_m", 0.03)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 1.5)

        candidate_topic = self.get_parameter("candidate_topic").value
        contact_modes_topic = self.get_parameter("contact_modes_topic").value
        adjusted_topic = self.get_parameter("adjusted_topic").value
        status_topic = self.get_parameter("status_topic").value
        safe_topic = self.get_parameter("safe_topic").value
        marker_topic = self.get_parameter("marker_topic").value
        depth_image_topic = self.get_parameter("depth_image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.base_frame = str(self.get_parameter("base_frame").value)
        update_rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.search_radius_m = max(
            0.0, float(self.get_parameter("search_radius_m").value)
        )
        self.search_step_m = max(0.005, float(self.get_parameter("search_step_m").value))
        self.support_radius_m = max(
            0.001, float(self.get_parameter("support_radius_m").value)
        )
        self.patch_radius_px = max(
            0, int(self.get_parameter("patch_radius_px").value)
        )
        self.min_support_samples = max(
            1, int(self.get_parameter("min_support_samples").value)
        )
        self.max_height_error_m = max(
            0.0, float(self.get_parameter("max_height_error_m").value)
        )
        self.max_height_std_m = max(
            0.0, float(self.get_parameter("max_height_std_m").value)
        )
        self.max_height_span_m = max(
            0.0, float(self.get_parameter("max_height_span_m").value)
        )
        self.min_depth_m = max(0.001, float(self.get_parameter("min_depth_m").value))
        self.max_depth_m = max(
            self.min_depth_m, float(self.get_parameter("max_depth_m").value)
        )

        self.depth_image: np.ndarray | None = None
        self.depth_frame_id = ""
        self.camera_info: CameraInfo | None = None
        self.candidate_points: np.ndarray | None = None
        self.candidate_frame = self.base_frame
        self.contact_modes = np.ones(4, dtype=int)
        self.search_offsets = self._build_search_offsets()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.adjusted_pub = self.create_publisher(PoseArray, adjusted_topic, 10)
        self.status_pub = self.create_publisher(Int32MultiArray, status_topic, 10)
        self.safe_pub = self.create_publisher(Bool, safe_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.create_subscription(
            PoseArray, candidate_topic, self._candidate_callback, 10
        )
        self.create_subscription(
            Int32MultiArray, contact_modes_topic, self._contact_modes_callback, 10
        )
        self.create_subscription(
            Image, depth_image_topic, self._depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_callback, qos_profile_sensor_data
        )
        self.create_timer(1.0 / update_rate_hz, self._evaluate)

    def _build_search_offsets(self) -> list[tuple[float, float]]:
        max_steps = int(round(self.search_radius_m / self.search_step_m))
        offsets = []
        for x_idx in range(-max_steps, max_steps + 1):
            for y_idx in range(-max_steps, max_steps + 1):
                dx = x_idx * self.search_step_m
                dy = y_idx * self.search_step_m
                if dx * dx + dy * dy <= (self.search_radius_m + 1e-9) ** 2:
                    offsets.append((dx, dy))
        offsets.sort(key=lambda value: value[0] * value[0] + value[1] * value[1])
        return offsets or [(0.0, 0.0)]

    def _candidate_callback(self, msg: PoseArray):
        try:
            self.candidate_points = matrix_from_pose_array(msg)
            self.candidate_frame = msg.header.frame_id or self.base_frame
        except ValueError as exc:
            self.get_logger().warning(
                f"Ignoring malformed foothold candidate message: {exc}"
            )

    def _contact_modes_callback(self, msg: Int32MultiArray):
        try:
            self.contact_modes = values_from_int32_multiarray(msg, expected_size=4)
        except ValueError as exc:
            self.get_logger().warning(
                f"Ignoring malformed foothold contact message: {exc}"
            )

    def _camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def _depth_callback(self, msg: Image):
        try:
            self.depth_image = self._decode_depth_image(msg)
            self.depth_frame_id = msg.header.frame_id
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring unsupported depth image: {exc}")

    def _decode_depth_image(self, msg: Image) -> np.ndarray:
        encoding = str(msg.encoding).upper()
        if encoding == "16UC1":
            dtype = np.dtype(np.uint16)
            scale = 0.001
        elif encoding == "32FC1":
            dtype = np.dtype(np.float32)
            scale = 1.0
        else:
            raise ValueError(f"expected 16UC1 or 32FC1, got {msg.encoding}")

        row_width = int(msg.step) // dtype.itemsize
        data = np.frombuffer(bytes(msg.data), dtype=dtype).reshape((msg.height, row_width))
        if bool(msg.is_bigendian) != (sys.byteorder == "big"):
            data = data.byteswap()
        depth = np.asarray(data[:, : msg.width], dtype=np.float32)
        depth *= scale
        depth[~np.isfinite(depth)] = 0.0
        return depth

    def _evaluate(self):
        if (
            self.depth_image is None
            or self.camera_info is None
            or self.candidate_points is None
            or not self.depth_frame_id
        ):
            return

        try:
            candidate_base = self._candidate_points_in_base()
            base_to_camera = self._lookup_transform(self.depth_frame_id, self.base_frame)
            camera_to_base = self._lookup_transform(self.base_frame, self.depth_frame_id)
        except (TransformException, ValueError) as exc:
            self.get_logger().warning(f"Skipping foothold evaluation: {exc}")
            return

        adjusted = np.array(candidate_base, dtype=float, copy=True)
        statuses = np.zeros(4, dtype=int)
        results: list[FootholdResult | None] = [None] * 4

        for leg_index in range(4):
            if self.contact_modes[leg_index] != 0:
                continue
            result = self._search_best_foothold(
                candidate_base[:, leg_index], base_to_camera, camera_to_base
            )
            results[leg_index] = result
            if result.safe:
                adjusted[:, leg_index] = result.adjusted_point
                statuses[leg_index] = 1
            else:
                statuses[leg_index] = -1

        swing_mask = self.contact_modes == 0
        overall_safe = not bool(np.any(statuses[swing_mask] < 0))

        stamp = self.get_clock().now().to_msg()
        self.adjusted_pub.publish(pose_array_from_matrix(stamp, adjusted, self.base_frame))
        self.status_pub.publish(int32_multiarray_from_values(statuses))
        safe_msg = Bool()
        safe_msg.data = overall_safe
        self.safe_pub.publish(safe_msg)
        self.marker_pub.publish(
            self._marker_array(stamp, candidate_base, adjusted, statuses, results)
        )

    def _candidate_points_in_base(self) -> np.ndarray:
        if self.candidate_points is None:
            raise ValueError("No foothold candidates available.")
        if self.candidate_frame == self.base_frame:
            return self.candidate_points
        rotation, translation = self._lookup_transform(
            self.base_frame, self.candidate_frame
        )
        return rotation @ self.candidate_points + translation[:, np.newaxis]

    def _lookup_transform(self, target_frame: str, source_frame: str):
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        rotation_matrix = quat2mat(
            (rotation.w, rotation.x, rotation.y, rotation.z)
        )
        translation_vector = np.array(
            [translation.x, translation.y, translation.z], dtype=float
        )
        return rotation_matrix, translation_vector

    def _search_best_foothold(
        self,
        nominal_base: np.ndarray,
        base_to_camera,
        camera_to_base,
    ) -> FootholdResult:
        best_safe: FootholdResult | None = None
        best_any: FootholdResult | None = None

        for dx, dy in self.search_offsets:
            query = np.array(nominal_base, dtype=float, copy=True)
            query[0] += dx
            query[1] += dy
            result = self._evaluate_patch(query, base_to_camera, camera_to_base)
            result.score += dx * dx + dy * dy

            if best_any is None or result.score < best_any.score:
                best_any = result
            if result.safe and (best_safe is None or result.score < best_safe.score):
                best_safe = result

        if best_safe is not None:
            return best_safe
        if best_any is not None:
            return best_any
        return FootholdResult(
            safe=False,
            adjusted_point=np.asarray(nominal_base, dtype=float),
            support_count=0,
            height_error=float("inf"),
            height_std=float("inf"),
            height_span=float("inf"),
            score=float("inf"),
            reason="no candidates",
        )

    def _evaluate_patch(
        self,
        candidate_base: np.ndarray,
        base_to_camera,
        camera_to_base,
    ) -> FootholdResult:
        if self.depth_image is None or self.camera_info is None:
            return FootholdResult(
                safe=False,
                adjusted_point=np.asarray(candidate_base, dtype=float),
                support_count=0,
                height_error=float("inf"),
                height_std=float("inf"),
                height_span=float("inf"),
                score=float("inf"),
                reason="missing depth",
            )

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return FootholdResult(
                safe=False,
                adjusted_point=np.asarray(candidate_base, dtype=float),
                support_count=0,
                height_error=float("inf"),
                height_std=float("inf"),
                height_span=float("inf"),
                score=float("inf"),
                reason="bad intrinsics",
            )

        rotation_bc, translation_bc = base_to_camera
        candidate_camera = rotation_bc @ candidate_base + translation_bc
        if not np.isfinite(candidate_camera).all() or candidate_camera[2] <= self.min_depth_m:
            return FootholdResult(
                safe=False,
                adjusted_point=np.asarray(candidate_base, dtype=float),
                support_count=0,
                height_error=float("inf"),
                height_std=float("inf"),
                height_span=float("inf"),
                score=float("inf"),
                reason="behind camera",
            )
        if candidate_camera[2] > self.max_depth_m:
            return FootholdResult(
                safe=False,
                adjusted_point=np.asarray(candidate_base, dtype=float),
                support_count=0,
                height_error=float("inf"),
                height_std=float("inf"),
                height_span=float("inf"),
                score=float("inf"),
                reason="too far",
            )

        pixel_u = int(round(fx * candidate_camera[0] / candidate_camera[2] + cx))
        pixel_v = int(round(fy * candidate_camera[1] / candidate_camera[2] + cy))
        if (
            pixel_u < self.patch_radius_px
            or pixel_v < self.patch_radius_px
            or pixel_u >= self.depth_image.shape[1] - self.patch_radius_px
            or pixel_v >= self.depth_image.shape[0] - self.patch_radius_px
        ):
            return FootholdResult(
                safe=False,
                adjusted_point=np.asarray(candidate_base, dtype=float),
                support_count=0,
                height_error=float("inf"),
                height_std=float("inf"),
                height_span=float("inf"),
                score=float("inf"),
                reason="outside image",
            )

        rotation_cb, translation_cb = camera_to_base
        support_points = []
        for v in range(pixel_v - self.patch_radius_px, pixel_v + self.patch_radius_px + 1):
            for u in range(
                pixel_u - self.patch_radius_px, pixel_u + self.patch_radius_px + 1
            ):
                depth_m = float(self.depth_image[v, u])
                if (
                    not np.isfinite(depth_m)
                    or depth_m <= self.min_depth_m
                    or depth_m > self.max_depth_m
                ):
                    continue

                point_camera = np.array(
                    [
                        (u - cx) * depth_m / fx,
                        (v - cy) * depth_m / fy,
                        depth_m,
                    ],
                    dtype=float,
                )
                point_base = rotation_cb @ point_camera + translation_cb
                if np.hypot(
                    point_base[0] - candidate_base[0],
                    point_base[1] - candidate_base[1],
                ) <= self.support_radius_m:
                    support_points.append(point_base)

        if len(support_points) < self.min_support_samples:
            return FootholdResult(
                safe=False,
                adjusted_point=np.asarray(candidate_base, dtype=float),
                support_count=len(support_points),
                height_error=float("inf"),
                height_std=float("inf"),
                height_span=float("inf"),
                score=float("inf"),
                reason="not enough support",
            )

        support_array = np.asarray(support_points, dtype=float)
        support_height = float(np.median(support_array[:, 2]))
        height_error = abs(support_height - float(candidate_base[2]))
        height_std = float(np.std(support_array[:, 2]))
        height_span = float(np.max(support_array[:, 2]) - np.min(support_array[:, 2]))
        safe = (
            height_error <= self.max_height_error_m
            and height_std <= self.max_height_std_m
            and height_span <= self.max_height_span_m
        )
        adjusted_point = np.array(
            [candidate_base[0], candidate_base[1], support_height], dtype=float
        )
        return FootholdResult(
            safe=safe,
            adjusted_point=adjusted_point,
            support_count=len(support_points),
            height_error=height_error,
            height_std=height_std,
            height_span=height_span,
            score=height_error + height_std + height_span,
            reason="ok" if safe else "rough or too tall",
        )

    def _marker_array(
        self,
        stamp,
        nominal_points: np.ndarray,
        adjusted_points: np.ndarray,
        statuses: np.ndarray,
        results: list[FootholdResult | None],
    ) -> MarkerArray:
        markers = MarkerArray()
        lifetime = Duration(sec=0, nanosec=300_000_000)

        for leg_index, leg_name in enumerate(LEG_ORDER):
            nominal = nominal_points[:, leg_index]
            adjusted = adjusted_points[:, leg_index]
            status = int(statuses[leg_index])

            nominal_marker = Marker()
            nominal_marker.header.stamp = stamp
            nominal_marker.header.frame_id = self.base_frame
            nominal_marker.ns = "foothold_nominal"
            nominal_marker.id = leg_index
            nominal_marker.type = Marker.SPHERE
            nominal_marker.action = Marker.ADD
            nominal_marker.pose.orientation.w = 1.0
            nominal_marker.pose.position.x = float(nominal[0])
            nominal_marker.pose.position.y = float(nominal[1])
            nominal_marker.pose.position.z = float(nominal[2])
            nominal_marker.scale.x = 0.028
            nominal_marker.scale.y = 0.028
            nominal_marker.scale.z = 0.028
            nominal_marker.color.r = 0.15
            nominal_marker.color.g = 0.55
            nominal_marker.color.b = 1.0
            nominal_marker.color.a = 0.45
            nominal_marker.lifetime = lifetime
            markers.markers.append(nominal_marker)

            adjusted_marker = Marker()
            adjusted_marker.header.stamp = stamp
            adjusted_marker.header.frame_id = self.base_frame
            adjusted_marker.ns = "foothold_adjusted"
            adjusted_marker.id = 100 + leg_index
            adjusted_marker.type = Marker.SPHERE
            adjusted_marker.action = Marker.ADD
            adjusted_marker.pose.orientation.w = 1.0
            adjusted_marker.pose.position.x = float(adjusted[0])
            adjusted_marker.pose.position.y = float(adjusted[1])
            adjusted_marker.pose.position.z = float(adjusted[2])
            adjusted_marker.scale.x = 0.034
            adjusted_marker.scale.y = 0.034
            adjusted_marker.scale.z = 0.034
            if status > 0:
                adjusted_marker.color.r = 0.15
                adjusted_marker.color.g = 0.85
                adjusted_marker.color.b = 0.2
                adjusted_marker.color.a = 0.9
            elif status < 0:
                adjusted_marker.color.r = 0.95
                adjusted_marker.color.g = 0.2
                adjusted_marker.color.b = 0.2
                adjusted_marker.color.a = 0.9
            else:
                adjusted_marker.color.r = 0.7
                adjusted_marker.color.g = 0.7
                adjusted_marker.color.b = 0.7
                adjusted_marker.color.a = 0.5
            adjusted_marker.lifetime = lifetime
            markers.markers.append(adjusted_marker)

            label_marker = Marker()
            label_marker.header.stamp = stamp
            label_marker.header.frame_id = self.base_frame
            label_marker.ns = "foothold_labels"
            label_marker.id = 200 + leg_index
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.orientation.w = 1.0
            label_marker.pose.position.x = float(adjusted[0])
            label_marker.pose.position.y = float(adjusted[1])
            label_marker.pose.position.z = float(adjusted[2] + 0.05)
            label_marker.scale.z = 0.03
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 0.9
            result = results[leg_index]
            if result is None:
                label_marker.text = f"{leg_name} stance"
            elif status > 0:
                label_marker.text = (
                    f"{leg_name} ok dz={result.height_error * 100.0:.1f}cm"
                )
            else:
                label_marker.text = f"{leg_name} {result.reason}"
            label_marker.lifetime = lifetime
            markers.markers.append(label_marker)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = FootholdCheckerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
