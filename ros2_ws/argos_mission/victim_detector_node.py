"""Detect likely victims from MLX90640 thermal frames and publish RViz markers."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

from .mission_contract import TOPICS
from .thermal_common import camera_matrix_from_fov
from .thermal_detection import HotspotTracker, TrackedHotspot, detect_hotspots


class VictimDetectorNode(Node):
    """Find and stabilize human-temperature hotspots from the thermal stream."""

    def __init__(self):
        super().__init__("victim_detector_node")

        self.declare_parameter("image_topic", TOPICS.thermal_image)
        self.declare_parameter("camera_info_topic", TOPICS.thermal_camera_info)
        self.declare_parameter("detections_topic", TOPICS.victim_detections)
        self.declare_parameter("detection_count_topic", TOPICS.mission_detection_count)
        self.declare_parameter("relative_threshold_c", 4.0)
        self.declare_parameter("min_cluster_pixels", 4)
        self.declare_parameter("max_cluster_pixels", 90)
        self.declare_parameter("min_victim_temp_c", 30.0)
        self.declare_parameter("max_victim_temp_c", 42.0)
        self.declare_parameter("min_peak_delta_c", 2.5)
        self.declare_parameter("smoothing_passes", 1)
        self.declare_parameter("connectivity", 8)
        self.declare_parameter("tracking_radius_px", 3.5)
        self.declare_parameter("min_confirmed_frames", 2)
        self.declare_parameter("max_missed_frames", 1)
        self.declare_parameter("assumed_range_m", 1.2)
        self.declare_parameter("horizontal_fov_deg", 110.0)
        self.declare_parameter("vertical_fov_deg", 75.0)
        self.declare_parameter("marker_scale_m", 0.20)
        self.declare_parameter("label_offset_m", 0.18)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        detection_count_topic = self.get_parameter("detection_count_topic").value

        self.relative_threshold_c = float(self.get_parameter("relative_threshold_c").value)
        self.min_cluster_pixels = int(self.get_parameter("min_cluster_pixels").value)
        self.max_cluster_pixels = int(self.get_parameter("max_cluster_pixels").value)
        self.min_victim_temp_c = float(self.get_parameter("min_victim_temp_c").value)
        self.max_victim_temp_c = float(self.get_parameter("max_victim_temp_c").value)
        self.min_peak_delta_c = float(self.get_parameter("min_peak_delta_c").value)
        self.smoothing_passes = int(self.get_parameter("smoothing_passes").value)
        self.connectivity = int(self.get_parameter("connectivity").value)
        self.assumed_range_m = float(self.get_parameter("assumed_range_m").value)
        self.horizontal_fov_rad = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value)
        )
        self.vertical_fov_rad = math.radians(
            float(self.get_parameter("vertical_fov_deg").value)
        )
        self.marker_scale_m = float(self.get_parameter("marker_scale_m").value)
        self.label_offset_m = float(self.get_parameter("label_offset_m").value)
        self.latest_intrinsics: Optional[Tuple[float, float, float, float]] = None
        self.latest_camera_size: Optional[Tuple[int, int]] = None
        self.warned_bad_encoding = False

        self.tracker = HotspotTracker(
            match_radius_px=float(self.get_parameter("tracking_radius_px").value),
            min_confirmed_frames=int(self.get_parameter("min_confirmed_frames").value),
            max_missed_frames=int(self.get_parameter("max_missed_frames").value),
        )

        self.detections_pub = self.create_publisher(MarkerArray, detections_topic, 10)
        self.detection_count_pub = self.create_publisher(Int32, detection_count_topic, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_callback, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)

    def _camera_info_callback(self, msg: CameraInfo):
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.latest_intrinsics = (
                float(msg.k[0]),
                float(msg.k[4]),
                float(msg.k[2]),
                float(msg.k[5]),
            )
            self.latest_camera_size = (int(msg.width), int(msg.height))

    def _intrinsics_for_shape(self, width: int, height: int) -> Tuple[float, float, float, float]:
        if self.latest_intrinsics is not None and self.latest_camera_size == (width, height):
            return self.latest_intrinsics
        return camera_matrix_from_fov(
            width=width,
            height=height,
            horizontal_fov_rad=self.horizontal_fov_rad,
            vertical_fov_rad=self.vertical_fov_rad,
        )

    def _project_track(
        self,
        track: TrackedHotspot,
        *,
        width: int,
        height: int,
    ) -> Tuple[float, float, float]:
        fx, fy, cx, cy = self._intrinsics_for_shape(width, height)
        x = self.assumed_range_m
        y = -((track.cluster.centroid_col - cx) / fx) * self.assumed_range_m
        z = -((track.cluster.centroid_row - cy) / fy) * self.assumed_range_m
        return x, y, z

    def _sphere_marker(self, track: TrackedHotspot, stamp, frame_id: str, width: int, height: int) -> Marker:
        x, y, z = self._project_track(track, width=width, height=height)
        confidence = min(
            1.0,
            float(track.consecutive_hits) / max(1.0, float(self.tracker.min_confirmed_frames)),
        )

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "victims"
        marker.id = track.track_id * 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale_m
        marker.scale.y = self.marker_scale_m
        marker.scale.z = self.marker_scale_m
        marker.color.r = 1.0
        marker.color.g = 0.30 + 0.40 * confidence
        marker.color.b = 0.10
        marker.color.a = 0.90
        return marker

    def _label_marker(self, track: TrackedHotspot, stamp, frame_id: str, width: int, height: int) -> Marker:
        x, y, z = self._project_track(track, width=width, height=height)
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "victim_labels"
        marker.id = track.track_id * 2 + 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z + self.label_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = max(0.05, self.marker_scale_m * 0.6)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = (
            f"{track.cluster.peak_temp_c:.1f}C "
            f"({track.cluster.pixel_count}px, +{track.cluster.contrast_temp_c:.1f}C)"
        )
        return marker

    def _publish_markers(self, tracks, stamp, frame_id: str, width: int, height: int):
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = frame_id
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for track in tracks:
            markers.markers.append(
                self._sphere_marker(track, stamp=stamp, frame_id=frame_id, width=width, height=height)
            )
            markers.markers.append(
                self._label_marker(track, stamp=stamp, frame_id=frame_id, width=width, height=height)
            )

        self.detections_pub.publish(markers)

    def _image_callback(self, msg: Image):
        if msg.encoding and msg.encoding != "32FC1" and not self.warned_bad_encoding:
            self.get_logger().warning(
                f"Expected thermal frames encoded as 32FC1, received {msg.encoding}."
            )
            self.warned_bad_encoding = True

        thermal = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if thermal.size != msg.height * msg.width:
            self.get_logger().warning("Ignoring malformed thermal image.")
            return

        thermal = thermal.reshape((msg.height, msg.width))
        clusters, _, _ = detect_hotspots(
            thermal,
            relative_threshold_c=self.relative_threshold_c,
            min_cluster_pixels=self.min_cluster_pixels,
            max_cluster_pixels=self.max_cluster_pixels,
            min_victim_temp_c=self.min_victim_temp_c,
            max_victim_temp_c=self.max_victim_temp_c,
            min_peak_delta_c=self.min_peak_delta_c,
            smoothing_passes=self.smoothing_passes,
            connectivity=self.connectivity,
        )
        confirmed_tracks = self.tracker.update(clusters)
        frame_id = msg.header.frame_id or "thermal_link"

        self._publish_markers(
            confirmed_tracks,
            stamp=msg.header.stamp,
            frame_id=frame_id,
            width=msg.width,
            height=msg.height,
        )

        detection_count = Int32()
        detection_count.data = len(confirmed_tracks)
        self.detection_count_pub.publish(detection_count)


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
