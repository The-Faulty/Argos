"""Detect thermal hotspots and publish RViz markers for likely victims."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray

from .mission_contract import TOPICS


class VictimDetectorNode(Node):
    def __init__(self):
        super().__init__("victim_detector_node")

        self.declare_parameter("image_topic", TOPICS.thermal_image)
        self.declare_parameter("detections_topic", TOPICS.victim_detections)
        self.declare_parameter("detection_count_topic", TOPICS.mission_detection_count)
        self.declare_parameter("relative_threshold_c", 8.0)
        self.declare_parameter("min_cluster_pixels", 3)
        self.declare_parameter("min_victim_temp_c", 30.0)
        self.declare_parameter("max_victim_temp_c", 42.0)
        self.declare_parameter("assumed_range_m", 1.2)
        self.declare_parameter("horizontal_fov_deg", 110.0)
        self.declare_parameter("vertical_fov_deg", 75.0)
        self.declare_parameter("marker_scale_m", 0.20)

        image_topic = self.get_parameter("image_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        detection_count_topic = self.get_parameter("detection_count_topic").value

        self.relative_threshold_c = float(
            self.get_parameter("relative_threshold_c").value
        )
        self.min_cluster_pixels = int(self.get_parameter("min_cluster_pixels").value)
        self.min_victim_temp_c = float(self.get_parameter("min_victim_temp_c").value)
        self.max_victim_temp_c = float(self.get_parameter("max_victim_temp_c").value)
        self.assumed_range_m = float(self.get_parameter("assumed_range_m").value)
        self.horizontal_fov_rad = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value)
        )
        self.vertical_fov_rad = math.radians(
            float(self.get_parameter("vertical_fov_deg").value)
        )
        self.marker_scale_m = float(self.get_parameter("marker_scale_m").value)

        self.detections_pub = self.create_publisher(MarkerArray, detections_topic, 10)
        self.detection_count_pub = self.create_publisher(Int32, detection_count_topic, 10)
        self.create_subscription(Image, image_topic, self._image_callback, 10)

    def _connected_components(self, mask: np.ndarray):
        visited = np.zeros_like(mask, dtype=bool)
        height, width = mask.shape
        components = []

        for row in range(height):
            for col in range(width):
                if not mask[row, col] or visited[row, col]:
                    continue

                queue = [(row, col)]
                visited[row, col] = True
                pixels = []

                while queue:
                    current_row, current_col = queue.pop()
                    pixels.append((current_row, current_col))
                    for delta_row, delta_col in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        next_row = current_row + delta_row
                        next_col = current_col + delta_col
                        if (
                            0 <= next_row < height
                            and 0 <= next_col < width
                            and mask[next_row, next_col]
                            and not visited[next_row, next_col]
                        ):
                            visited[next_row, next_col] = True
                            queue.append((next_row, next_col))

                components.append(pixels)

        return components

    def _cluster_to_marker(self, cluster, image: np.ndarray, stamp, frame_id: str, marker_id: int):
        rows = np.array([pixel[0] for pixel in cluster], dtype=float)
        cols = np.array([pixel[1] for pixel in cluster], dtype=float)
        centroid_row = float(rows.mean())
        centroid_col = float(cols.mean())

        width = image.shape[1]
        height = image.shape[0]
        x_norm = ((centroid_col + 0.5) / width - 0.5) * 2.0
        y_norm = ((centroid_row + 0.5) / height - 0.5) * 2.0

        lateral = math.tan(self.horizontal_fov_rad / 2.0) * self.assumed_range_m
        vertical = math.tan(self.vertical_fov_rad / 2.0) * self.assumed_range_m

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = "victims"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.assumed_range_m
        marker.pose.position.y = -x_norm * lateral
        marker.pose.position.z = -y_norm * vertical
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale_m
        marker.scale.y = self.marker_scale_m
        marker.scale.z = self.marker_scale_m
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.1
        marker.color.a = 0.85
        marker.text = f"{float(image[int(round(centroid_row)), int(round(centroid_col))]):.1f}C"
        return marker

    def _image_callback(self, msg: Image):
        thermal = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if thermal.size != msg.height * msg.width:
            self.get_logger().warning("Ignoring malformed thermal image.")
            return

        thermal = thermal.reshape((msg.height, msg.width))
        ambient = float(np.median(thermal))
        threshold = max(self.min_victim_temp_c, ambient + self.relative_threshold_c)
        mask = np.logical_and(
            thermal >= threshold,
            thermal <= self.max_victim_temp_c,
        )
        clusters = [
            cluster
            for cluster in self._connected_components(mask)
            if len(cluster) >= self.min_cluster_pixels
        ]

        markers = MarkerArray()

        delete_all = Marker()
        delete_all.header.stamp = msg.header.stamp
        delete_all.header.frame_id = msg.header.frame_id or "thermal_link"
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for marker_id, cluster in enumerate(clusters, start=1):
            markers.markers.append(
                self._cluster_to_marker(
                    cluster=cluster,
                    image=thermal,
                    stamp=msg.header.stamp,
                    frame_id=msg.header.frame_id or "thermal_link",
                    marker_id=marker_id,
                )
            )

        detection_count = Int32()
        detection_count.data = len(clusters)
        self.detection_count_pub.publish(detection_count)
        self.detections_pub.publish(markers)


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
