"""Publish a mock MLX90640-style thermal stream for bench testing."""

from array import array
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .mission_contract import TOPICS


class ThermalNode(Node):
    def __init__(self):
        super().__init__("thermal_node")

        self.declare_parameter("image_topic", TOPICS.thermal_image)
        self.declare_parameter("camera_info_topic", TOPICS.thermal_camera_info)
        self.declare_parameter("frame_id", "thermal_link")
        self.declare_parameter("publish_rate_hz", 4.0)
        self.declare_parameter("width", 32)
        self.declare_parameter("height", 24)
        self.declare_parameter("ambient_temperature_c", 22.0)
        self.declare_parameter("hot_spot_temperature_c", 36.0)
        self.declare_parameter("hot_spot_radius_px", 3.0)
        self.declare_parameter("hot_spot_center_x", 16.0)
        self.declare_parameter("hot_spot_center_y", 12.0)
        self.declare_parameter("horizontal_fov_deg", 110.0)
        self.declare_parameter("vertical_fov_deg", 75.0)
        self.declare_parameter("enable_hot_spot", True)
        self.declare_parameter("animate_hot_spot", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.ambient_temperature_c = float(
            self.get_parameter("ambient_temperature_c").value
        )
        self.hot_spot_temperature_c = float(
            self.get_parameter("hot_spot_temperature_c").value
        )
        self.hot_spot_radius_px = float(self.get_parameter("hot_spot_radius_px").value)
        self.hot_spot_center_x = float(self.get_parameter("hot_spot_center_x").value)
        self.hot_spot_center_y = float(self.get_parameter("hot_spot_center_y").value)
        self.horizontal_fov_rad = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value)
        )
        self.vertical_fov_rad = math.radians(
            float(self.get_parameter("vertical_fov_deg").value)
        )
        self.enable_hot_spot = bool(self.get_parameter("enable_hot_spot").value)
        self.animate_hot_spot = bool(self.get_parameter("animate_hot_spot").value)

        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_frame)

    def _camera_info(self, stamp) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height

        fx = self.width / (2.0 * math.tan(self.horizontal_fov_rad / 2.0))
        fy = self.height / (2.0 * math.tan(self.vertical_fov_rad / 2.0))
        cx = (self.width - 1) / 2.0
        cy = (self.height - 1) / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return info

    def _build_thermal_image(self) -> np.ndarray:
        thermal = np.full(
            (self.height, self.width),
            self.ambient_temperature_c,
            dtype=np.float32,
        )

        if not self.enable_hot_spot:
            return thermal

        center_x = self.hot_spot_center_x
        center_y = self.hot_spot_center_y
        if self.animate_hot_spot:
            time_s = self.get_clock().now().nanoseconds / 1e9
            center_x += 4.0 * math.sin(time_s * 0.35)
            center_y += 2.0 * math.cos(time_s * 0.25)

        yy, xx = np.mgrid[0:self.height, 0:self.width]
        dist = np.sqrt((xx - center_x) ** 2 + (yy - center_y) ** 2)
        mask = dist <= self.hot_spot_radius_px
        thermal[mask] = self.hot_spot_temperature_c

        ring_mask = np.logical_and(
            dist > self.hot_spot_radius_px,
            dist <= self.hot_spot_radius_px + 1.5,
        )
        thermal[ring_mask] = self.ambient_temperature_c + 5.0
        return thermal

    def _publish_frame(self):
        stamp = self.get_clock().now().to_msg()
        thermal = self._build_thermal_image()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = "32FC1"
        image.is_bigendian = False
        image.step = self.width * 4
        image.data = array("B", thermal.tobytes())

        self.image_pub.publish(image)
        self.camera_info_pub.publish(self._camera_info(stamp))


def main(args=None):
    rclpy.init(args=args)
    node = ThermalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
