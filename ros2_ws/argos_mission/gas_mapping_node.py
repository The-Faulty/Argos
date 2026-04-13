"""Project gas sensor readings into a simple 2D hazard occupancy grid."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

from .mission_contract import TOPICS


class GasMappingNode(Node):
    def __init__(self):
        super().__init__("gas_mapping_node")

        self.declare_parameter("gas_topic", TOPICS.gas)
        self.declare_parameter("odometry_topic", TOPICS.odometry)
        self.declare_parameter("hazard_map_topic", TOPICS.hazard_map)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("grid_width_m", 10.0)
        self.declare_parameter("grid_height_m", 10.0)
        self.declare_parameter("resolution_m", 0.10)
        self.declare_parameter("gas_baseline", 0.0)
        self.declare_parameter("gas_alert_threshold", 1.0)
        self.declare_parameter("kernel_radius_cells", 2)
        self.declare_parameter("decay_per_publish", 0.99)

        gas_topic = self.get_parameter("gas_topic").value
        odometry_topic = self.get_parameter("odometry_topic").value
        hazard_map_topic = self.get_parameter("hazard_map_topic").value

        self.map_frame = self.get_parameter("map_frame").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.grid_width_m = float(self.get_parameter("grid_width_m").value)
        self.grid_height_m = float(self.get_parameter("grid_height_m").value)
        self.resolution_m = float(self.get_parameter("resolution_m").value)
        self.gas_baseline = float(self.get_parameter("gas_baseline").value)
        self.gas_alert_threshold = float(
            self.get_parameter("gas_alert_threshold").value
        )
        self.kernel_radius_cells = int(
            self.get_parameter("kernel_radius_cells").value
        )
        self.decay_per_publish = float(self.get_parameter("decay_per_publish").value)

        self.width_cells = max(1, int(round(self.grid_width_m / self.resolution_m)))
        self.height_cells = max(1, int(round(self.grid_height_m / self.resolution_m)))
        self.origin_x = -self.grid_width_m / 2.0
        self.origin_y = -self.grid_height_m / 2.0
        self.grid = np.zeros((self.height_cells, self.width_cells), dtype=np.float32)
        self.latest_pose_xy = None

        self.map_pub = self.create_publisher(OccupancyGrid, hazard_map_topic, 10)
        self.create_subscription(Float32, gas_topic, self._gas_callback, 10)
        self.create_subscription(Odometry, odometry_topic, self._odometry_callback, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_map)

    def _odometry_callback(self, msg: Odometry):
        self.latest_pose_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def _world_to_grid(self, x_m: float, y_m: float):
        col = int(math.floor((x_m - self.origin_x) / self.resolution_m))
        row = int(math.floor((y_m - self.origin_y) / self.resolution_m))
        if 0 <= row < self.height_cells and 0 <= col < self.width_cells:
            return row, col
        return None

    def _reading_to_intensity(self, reading: float) -> float:
        span = max(1e-6, self.gas_alert_threshold - self.gas_baseline)
        return float(np.clip((reading - self.gas_baseline) / span, 0.0, 1.0))

    def _deposit(self, row: int, col: int, intensity: float):
        radius = max(0, self.kernel_radius_cells)
        for delta_row in range(-radius, radius + 1):
            for delta_col in range(-radius, radius + 1):
                next_row = row + delta_row
                next_col = col + delta_col
                if not (0 <= next_row < self.height_cells and 0 <= next_col < self.width_cells):
                    continue
                distance = math.sqrt(delta_row ** 2 + delta_col ** 2)
                if distance > radius + 0.25:
                    continue
                weight = max(0.0, 1.0 - distance / max(1.0, radius + 1.0))
                self.grid[next_row, next_col] = max(
                    self.grid[next_row, next_col],
                    intensity * weight,
                )

    def _gas_callback(self, msg: Float32):
        if self.latest_pose_xy is None:
            return

        intensity = self._reading_to_intensity(float(msg.data))
        if intensity <= 0.0:
            return

        cell = self._world_to_grid(*self.latest_pose_xy)
        if cell is None:
            return
        self._deposit(cell[0], cell[1], intensity)

    def _publish_map(self):
        self.grid *= self.decay_per_publish
        occupancy = np.clip(np.rint(self.grid * 100.0), 0.0, 100.0).astype(np.int8)

        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = self.map_frame
        grid_msg.info.map_load_time = grid_msg.header.stamp
        grid_msg.info.resolution = self.resolution_m
        grid_msg.info.width = self.width_cells
        grid_msg.info.height = self.height_cells
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.orientation.w = 1.0
        grid_msg.data = occupancy.flatten().tolist()
        self.map_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GasMappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
