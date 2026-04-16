"""Publish MLX90640 thermal frames, with a mock fallback for bench development."""

from __future__ import annotations

from array import array
import importlib
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .mission_contract import TOPICS
from .thermal_common import (
    MLX90640_NATIVE_HEIGHT,
    MLX90640_NATIVE_WIDTH,
    apply_frame_orientation,
    camera_matrix_from_fov,
    oriented_fov_radians,
)


class ThermalBackendError(RuntimeError):
    """Raised when the requested thermal-camera backend cannot be created."""


class MockThermalFrameProvider:
    """Synthetic 32x24 thermal stream used when hardware is absent."""

    def __init__(
        self,
        *,
        ambient_temperature_c: float,
        hot_spot_temperature_c: float,
        hot_spot_radius_px: float,
        hot_spot_center_x: float,
        hot_spot_center_y: float,
        enable_hot_spot: bool,
        animate_hot_spot: bool,
        noise_stddev_c: float,
        rotation_degrees: int,
        flip_horizontal: bool,
        flip_vertical: bool,
    ):
        self.ambient_temperature_c = float(ambient_temperature_c)
        self.hot_spot_temperature_c = float(hot_spot_temperature_c)
        self.hot_spot_radius_px = float(hot_spot_radius_px)
        self.hot_spot_center_x = float(hot_spot_center_x)
        self.hot_spot_center_y = float(hot_spot_center_y)
        self.enable_hot_spot = bool(enable_hot_spot)
        self.animate_hot_spot = bool(animate_hot_spot)
        self.noise_stddev_c = max(0.0, float(noise_stddev_c))
        self.rotation_degrees = int(rotation_degrees)
        self.flip_horizontal = bool(flip_horizontal)
        self.flip_vertical = bool(flip_vertical)
        self.rng = np.random.default_rng()

    def read_frame(self, time_s: float) -> np.ndarray:
        """Generate one simulated thermal frame."""
        thermal = np.full(
            (MLX90640_NATIVE_HEIGHT, MLX90640_NATIVE_WIDTH),
            self.ambient_temperature_c,
            dtype=np.float32,
        )

        if self.enable_hot_spot:
            center_x = self.hot_spot_center_x
            center_y = self.hot_spot_center_y
            if self.animate_hot_spot:
                center_x += 4.0 * math.sin(time_s * 0.35)
                center_y += 2.0 * math.cos(time_s * 0.25)

            yy, xx = np.mgrid[0:MLX90640_NATIVE_HEIGHT, 0:MLX90640_NATIVE_WIDTH]
            dist = np.sqrt((xx - center_x) ** 2 + (yy - center_y) ** 2)
            thermal[dist <= self.hot_spot_radius_px] = self.hot_spot_temperature_c
            ring_mask = np.logical_and(
                dist > self.hot_spot_radius_px,
                dist <= self.hot_spot_radius_px + 1.5,
            )
            thermal[ring_mask] = self.ambient_temperature_c + 5.0

        if self.noise_stddev_c > 0.0:
            thermal += self.rng.normal(
                loc=0.0,
                scale=self.noise_stddev_c,
                size=thermal.shape,
            ).astype(np.float32)

        return apply_frame_orientation(
            thermal,
            rotation_degrees=self.rotation_degrees,
            flip_horizontal=self.flip_horizontal,
            flip_vertical=self.flip_vertical,
        )


class Mlx90640FrameProvider:
    """Read 32x24 thermal frames from an MLX90640 over I2C."""

    def __init__(
        self,
        *,
        i2c_address: int,
        i2c_frequency_hz: int,
        refresh_rate_hz: float,
        rotation_degrees: int,
        flip_horizontal: bool,
        flip_vertical: bool,
    ):
        try:
            board = importlib.import_module("board")
            busio = importlib.import_module("busio")
            adafruit_mlx90640 = importlib.import_module("adafruit_mlx90640")
        except ImportError as exc:
            raise ThermalBackendError(
                "MLX90640 backend requested but the Raspberry Pi libraries are not installed. "
                "Install adafruit-blinka and adafruit-circuitpython-mlx90640 on the Pi."
            ) from exc

        try:
            self.i2c = busio.I2C(board.SCL, board.SDA, frequency=int(i2c_frequency_hz))
            self.sensor = adafruit_mlx90640.MLX90640(self.i2c, address=int(i2c_address))
        except Exception as exc:
            raise ThermalBackendError(
                f"Unable to initialize the MLX90640 at I2C address 0x{int(i2c_address):02X}."
            ) from exc

        self.sensor.refresh_rate = getattr(
            adafruit_mlx90640.RefreshRate,
            self._refresh_enum_name(float(refresh_rate_hz)),
        )
        self.selected_refresh_rate_hz = self._nearest_supported_rate(float(refresh_rate_hz))
        self.rotation_degrees = int(rotation_degrees)
        self.flip_horizontal = bool(flip_horizontal)
        self.flip_vertical = bool(flip_vertical)
        self.frame_buffer = [0.0] * (MLX90640_NATIVE_WIDTH * MLX90640_NATIVE_HEIGHT)

    @staticmethod
    def _nearest_supported_rate(refresh_rate_hz: float) -> float:
        supported = (0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0)
        return min(supported, key=lambda value: abs(value - refresh_rate_hz))

    @classmethod
    def _refresh_enum_name(cls, refresh_rate_hz: float) -> str:
        names = {
            0.5: "REFRESH_0_5_HZ",
            1.0: "REFRESH_1_HZ",
            2.0: "REFRESH_2_HZ",
            4.0: "REFRESH_4_HZ",
            8.0: "REFRESH_8_HZ",
            16.0: "REFRESH_16_HZ",
            32.0: "REFRESH_32_HZ",
            64.0: "REFRESH_64_HZ",
        }
        return names[cls._nearest_supported_rate(refresh_rate_hz)]

    def read_frame(self, _: float) -> np.ndarray:
        """Read one sensor frame and reshape it to 2-D."""
        try:
            self.sensor.getFrame(self.frame_buffer)
        except Exception as exc:
            raise ThermalBackendError("Failed to read a frame from the MLX90640.") from exc

        thermal = np.asarray(self.frame_buffer, dtype=np.float32).reshape(
            (MLX90640_NATIVE_HEIGHT, MLX90640_NATIVE_WIDTH)
        )
        return apply_frame_orientation(
            thermal,
            rotation_degrees=self.rotation_degrees,
            flip_horizontal=self.flip_horizontal,
            flip_vertical=self.flip_vertical,
        )


class ThermalNode(Node):
    """ROS2 publisher for thermal frames and camera intrinsics."""

    def __init__(self):
        super().__init__("thermal_node")

        self.declare_parameter("backend", "auto")
        self.declare_parameter("fallback_to_mock", True)
        self.declare_parameter("image_topic", TOPICS.thermal_image)
        self.declare_parameter("camera_info_topic", TOPICS.thermal_camera_info)
        self.declare_parameter("frame_id", "thermal_link")
        self.declare_parameter("publish_rate_hz", 4.0)
        self.declare_parameter("horizontal_fov_deg", 110.0)
        self.declare_parameter("vertical_fov_deg", 75.0)
        self.declare_parameter("rotation_degrees", 0)
        self.declare_parameter("flip_horizontal", False)
        self.declare_parameter("flip_vertical", False)
        self.declare_parameter("i2c_address", 0x33)
        self.declare_parameter("i2c_frequency_hz", 800000)
        self.declare_parameter("refresh_rate_hz", 4.0)
        self.declare_parameter("ambient_temperature_c", 22.0)
        self.declare_parameter("hot_spot_temperature_c", 36.0)
        self.declare_parameter("hot_spot_radius_px", 3.0)
        self.declare_parameter("hot_spot_center_x", 16.0)
        self.declare_parameter("hot_spot_center_y", 12.0)
        self.declare_parameter("enable_hot_spot", True)
        self.declare_parameter("animate_hot_spot", True)
        self.declare_parameter("noise_stddev_c", 0.35)

        self.backend_name = str(self.get_parameter("backend").value).strip().lower()
        self.fallback_to_mock = bool(self.get_parameter("fallback_to_mock").value)
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.horizontal_fov_rad = math.radians(
            float(self.get_parameter("horizontal_fov_deg").value)
        )
        self.vertical_fov_rad = math.radians(
            float(self.get_parameter("vertical_fov_deg").value)
        )
        self.rotation_degrees = int(self.get_parameter("rotation_degrees").value)
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.flip_vertical = bool(self.get_parameter("flip_vertical").value)

        self.frame_provider = self._create_frame_provider()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_frame)

    def _create_frame_provider(self):
        backend_preference = self.backend_name or "auto"
        attempted_mlx = backend_preference in {"auto", "mlx90640"}

        if attempted_mlx:
            try:
                provider = Mlx90640FrameProvider(
                    i2c_address=int(self.get_parameter("i2c_address").value),
                    i2c_frequency_hz=int(self.get_parameter("i2c_frequency_hz").value),
                    refresh_rate_hz=float(self.get_parameter("refresh_rate_hz").value),
                    rotation_degrees=self.rotation_degrees,
                    flip_horizontal=self.flip_horizontal,
                    flip_vertical=self.flip_vertical,
                )
                self.get_logger().info(
                    "Thermal backend: MLX90640 at "
                    f"0x{int(self.get_parameter('i2c_address').value):02X} "
                    f"({provider.selected_refresh_rate_hz:.1f} Hz)."
                )
                return provider
            except ThermalBackendError as exc:
                if backend_preference == "mlx90640" and not self.fallback_to_mock:
                    raise
                self.get_logger().warning(f"{exc} Falling back to the mock thermal stream.")

        if backend_preference not in {"auto", "mock", "mlx90640"}:
            raise ThermalBackendError("backend must be one of: auto, mlx90640, mock.")

        self.get_logger().info("Thermal backend: mock MLX90640 stream.")
        return MockThermalFrameProvider(
            ambient_temperature_c=float(self.get_parameter("ambient_temperature_c").value),
            hot_spot_temperature_c=float(self.get_parameter("hot_spot_temperature_c").value),
            hot_spot_radius_px=float(self.get_parameter("hot_spot_radius_px").value),
            hot_spot_center_x=float(self.get_parameter("hot_spot_center_x").value),
            hot_spot_center_y=float(self.get_parameter("hot_spot_center_y").value),
            enable_hot_spot=bool(self.get_parameter("enable_hot_spot").value),
            animate_hot_spot=bool(self.get_parameter("animate_hot_spot").value),
            noise_stddev_c=float(self.get_parameter("noise_stddev_c").value),
            rotation_degrees=self.rotation_degrees,
            flip_horizontal=self.flip_horizontal,
            flip_vertical=self.flip_vertical,
        )

    def _camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        """Build CameraInfo using the configured thermal FOV."""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height

        horizontal_fov_rad, vertical_fov_rad = oriented_fov_radians(
            self.horizontal_fov_rad,
            self.vertical_fov_rad,
            rotation_degrees=self.rotation_degrees,
        )
        fx, fy, cx, cy = camera_matrix_from_fov(
            width=width,
            height=height,
            horizontal_fov_rad=horizontal_fov_rad,
            vertical_fov_rad=vertical_fov_rad,
        )
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return info

    def _read_frame(self) -> Optional[np.ndarray]:
        try:
            return self.frame_provider.read_frame(self.get_clock().now().nanoseconds / 1e9)
        except ThermalBackendError as exc:
            self.get_logger().warning(f"Thermal frame read failed: {exc}")
            return None

    def _publish_frame(self):
        thermal = self._read_frame()
        if thermal is None:
            return

        stamp = self.get_clock().now().to_msg()
        height, width = thermal.shape
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = height
        image.width = width
        image.encoding = "32FC1"
        image.is_bigendian = False
        image.step = width * 4
        image.data = array("B", thermal.astype(np.float32, copy=False).tobytes())

        self.image_pub.publish(image)
        self.camera_info_pub.publish(self._camera_info(stamp, width=width, height=height))


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
