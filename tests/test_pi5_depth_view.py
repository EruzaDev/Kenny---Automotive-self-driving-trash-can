"""Projection checks run with system Python after sourcing ROS 2 Jazzy."""
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy", reason="Pi visualization tests require system ROS Python")
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from scripts.pi5_depth_view import DepthView


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def view():
    rclpy.init(args=[])
    node = DepthView(SimpleNamespace(near=0.5, far=3.0, fps=5))
    node.cloud_pub = Capture()
    node.ray_pub = Capture()
    info = CameraInfo(header=Header(frame_id="depth_optical"), width=8, height=8)
    info.k = [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 1.0]
    node.camera_info(info)
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.mark.parametrize("encoding,dtype,scale", [
    ("16UC1", "<u2", 1000), ("16UC1", ">u2", 1000),
    ("32FC1", "<f4", 1), ("32FC1", ">f4", 1),
])
def test_projection_units_row_padding_invalid_depth_and_range_colors(view, encoding, dtype, scale):
    # Two extra values pad each row and must not be interpreted as image pixels.
    raw = np.zeros((8, 10), dtype=dtype)
    raw[:, 8:] = 123
    raw[0, 0], raw[0, 4], raw[4, 4] = 0.25 * scale, 3 * scale, 0.5 * scale
    image = Image(header=Header(frame_id="depth_optical"), width=8, height=8,
                  encoding=encoding, is_bigendian=int(dtype.startswith(">")),
                  step=raw.strides[0], data=raw.tobytes())
    view.depth_image(image)
    cloud = view.cloud_pub.messages[0]
    points = np.frombuffer(cloud.data, dtype=[("x", "<f4"), ("y", "<f4"),
                                            ("z", "<f4"), ("rgb", "<u4")])
    assert cloud.width == 3  # Zero at the fourth sampled pixel is omitted.
    np.testing.assert_allclose(points["z"], [0.25, 3, 0.5])
    np.testing.assert_allclose(points["x"], [0, 3, 0.5])
    np.testing.assert_allclose(points["y"], [0, 0, 0.5])
    assert points["rgb"][0] == 0xff0000
    assert points["rgb"][1] == 0x00ff00
    rays = view.ray_pub.messages[0].markers[0]
    assert len(rays.points) > 0
    assert len(rays.points) == len(rays.colors)
    assert all(point.x == point.y == point.z == 0 for point in rays.points[::2])
    assert all(point.z > 0 for point in rays.points[1::2])


def test_mismatched_camera_frame_does_not_project(view):
    view.depth_image(Image(header=Header(frame_id="different_camera"), width=8,
                           height=8, encoding="16UC1", step=16, data=bytes(128)))
    assert not view.cloud_pub.messages
    assert not view.ray_pub.messages
