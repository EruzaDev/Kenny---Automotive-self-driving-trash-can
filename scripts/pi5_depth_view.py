"""Project measured depth into a distance-colored cloud and sparse range rays."""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def distance_colors(distance, near, far):
    fraction = np.clip((distance - near) / (far - near), 0, 1)
    red = (255 * np.minimum(1, 2 * (1 - fraction))).astype(np.uint32)
    green = (255 * np.minimum(1, 2 * fraction)).astype(np.uint32)
    return (red << 16) | (green << 8)


class DepthView(Node):
    def __init__(self, args):
        super().__init__("kenny_depth_view")
        self.args = args
        self.info = None
        self.last_update = 0
        self.frames = 0
        self.cloud_pub = self.create_publisher(PointCloud2, "/kenny/depth/points", qos_profile_sensor_data)
        self.ray_pub = self.create_publisher(MarkerArray, "/kenny/depth/rays", 2)
        self.info_sub = self.create_subscription(CameraInfo, "/camera/depth/camera_info",
                                                 self.camera_info, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_raw",
                                                  self.depth_image, qos_profile_sensor_data)
        self.get_logger().info(f"Measured depth view: red <= {args.near} m, green >= {args.far} m")

    def camera_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.info = msg

    def depth_image(self, msg):
        now = time.monotonic()
        if self.info is None or now - self.last_update < 1 / self.args.fps:
            return
        if (self.info.width, self.info.height, self.info.header.frame_id) != (msg.width, msg.height, msg.header.frame_id):
            return
        encoding = {"16UC1": ("u2", 0.001), "32FC1": ("f4", 1.0)}.get(msg.encoding)
        if encoding is None:
            return
        dtype = np.dtype((">" if msg.is_bigendian else "<") + encoding[0])
        depth = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                           strides=(msg.step, dtype.itemsize)).astype(np.float32) * encoding[1]
        self.last_update = now
        fx, fy, cx, cy = (self.info.k[index] for index in (0, 4, 2, 5))
        rows, cols = np.mgrid[0:msg.height:4, 0:msg.width:4]
        z = depth[rows, cols]
        valid = np.isfinite(z) & (z >= 0.1) & (z <= 8)
        z = z[valid]
        x = (cols[valid] - cx) * z / fx
        y = (rows[valid] - cy) * z / fy
        ranges = np.sqrt(x * x + y * y + z * z)
        points = np.empty(len(z), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        points["x"], points["y"], points["z"] = x, y, z
        points["rgb"] = distance_colors(ranges, self.args.near, self.args.far)
        fields = [PointField(name=name, offset=index * 4,
                             datatype=PointField.UINT32 if name == "rgb" else PointField.FLOAT32,
                             count=1) for index, name in enumerate(("x", "y", "z", "rgb"))]
        self.cloud_pub.publish(PointCloud2(header=msg.header, height=1, width=len(points),
                                           fields=fields, is_bigendian=False, point_step=16,
                                           row_step=16 * len(points), data=points.tobytes(), is_dense=True))
        marker = Marker(header=msg.header, ns="depth_range_rays", id=0,
                        type=Marker.LINE_LIST, action=Marker.ADD)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.004
        marker.color.a = 0.45
        marker.lifetime.sec = 1
        for row in np.linspace(0, msg.height - 1, 16, dtype=int):
            for col in np.linspace(0, msg.width - 1, 24, dtype=int):
                dz = float(depth[row, col])
                if not np.isfinite(dz) or not 0.1 <= dz <= 8:
                    continue
                dx, dy = float((col - cx) * dz / fx), float((row - cy) * dz / fy)
                color = int(distance_colors(np.array([np.sqrt(dx * dx + dy * dy + dz * dz)]),
                                            self.args.near, self.args.far)[0])
                rgba = ColorRGBA(r=((color >> 16) & 255) / 255.0,
                                 g=((color >> 8) & 255) / 255.0, b=0.0, a=0.45)
                marker.points.extend([Point(), Point(x=dx, y=dy, z=dz)])
                marker.colors.extend([rgba, rgba])
        self.ray_pub.publish(MarkerArray(markers=[marker]))
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(f"Publishing {len(points)} measured 3D points and {len(marker.points)//2} depth rays")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--near", type=float, default=0.5)
    parser.add_argument("--far", type=float, default=3.0)
    parser.add_argument("--fps", type=float, default=5.0)
    args = parser.parse_args()
    if not 0 < args.near < args.far or args.fps <= 0:
        parser.error("Require 0 < near < far and positive fps")
    rclpy.init(args=[])
    node = DepthView(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
