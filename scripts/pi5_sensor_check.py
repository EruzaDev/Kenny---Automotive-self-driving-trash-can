"""Start sensor bringup, measure ROS streams, save evidence, and stop drivers."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--attach", action="store_true", help="Check already-running drivers")
    parser.add_argument("--colored", action="store_true", help="Also require the registered RGB cloud")
    parser.add_argument("--visualization", action="store_true", help="Also check the depth cloud and rays")
    args = parser.parse_args()
    if args.seconds < 10:
        parser.error("Use at least 10 seconds for startup and sampling")
    repo = Path(__file__).resolve().parents[1]
    output = repo / "artifacts/pi5"
    output.mkdir(parents=True, exist_ok=True)
    topics = {
        "/scan": LaserScan,
        "/camera/color/image_raw": Image,
        "/camera/color/camera_info": CameraInfo,
        "/camera/depth/image_raw": Image,
        "/camera/depth/camera_info": CameraInfo,
        "/camera/depth/points": PointCloud2,
    }
    if args.colored:
        topics["/camera/depth_registered/points"] = PointCloud2
    if args.visualization:
        topics["/kenny/depth/points"] = PointCloud2
        topics["/kenny/depth/rays"] = MarkerArray
    stats = {topic: {"count": 0} for topic in topics}
    rclpy.init()
    node = rclpy.create_node("kenny_pi5_sensor_check")

    def record(topic, msg):
        stat = stats[topic]
        now = time.monotonic()
        stat.setdefault("first", now)
        if "last" in stat:
            stat["max_gap_s"] = max(stat.get("max_gap_s", 0), now - stat["last"])
        stat["last"] = now
        stat["count"] += 1
        if isinstance(msg, MarkerArray):
            stat["rays"] = sum(len(marker.points) // 2 for marker in msg.markers)
            return
        stat["frame_id"] = msg.header.frame_id
        if isinstance(msg, LaserScan):
            stat["points"] = len(msg.ranges)
            valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
            stat["valid_ranges"] = len(valid)
            stat["nearest_m"] = min(valid) if valid else None
        elif isinstance(msg, Image):
            stat["resolution"] = [msg.width, msg.height]
            stat["encoding"] = msg.encoding
        elif isinstance(msg, CameraInfo):
            stat["intrinsics_present"] = bool(msg.k[0] > 0 and msg.k[4] > 0)
        elif isinstance(msg, PointCloud2):
            stat["points"] = msg.width * msg.height
            stat["fields"] = [field.name for field in msg.fields]
            if "rgb" in stat["fields"] and "sampled_distinct_colors" not in stat:
                offsets = {field.name: field.offset for field in msg.fields}
                endian = ">" if msg.is_bigendian else "<"
                colors = set()
                valid = 0
                for index in range(0, stat["points"], max(1, stat["points"] // 1024)):
                    row, col = divmod(index, msg.width)
                    start = row * msg.row_step + col * msg.point_step
                    xyz = [struct.unpack_from(endian + "f", msg.data, start + offsets[key])[0]
                           for key in ("x", "y", "z")]
                    if all(math.isfinite(value) for value in xyz):
                        valid += 1
                        colors.add(struct.unpack_from(endian + "I", msg.data,
                                                      start + offsets["rgb"])[0] & 0xffffff)
                stat["sampled_valid_xyz"] = valid
                stat["sampled_distinct_colors"] = len(colors)

    subscriptions = [node.create_subscription(kind, topic, lambda msg, t=topic: record(t, msg),
                                               qos_profile_sensor_data)
                     for topic, kind in topics.items()]
    try:
        log_path = output / ("live-check.log" if args.attach else "sensor-bringup.log")
        with log_path.open("w") as log:
            process = None
            if not args.attach:
                command = ["ros2", "launch", str(repo / "scripts/pi5_sensors.launch.py")]
                if args.colored:
                    command.append("colored_cloud:=true")
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           start_new_session=True)
            try:
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline and (process is None or process.poll() is None):
                    rclpy.spin_once(node, timeout_sec=0.2)
            finally:
                if process is not None and process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=10)
        for stat in stats.values():
            if "last" in stat:
                stat["last_received_age_s"] = round(time.monotonic() - stat["last"], 3)
            if stat["count"] > 1:
                stat["hz"] = round((stat["count"] - 1) / (stat["last"] - stat["first"]), 2)
            stat.pop("first", None)
            stat.pop("last", None)
        passed = all(stat["count"] > 1 and stat.get("last_received_age_s", math.inf) < 5
                     for stat in stats.values())
        if args.colored:
            cloud = stats["/camera/depth_registered/points"]
            passed = passed and cloud.get("sampled_valid_xyz", 0) > 0 and cloud.get("sampled_distinct_colors", 0) > 1
        result = {"duration_requested_s": args.seconds, "streams_received": passed, "topics": stats}
        result_path = output / ("live-check.json" if args.attach else "sensor-check.json")
        result_path.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
