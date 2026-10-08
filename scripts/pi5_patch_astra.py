"""Apply the tested Jazzy compatibility edits to the pinned Astra driver."""
import argparse
from pathlib import Path
import subprocess

REVISION = "f7e71d9ce806e788cb48d8580aac2c778fba4214"
HEADER_EDITS = {
    "include/astra_camera/ob_camera_node.h": [("cv_bridge/cv_bridge.h", "cv_bridge/cv_bridge.hpp")],
    "include/astra_camera/point_cloud_proc/point_cloud_xyz.h": [("image_geometry/pinhole_camera_model.h", "image_geometry/pinhole_camera_model.hpp")],
    "include/astra_camera/point_cloud_proc/point_cloud_xyzrgb.h": [
        ("image_geometry/pinhole_camera_model.h", "image_geometry/pinhole_camera_model.hpp"),
        ("cv_bridge/cv_bridge.h", "cv_bridge/cv_bridge.hpp")],
    "src/ob_camera_node.cpp": [("cv_bridge/cv_bridge.h", "cv_bridge/cv_bridge.hpp")],
    "src/uvc_camera_driver.cpp": [("cv_bridge/cv_bridge.h", "cv_bridge/cv_bridge.hpp")],
    "src/point_cloud_proc/point_cloud_xyz.cpp": [("image_geometry/pinhole_camera_model.h", "image_geometry/pinhole_camera_model.hpp")],
    "src/point_cloud_proc/point_cloud_xyzrgb.cpp": [
        ("image_geometry/pinhole_camera_model.h", "image_geometry/pinhole_camera_model.hpp"),
        ("cv_bridge/cv_bridge.h", "cv_bridge/cv_bridge.hpp")],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path.home() / "astra_ws/src/ros2_astra_camera")
    parser.add_argument("--check", action="store_true", help="Validate edits without writing files")
    args = parser.parse_args()
    revision = subprocess.check_output(["git", "-C", str(args.source), "rev-parse", "HEAD"], text=True).strip()
    if revision != REVISION:
        parser.error(f"Expected Astra revision {REVISION}, found {revision}")
    edits = dict(HEADER_EDITS)
    edits["CMakeLists.txt"] = [('set(CMAKE_BUILD_TYPE "Debug")',
                               'if(NOT CMAKE_BUILD_TYPE)\n  set(CMAKE_BUILD_TYPE "Release")\nendif()')]
    edits["include/astra_camera/constants.h"] = [("#include <cstdlib>\n", "#include <cstdlib>\n#include <cstdint>\n")]
    callback = "rclcpp::node_interfaces::NodeParametersInterface::"
    for path in ("include/astra_camera/ros_param_backend.h", "src/ros_param_backend.cpp"):
        edits[path] = [(callback + "OnParametersSetCallbackType", callback + "OnSetParametersCallbackType")]
    pending = []
    for relative, replacements in edits.items():
        path = args.source / "astra_camera" / relative
        original = path.read_text()
        updated = original
        for old, new in replacements:
            # Check the new form first: old header paths are prefixes of new ones.
            if updated.count(new) == 1:
                continue
            if updated.count(old) != 1:
                parser.error(f"Unexpected source content in {path}; no files were changed")
            updated = updated.replace(old, new, 1)
        if original != updated:
            pending.append((path, updated))
    if not args.check:
        for path, updated in pending:
            path.write_text(updated)
    print(f"Astra Jazzy compatibility: {len(pending)} files {'would change' if args.check else 'updated'}")


if __name__ == "__main__":
    main()
