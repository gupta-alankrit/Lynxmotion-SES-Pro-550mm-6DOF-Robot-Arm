#!/usr/bin/env python3

"""
RealSense D455 RGB + aligned depth viewer using ROS 2.

Subscribes to synchronized color image, aligned depth image, and color
camera_info via the shared `RGBDSubscriberBaseNode`. Displays the color image
and a colorized depth visualization in two OpenCV windows and optionally
prints the depth at a selected pixel.
"""

import time

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo

from machine_vision_pkg.realsense_synced_rgbd import RealsenseSyncedRGBDNode


class RealsenseRGBDViewerNode(RealsenseSyncedRGBDNode):
    def __init__(self):
        super().__init__("realsense_rgbd_viewer")

        self.declare_parameter("display", True)
        self.declare_parameter("mirror_display", False)
        self.declare_parameter("print_selected_px_depth", True)
        self.declare_parameter("print_interval_sec", 0.25)
        self.declare_parameter("selected_px_ratio", 0.5)
        self.declare_parameter("selected_py_ratio", 0.5)
        self.declare_parameter("depth_visualization_scale", 50.0)

        self.display = bool(self.get_parameter("display").value)
        self.mirror_display = bool(self.get_parameter("mirror_display").value)
        self.print_selected_px_depth = bool(self.get_parameter("print_selected_px_depth").value)
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)
        self.selected_px_ratio = float(self.get_parameter("selected_px_ratio").value)
        self.selected_py_ratio = float(self.get_parameter("selected_py_ratio").value)
        self.depth_visualization_scale = float(self.get_parameter("depth_visualization_scale").value)

        self.last_print_time = 0.0
        self.get_logger().info("Realsense RGBD viewer node started.")

    def process(
        self, color_bgr: np.ndarray, depth_meters: np.ndarray, camera_info: CameraInfo
    ) -> None:
        h, w = depth_meters.shape[:2]
        u = int(np.clip(round(self.selected_px_ratio * w), 0, w - 1))
        v = int(np.clip(round(self.selected_py_ratio * h), 0, h - 1))

        center_depth_m = float(depth_meters[v, u])
        valid_depth = np.isfinite(center_depth_m) and center_depth_m > 0.0

        now = time.time()
        if self.print_selected_px_depth and (now - self.last_print_time) >= self.print_interval_sec:
            if valid_depth:
                self.get_logger().info(
                    f"Center depth at ({u}, {v}): {center_depth_m:.3f} m"
                )
            else:
                self.get_logger().info(f"Center depth at ({u}, {v}): invalid")
            self.last_print_time = now

        if not self.display:
            return

        color_display = color_bgr.copy()
        depth_display_raw = cv2.convertScaleAbs(
            depth_meters, alpha=self.depth_visualization_scale
        )
        depth_colormap = cv2.applyColorMap(depth_display_raw, cv2.COLORMAP_JET)

        display_u = u
        if self.mirror_display:
            color_display = cv2.flip(color_display, 1)
            depth_colormap = cv2.flip(depth_colormap, 1)
            display_u = w - 1 - u

        label = f"{center_depth_m:.3f} m" if valid_depth else "invalid"

        cv2.circle(color_display, (display_u, v), 5, (0, 0, 255), -1)
        cv2.putText(
            color_display, label, (display_u + 10, v),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
        )
        cv2.circle(depth_colormap, (display_u, v), 5, (255, 255, 255), -1)
        cv2.putText(
            depth_colormap, label, (display_u + 10, v),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
        )

        cv2.imshow("D455 RGB from ROS2", color_display)
        cv2.imshow("D455 Aligned Depth from ROS2", depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("q pressed. Shutting down.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseRGBDViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
