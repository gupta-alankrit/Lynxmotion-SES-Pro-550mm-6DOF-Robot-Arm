#!/usr/bin/env python3

"""
3D reprojection node for the pro_arm + RealSense pipeline.

Subscribes (via RealsenseSyncedRGBDNode) to synchronized color +
aligned-depth + camera_info, and separately to /classifier/detections.
For every incoming detection it:
  1. Looks up a robust median depth in a k×k window around the bbox centre,
  2. Deprojects (u, v, z) to camera-frame XYZ using fx, fy, cx, cy from the
     latest CameraInfo (OpenCV pinhole),
  3. Applies a static camera→arm rigid transform (built once from ROS
     parameters describing the camera's position and orientation in the
     arm base frame),
  4. Publishes vision_msgs/Detection3DArray on /reprojection_3D/targets,
     header.frame_id = arm base link.

This is the ROS 2 / RealSense / OpenCV-convention port of the logic in
~/machine_vision_isaac/.../geometry/reprojection_3D.py. The USD-convention
conversion present in the Isaac version is *not* applied here because
RealSense and ROS already publish images in OpenCV convention (x right,
y down, z forward).
"""

import math

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from machine_vision_pkg.realsense_synced_rgbd import RealsenseSyncedRGBDNode


def robust_depth_at(depth_hw: np.ndarray, u: float, v: float, k: int = 7):
    """Median depth over a k×k window around (u, v); ignores non-finite and ≤0."""
    if depth_hw is None or depth_hw.ndim != 2:
        return None
    H, W = depth_hw.shape
    ui = int(np.clip(round(float(u)), 0, W - 1))
    vi = int(np.clip(round(float(v)), 0, H - 1))
    r = int(k) // 2
    u0, u1 = max(0, ui - r), min(W, ui + r + 1)
    v0, v1 = max(0, vi - r), min(H, vi + r + 1)
    patch = depth_hw[v0:v1, u0:u1].reshape(-1)
    patch = patch[np.isfinite(patch)]
    patch = patch[patch > 0.0]
    if patch.size == 0:
        return None
    return float(np.median(patch))


def deproject_pixel_to_cam(u: float, v: float, z: float, K) -> np.ndarray:
    """OpenCV pinhole deprojection. K is the 9-element row-major intrinsic matrix
    (as published by sensor_msgs/CameraInfo.k)."""
    fx, fy = float(K[0]), float(K[4])
    cx, cy = float(K[2]), float(K[5])
    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy
    return np.array([x, y, float(z)], dtype=np.float32)


def build_T_arm_cam(
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
) -> np.ndarray:
    """4×4 homogeneous transform from camera frame to arm base frame.

    Translation (x, y, z) is the camera origin expressed in arm coordinates.
    Rotation (roll, pitch, yaw) describes the camera orientation w.r.t. the
    arm base frame, applied in extrinsic xyz order, in **radians**.
    """
    R_arm_cam = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R_arm_cam
    T[:3, 3] = [float(x), float(y), float(z)]
    return T


class Reprojection3DNode(RealsenseSyncedRGBDNode):
    def __init__(self):
        super().__init__("reprojection_3D")

        # --- Frame + camera-mount parameters ----------------------------
        self.declare_parameter("arm_frame", "pro_arm_base_link")
        self.declare_parameter("camera_position_x", 0.0)
        self.declare_parameter("camera_position_y", 0.0)
        self.declare_parameter("camera_position_z", 0.0)
        self.declare_parameter("camera_orientation_roll", 0.0)
        self.declare_parameter("camera_orientation_pitch", 0.0)
        self.declare_parameter("camera_orientation_yaw", 0.0)

        # --- Reprojection settings --------------------------------------
        self.declare_parameter("depth_window", 7)
        self.declare_parameter("detections_topic", "/classifier/detections")
        self.declare_parameter("output_topic", "/reprojection_3D/targets")

        self.arm_frame = str(self.get_parameter("arm_frame").value)
        cam_x = float(self.get_parameter("camera_position_x").value)
        cam_y = float(self.get_parameter("camera_position_y").value)
        cam_z = float(self.get_parameter("camera_position_z").value)
        # Orientation params are declared in degrees for ergonomics; convert to
        # radians here before handing them to build_T_arm_cam.
        cam_roll_deg = float(self.get_parameter("camera_orientation_roll").value)
        cam_pitch_deg = float(self.get_parameter("camera_orientation_pitch").value)
        cam_yaw_deg = float(self.get_parameter("camera_orientation_yaw").value)
        cam_roll = math.radians(cam_roll_deg)
        cam_pitch = math.radians(cam_pitch_deg)
        cam_yaw = math.radians(cam_yaw_deg)

        self.depth_window = int(self.get_parameter("depth_window").value)
        detections_topic = str(self.get_parameter("detections_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.T_arm_cam = build_T_arm_cam(
            cam_x, cam_y, cam_z, cam_roll, cam_pitch, cam_yaw
        )

        self.get_logger().info(
            f"Camera in {self.arm_frame}: "
            f"pos=({cam_x:.3f}, {cam_y:.3f}, {cam_z:.3f}) m, "
            f"rpy=({cam_roll_deg:.1f}, {cam_pitch_deg:.1f}, {cam_yaw_deg:.1f}) deg"
        )

        # Latest synced RGBD frame stash (set in process())
        self.latest_depth = None
        self.latest_K = None

        self.det_sub = self.create_subscription(
            Detection2DArray, detections_topic, self._on_detections, 10
        )
        self.out_pub = self.create_publisher(Detection3DArray, output_topic, 10)

        self.get_logger().info(
            f"Subscribed to detections on '{detections_topic}', "
            f"publishing arm-frame targets on '{output_topic}'."
        )

    # ----- RGBD callback (just stash, no per-frame work) -----------------
    def process(self, color_bgr, depth_meters, camera_info):
        self.latest_depth = depth_meters
        self.latest_K = list(camera_info.k)

    # ----- Helpers -------------------------------------------------------
    def _transform_to_arm(self, p_cam: np.ndarray) -> np.ndarray:
        ph = np.ones(4, dtype=np.float32)
        ph[:3] = p_cam
        return (self.T_arm_cam @ ph)[:3]

    # ----- Detection callback (does the actual reprojection) -------------
    def _on_detections(self, msg: Detection2DArray) -> None:
        if self.latest_depth is None or self.latest_K is None:
            self.get_logger().warn(
                "Detections received but no RGBD frame seen yet — dropping."
            )
            return

        depth = self.latest_depth
        K = self.latest_K

        out = Detection3DArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.arm_frame

        for det2d in msg.detections:
            u = float(det2d.bbox.center.position.x)
            v = float(det2d.bbox.center.position.y)
            z = robust_depth_at(depth, u, v, k=self.depth_window)
            if z is None:
                continue

            p_cam = deproject_pixel_to_cam(u, v, z, K)
            p_arm = self._transform_to_arm(p_cam)

            det3d = Detection3D()
            det3d.header = out.header
            det3d.id = det2d.id
            det3d.bbox.center.position.x = float(p_arm[0])
            det3d.bbox.center.position.y = float(p_arm[1])
            det3d.bbox.center.position.z = float(p_arm[2])
            det3d.bbox.center.orientation.w = 1.0
            # bbox.size left at zero (3D extent of the object is not known here).

            for hyp_in in det2d.results:
                hyp_out = ObjectHypothesisWithPose()
                hyp_out.hypothesis.class_id = hyp_in.hypothesis.class_id
                hyp_out.hypothesis.score = hyp_in.hypothesis.score
                hyp_out.pose.pose.position.x = float(p_arm[0])
                hyp_out.pose.pose.position.y = float(p_arm[1])
                hyp_out.pose.pose.position.z = float(p_arm[2])
                hyp_out.pose.pose.orientation.w = 1.0
                det3d.results.append(hyp_out)

            out.detections.append(det3d)

        self.out_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Reprojection3DNode()
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
