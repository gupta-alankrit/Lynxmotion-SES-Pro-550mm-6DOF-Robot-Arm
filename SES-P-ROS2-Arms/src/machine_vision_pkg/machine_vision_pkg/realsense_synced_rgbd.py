#!/usr/bin/env python3

"""
Base ROS 2 node that subscribes to synchronized RGB + aligned depth + camera_info
from a RealSense camera (or any compatible publisher), converts the images to
OpenCV/NumPy arrays, and dispatches them to a `process` hook implemented by
subclasses. Not a runnable node on its own — meant to be subclassed.

Subclasses should:
  - call super().__init__(node_name) in their __init__
  - declare any additional parameters they need
  - override `process(color_bgr, depth_meters, camera_info)`
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class RealsenseSyncedRGBDNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "aligned_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.05)
        self.declare_parameter("depth_units", 0.001)

        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("aligned_depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        queue_size = int(self.get_parameter("sync_queue_size").value)
        slop = float(self.get_parameter("sync_slop_sec").value)
        self.depth_units = float(self.get_parameter("depth_units").value)

        self.bridge = CvBridge()

        self.color_sub = Subscriber(
            node=self, msg_type=Image, topic=color_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            node=self, msg_type=Image, topic=depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.info_sub = Subscriber(
            node=self,
            msg_type=CameraInfo,
            topic=info_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            fs=[self.color_sub, self.depth_sub, self.info_sub],
            queue_size=queue_size,
            slop=slop,
        )
        self.sync.registerCallback(self._synced_callback)

    def _synced_callback(
        self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo
    ) -> None:
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(img_msg=color_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(img_msg=depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        if color_bgr is None or depth_raw is None:
            return

        if depth_raw.dtype == np.uint16:
            depth_meters = depth_raw.astype(np.float32) * self.depth_units
        elif depth_raw.dtype == np.float32:
            depth_meters = depth_raw
        else:
            self.get_logger().warn(
                f"Unexpected depth dtype: {depth_raw.dtype}. Trying float conversion."
            )
            depth_meters = depth_raw.astype(np.float32)

        try:
            self.process(color_bgr, depth_meters, info_msg)
        except Exception as e:
            self.get_logger().error(f"process() raised: {e}")

    def process(
        self, color_bgr: np.ndarray, depth_meters: np.ndarray, camera_info: CameraInfo
    ) -> None:
        raise NotImplementedError("Subclasses must implement process().")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()
