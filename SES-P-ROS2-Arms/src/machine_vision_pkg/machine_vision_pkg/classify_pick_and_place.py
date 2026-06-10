#!/usr/bin/env python3

"""
Vision-driven pick-and-place loop on the real pro_arm.

Cycle:
  1. Move the arm to the home (default) joint pose.
  2. Enable two_stage_live_classifier (std_srvs/SetBool) and wait for one
     fresh Detection3DArray on /reprojection_3D/targets; then disable it.
  3. If no targets, the scene is empty — exit.
  4. Move to the first target's (x, y, z + target_offset_z) with the
     configured approach orientation; wait `wait_at_target_sec` seconds.
  5. Move to the user-supplied drop point; wait `wait_at_drop_sec` seconds.
  6. Go back to step 1.

The classifier is the heaviest node in the pipeline (YOLOv8-seg + ConvNeXt
Stage-2). It is enabled only while the arm is parked at home and we are
actively collecting a fresh detection batch; during all motion phases the
SetBool gate keeps `process()` returning immediately so no inference runs.
"""

import time

import rclpy
from std_srvs.srv import SetBool
from vision_msgs.msg import Detection3DArray

from machine_vision_pkg.moveit_arm_base_node import MoveItArmBaseNode


class ClassifyPickAndPlaceNode(MoveItArmBaseNode):
    def __init__(self):
        super().__init__("classify_pick_and_place")

        # Drop point in the arm base frame.
        self.declare_parameter("drop_x", 0.0)
        self.declare_parameter("drop_y", -0.4)
        self.declare_parameter("drop_z", 0.1)
        self.declare_parameter("drop_roll", 0.0)
        self.declare_parameter("drop_pitch", 0.0)
        self.declare_parameter("drop_yaw", 0.0)

        # Approach pose above each detected target.
        self.declare_parameter("target_offset_z", 0.05)
        self.declare_parameter("target_roll", 0.0)
        self.declare_parameter("target_pitch", 0.0)
        self.declare_parameter("target_yaw", 0.0)

        # Dwell + detection timing.
        self.declare_parameter("wait_at_target_sec", 5.0)
        self.declare_parameter("wait_at_drop_sec", 5.0)
        self.declare_parameter("settle_sec", 2.0)
        self.declare_parameter("detection_timeout_sec", 10.0)
        self.declare_parameter("empty_confirmations", 5)

        # Topics + service names.
        self.declare_parameter("targets_topic", "/reprojection_3D/targets")
        self.declare_parameter(
            "classifier_set_active_service",
            "/two_stage_live_classifier/set_active",
        )

        self.drop_x = float(self.get_parameter("drop_x").value)
        self.drop_y = float(self.get_parameter("drop_y").value)
        self.drop_z = float(self.get_parameter("drop_z").value)
        self.drop_roll = float(self.get_parameter("drop_roll").value)
        self.drop_pitch = float(self.get_parameter("drop_pitch").value)
        self.drop_yaw = float(self.get_parameter("drop_yaw").value)

        self.target_offset_z = float(self.get_parameter("target_offset_z").value)
        self.target_roll = float(self.get_parameter("target_roll").value)
        self.target_pitch = float(self.get_parameter("target_pitch").value)
        self.target_yaw = float(self.get_parameter("target_yaw").value)

        self.wait_at_target_sec = float(self.get_parameter("wait_at_target_sec").value)
        self.wait_at_drop_sec = float(self.get_parameter("wait_at_drop_sec").value)
        self.settle_sec = float(self.get_parameter("settle_sec").value)
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )
        self.empty_confirmations = int(self.get_parameter("empty_confirmations").value)

        targets_topic = str(self.get_parameter("targets_topic").value)
        classifier_service = str(
            self.get_parameter("classifier_set_active_service").value
        )

        self._latest_targets_msg = None
        self.create_subscription(
            Detection3DArray, targets_topic, self._on_targets, 10
        )
        self.classifier_client = self.create_client(SetBool, classifier_service)

    # -------- helpers --------------------------------------------------------
    def _on_targets(self, msg: Detection3DArray) -> None:
        self._latest_targets_msg = msg

    def wait_for_servers(self) -> None:
        super().wait_for_servers()
        if not self.classifier_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(
                f"Classifier service '{self.classifier_client.srv_name}' is unavailable"
            )

    def _set_classifier_active(self, active: bool) -> None:
        req = SetBool.Request()
        req.data = bool(active)
        future = self.classifier_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn(
                f"Could not set classifier active={active}; continuing anyway."
            )

    def _wait_for_fresh_targets(self):
        # Phase 1: let the tracker mature. With confirm_frames=10 cycles and
        # classify_every_n_frames=3 at ~30 fps, real objects need ~1 s to be
        # confirmed; we hold for `settle_sec` (default 2 s) for safety.
        self._latest_targets_msg = None
        t_start = time.time()
        while time.time() - t_start < self.settle_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        # Phase 2: return the first NON-empty publication. Tolerate up to
        # `empty_confirmations` consecutive empty publications (handles
        # occasional tracker misses) before declaring the scene clear.
        self._latest_targets_msg = None
        last_stamp = None
        empty_streak = 0
        while time.time() - t_start < self.detection_timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = self._latest_targets_msg
            if msg is None:
                continue
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == last_stamp:
                continue
            last_stamp = stamp
            if msg.detections:
                return list(msg.detections)
            empty_streak += 1
            if empty_streak >= self.empty_confirmations:
                return []
        self.get_logger().warn(
            f"No /reprojection_3D/targets message received within "
            f"{self.detection_timeout_sec:.1f} s; treating as empty."
        )
        return []

    # -------- main loop ------------------------------------------------------
    def run(self) -> None:
        self.wait_for_servers()
        self.add_table_obstacle()
        # The classifier starts in `active=True` by default. Silence it
        # immediately and only re-enable around each detection window.
        self._set_classifier_active(False)

        cycle = 0
        while rclpy.ok():
            cycle += 1
            self.get_logger().info(f"--- Cycle {cycle}: returning home ---")
            if not self.move_to_default():
                self.get_logger().error("Failed to reach home pose; aborting.")
                return

            self.get_logger().info("Enabling classifier; waiting for fresh targets.")
            self._set_classifier_active(True)
            targets = self._wait_for_fresh_targets()
            self._set_classifier_active(False)

            if not targets:
                self.get_logger().info("No targets detected. Loop complete.")
                return

            first = targets[0]
            x = float(first.bbox.center.position.x)
            y = float(first.bbox.center.position.y)
            z = float(first.bbox.center.position.z) + self.target_offset_z
            self.get_logger().info(
                f"{len(targets)} target(s); moving above first at "
                f"({x:.3f}, {y:.3f}, {z:.3f}) m."
            )
            if not self.move_to_pose(
                x, y, z, self.target_roll, self.target_pitch, self.target_yaw
            ):
                self.get_logger().error("Failed to reach target; aborting.")
                return
            time.sleep(self.wait_at_target_sec)

            self.get_logger().info(
                f"Moving to drop point "
                f"({self.drop_x:.3f}, {self.drop_y:.3f}, {self.drop_z:.3f}) m."
            )
            if not self.move_to_pose(
                self.drop_x, self.drop_y, self.drop_z,
                self.drop_roll, self.drop_pitch, self.drop_yaw,
            ):
                self.get_logger().error("Failed to reach drop point; aborting.")
                return
            time.sleep(self.wait_at_drop_sec)


def main():
    rclpy.init()
    node = ClassifyPickAndPlaceNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
