#!/usr/bin/env python3

"""
Move the pro_arm end-effector to a point `offset_z` metres above a given (x, y, z) point.
Defaults to 2 cm above (offset_z = 0.02) so the EE hovers over the target rather than crashing into it.

Run:
ros2 run machine_vision_pkg move_to_a_point --ros-args -p x:=0.30 -p y:=0.00 -p z:=0.30
"""

import rclpy

from machine_vision_pkg.moveit_arm_base_node import MoveItArmBaseNode


class MoveToAPointNode(MoveItArmBaseNode):
    def __init__(self):
        super().__init__("move_to_a_point")

        self.declare_parameter("x", 0.3)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.3)
        self.declare_parameter("offset_z", 0.02)
        self.declare_parameter("roll", 0.0)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.offset_z = float(self.get_parameter("offset_z").value)
        self.roll = float(self.get_parameter("roll").value)
        self.pitch = float(self.get_parameter("pitch").value)
        self.yaw = float(self.get_parameter("yaw").value)

    def run(self) -> None:
        self.wait_for_servers()
        self.add_table_obstacle()

        target_z = self.z + self.offset_z
        self.get_logger().info(
            f"Target point: ({self.x:.3f}, {self.y:.3f}, {self.z:.3f}); "
            f"moving EE to ({self.x:.3f}, {self.y:.3f}, {target_z:.3f}) "
            f"[{self.offset_z * 100:.1f} cm above]"
        )

        if self.move_to_pose(self.x, self.y, target_z, self.roll, self.pitch, self.yaw):
            self.get_logger().info("Reached target.")


def main():
    rclpy.init()
    node = MoveToAPointNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
