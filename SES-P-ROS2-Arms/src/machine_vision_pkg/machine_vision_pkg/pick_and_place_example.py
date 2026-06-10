#!/usr/bin/env python3

"""
Example pick-and-place sequence on the real pro_arm, mirroring
pro_sim_examples/ex_move_object.cpp. Hard-coded box and target poses; this is
a reference example rather than a vision-driven node.
"""

import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from machine_vision_pkg.moveit_arm_base_node import MoveItArmBaseNode


def make_box(name: str, frame: str, dims, position) -> CollisionObject:
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(d) for d in dims]
    obj.primitives = [prim]
    pose = Pose()
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.w = 1.0
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    return obj


class PickAndPlaceExampleNode(MoveItArmBaseNode):
    def __init__(self):
        super().__init__("pick_and_place_example")

        self.declare_parameter("startup_wait", 2.0)
        self.startup_wait = float(self.get_parameter("startup_wait").value)

        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.cube_object = make_box(
            "object", self.BASE_LINK, [0.05, 0.05, 0.05], [0.3, 0.0, 0.3]
        )

    def wait_for_servers(self) -> None:
        super().wait_for_servers()
        if not self.scene_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/apply_planning_scene service is unavailable")

    def _apply_scene(self, scene: PlanningScene, action_label: str) -> bool:
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(f"{action_label} failed")
            return False
        return True

    def add_collision_objects(self) -> None:
        table_1 = make_box("table_1", self.BASE_LINK, [0.15, 0.25, 0.55], [0.35, 0.0, 0.0])
        table_2 = make_box("table_2", self.BASE_LINK, [0.25, 0.15, 0.49], [0.0, 0.35, 0.0])

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [table_1, table_2, self.cube_object]

        if self._apply_scene(scene, "Adding collision objects"):
            self.get_logger().info("Added collision objects: table_1, table_2, object")

    def attach_object(self) -> None:
        attached = AttachedCollisionObject()
        attached.link_name = self.EE_LINK
        attached.object = self.cube_object
        attached.object.operation = CollisionObject.ADD

        remove_from_world = CollisionObject()
        remove_from_world.id = self.cube_object.id
        remove_from_world.header.frame_id = self.BASE_LINK
        remove_from_world.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        scene.world.collision_objects = [remove_from_world]

        if self._apply_scene(scene, "Attaching the object"):
            self.get_logger().info("Attached the object")

    def detach_object(self) -> None:
        detach = AttachedCollisionObject()
        detach.link_name = self.EE_LINK
        detach.object.id = self.cube_object.id
        detach.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [detach]

        if self._apply_scene(scene, "Detaching the object"):
            self.get_logger().info("Detached the object")

    def run(self) -> None:
        self.wait_for_servers()
        self.add_collision_objects()
        time.sleep(self.startup_wait)

        self.get_logger().info("Moving to object")
        if not self.move_to_pose(0.3, 0.0, 0.325):
            return
        self.attach_object()

        self.get_logger().info("Moving to target position")
        if not self.move_to_pose(0.0, 0.3, 0.295):
            return
        self.detach_object()

        self.get_logger().info("Moving to default (home) pose")
        self.move_to_default()
        self.get_logger().info("Pick-and-place sequence finished")


def main():
    rclpy.init()
    node = PickAndPlaceExampleNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
