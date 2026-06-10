#!/usr/bin/env python3

"""
Base ROS 2 node bundling the MoveIt 2 plumbing shared by every motion node in `machine_vision_pkg`.

Subclass `MoveItArmBaseNode`, declare any node-specific parameters in your own `__init__`, then
call the inherited helpers (`wait_for_servers`, `move_to_pose`, …) from your `run()` instead of
re-implementing the IK + `/move_action` dance.
"""

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.spatial.transform import Rotation

MOVEIT_SUCCESS = 1  # moveit_msgs/MoveItErrorCodes.SUCCESS


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    x, y, z, w = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w))


class MoveItArmBaseNode(Node):
    # Override in a subclass for a different robot / planning group.
    MOVE_GROUP = "pro_arm"
    BASE_LINK = "pro_arm_base_link"
    EE_LINK = "pro_arm_ee"
    JOINT_NAMES = [f"pro_arm_joint_{i}" for i in range(1, 7)]

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter("velocity_scaling", 0.2)
        self.declare_parameter("acceleration_scaling", 0.2)
        self.declare_parameter("planning_time", 10.0)

        self.vel_scale = float(self.get_parameter("velocity_scaling").value)
        self.acc_scale = float(self.get_parameter("acceleration_scaling").value)
        self.plan_time = float(self.get_parameter("planning_time").value)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        # Static workshop table — pushed into the planning scene once via
        # add_table_obstacle(). Once present, /compute_ik (avoid_collisions=True)
        # and OMPL refuse any pose / trajectory that intersects it.
        self.declare_parameter("table_enabled", True)
        self.declare_parameter("table_size_x", 0.508) # Table's dimension in x-direction is 20 in.
        self.declare_parameter("table_size_y", 1.8288) # Table's dimension in y-direction is 6 ft.
        self.declare_parameter("table_size_z", 0.7874) # Table's top surface height from the ground is 31 in.
        self.declare_parameter("table_center_x", -0.002) # Arm's origin is 2 mm in +x-direction w.r.t. table's center at top surface.
        self.declare_parameter("table_center_y", 0.1016) # Arm's origin is 4 in. in -y-direction w.r.t. table's center at top surface.
        self.declare_parameter("table_center_z", -0.4037) # Arm's origin is 10 mm above (in +z-direction) w.r.t. table's center at top surface.
        self.declare_parameter("table_name", "table")

        self._table_enabled = bool(self.get_parameter("table_enabled").value)
        self._table_size = (
            float(self.get_parameter("table_size_x").value),
            float(self.get_parameter("table_size_y").value),
            float(self.get_parameter("table_size_z").value),
        )
        self._table_center = (
            float(self.get_parameter("table_center_x").value),
            float(self.get_parameter("table_center_y").value),
            float(self.get_parameter("table_center_z").value),
        )
        self._table_name = str(self.get_parameter("table_name").value)

        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def wait_for_servers(self) -> None:
        if not self.ik_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/compute_ik service is unavailable")
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/move_action action server is unavailable")
        if not self.scene_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("/apply_planning_scene service is unavailable")

    def add_table_obstacle(self) -> bool:
        """Push the static workshop table into MoveIt's planning scene."""
        if not self._table_enabled:
            self.get_logger().info("Table obstacle disabled via parameter.")
            return True

        obj = CollisionObject()
        obj.header.frame_id = self.BASE_LINK
        obj.id = self._table_name

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = list(self._table_size)
        obj.primitives = [prim]

        pose = Pose()
        pose.position.x = self._table_center[0]
        pose.position.y = self._table_center[1]
        pose.position.z = self._table_center[2]
        pose.orientation.w = 1.0
        obj.primitive_poses = [pose]
        obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(
                f"Failed to add '{self._table_name}' to planning scene."
            )
            return False
        self.get_logger().info(
            f"Added '{self._table_name}' to planning scene: "
            f"size={self._table_size} m, center={self._table_center} m in {self.BASE_LINK}"
        )
        return True

    def solve_ik(self, pose_stamped: PoseStamped):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.MOVE_GROUP
        req.ik_request.ik_link_name = self.EE_LINK
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.avoid_collisions = True
        req.ik_request.timeout.sec = 2

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or result.error_code.val != MOVEIT_SUCCESS:
            code = "no response" if result is None else result.error_code.val
            self.get_logger().error(
                f"IK failed (code={code}). Pose may be unreachable or in collision."
            )
            return None
        return list(zip(result.solution.joint_state.name, result.solution.joint_state.position))

    def _build_joint_goal(self, joint_solution) -> Constraints:
        c = Constraints()
        for name, pos in joint_solution:
            if name not in self.JOINT_NAMES:
                continue
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def _send_move_goal(self, goal_constraints: Constraints) -> bool:
        req = MotionPlanRequest()
        req.group_name = self.MOVE_GROUP
        req.goal_constraints = [goal_constraints]
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        opts = PlanningOptions()
        opts.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = opts

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("Goal rejected by move_group")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val == MOVEIT_SUCCESS:
            return True
        self.get_logger().error(f"Execution failed (code={result.error_code.val})")
        return False

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
    ) -> bool:
        """Plan and execute a motion that takes the EE to (x, y, z) with the requested orientation."""
        target = PoseStamped()
        target.header.frame_id = self.BASE_LINK
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation = rpy_to_quaternion(roll, pitch, yaw)

        sol = self.solve_ik(target)
        if sol is None:
            return False
        return self._send_move_goal(self._build_joint_goal(sol))

    def move_to_default(self) -> bool:
        # The pro_arm SRDF defines group_state name="default" with every joint at 0.0
        # (pro_arm_moveit/srdf/pro_arm.xacro). That's what RViz's "Goal State → default"
        # dropdown selects; reproducing it here as a joint-space goal avoids having to
        # parse the SRDF at runtime.
        c = Constraints()
        for name in self.JOINT_NAMES:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = 0.0
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._send_move_goal(c)
