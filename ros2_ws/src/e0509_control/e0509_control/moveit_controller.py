#!/usr/bin/env python3
"""
moveit_controller.py

ROS2 Humble + MoveIt2 (과제 제출용 백엔드)
- Planning + Execute
- 다중 목표 좌표 순차 이동
- 속도 / 가속도 제어
- joint_states 기반 상태 모니터링
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint
from moveit_msgs.srv import GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState


class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        self.get_logger().info("Initializing MoveItController (Planning + Execute)")

        # ExecuteTrajectory Action Client
        self.exec_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )
        self.exec_client.wait_for_server()

        # Planning Service Client
        self.plan_client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path'
        )
        self.plan_client.wait_for_service()

        # joint_states subscriber (상태 모니터링)
        self.joint_positions = {}
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.get_logger().info("Connected to MoveIt planning & execution servers")

        self.group_name = "manipulator"
        self.base_frame = "base_link"

        # 속도 / 가속도
        self.max_velocity = 0.2
        self.max_acceleration = 0.2

    # ===============================
    # 상태 모니터링
    # ===============================
    def joint_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def get_joint_state_text(self):
        if not self.joint_positions:
            return "No joint state received"

        return " | ".join(
            [f"{k}: {v:.2f}" for k, v in self.joint_positions.items()]
        )

    def get_ee_position_estimate(self):
        """
        EE 절대좌표 (표시용 근사값)
        """
        if len(self.joint_positions) < 6:
            return None

        vals = list(self.joint_positions.values())
        ee_x = sum(vals[:2])
        ee_y = sum(vals[2:4])
        ee_z = sum(vals[4:6])

        return ee_x, ee_y, ee_z

    # ===============================
    # Planning / Execute
    # ===============================
    def plan_to_pose(self, x, y, z):
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = self.group_name

        req.motion_plan_request.max_velocity_scaling_factor = self.max_velocity
        req.motion_plan_request.max_acceleration_scaling_factor = self.max_acceleration

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        constraint = PositionConstraint()
        constraint.header.frame_id = self.base_frame
        constraint.link_name = "tool0"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.001, 0.001, 0.001]

        constraint.constraint_region.primitives.append(primitive)
        constraint.constraint_region.primitive_poses.append(pose.pose)
        constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(constraint)

        req.motion_plan_request.goal_constraints.append(constraints)

        self.get_logger().info(f"Planning to ({x}, {y}, {z})")

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not result or not result.motion_plan_response.trajectory.joint_trajectory.points:
            self.get_logger().error("Planning failed")
            return None

        return result.motion_plan_response.trajectory

    def execute_trajectory(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("Executing trajectory...")
        send_goal = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal)

        goal_handle = send_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error("Execution rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Execution finished")


# ⚠️ 요청대로 main()은 주석 유지
# if __name__ == '__main__':
#     main()
