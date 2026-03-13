#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import numpy as np


# ─────────────────────────────────────────────
#  POSITIONS — Add or edit positions here only
#  Format: 'name': [j1, j2, j3, j4, j5, j6]  (degrees)
# ─────────────────────────────────────────────
POSITIONS = {
    'home_pos':   [19.0, -105.0, -84.0, -27.0, 88.0, 0.0],

    # Add your positions below:
    # 'position_1':     [0.0, -90.0, -90.0, 0.0, 90.0, 0.0],
    # 'postposition_1': [0.0, -90.0, -120.0, 0.0, 90.0, 0.0],
}


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover_node')

        self.controller_name = 'fairino5_controller'
        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.group_name = 'fairino5_v6_group'  

        # Load positions from the dict above
        self.positions = {name: {'angles': angles} for name, angles in POSITIONS.items()}

        # Action client → sends trajectory to controller
        self._action_client = ActionClient(
            self, FollowJointTrajectory,
            f'/{self.controller_name}/follow_joint_trajectory'
        )
        self._action_client.wait_for_server()

        # Service client → requests motion plan from MoveIt
        self.planning_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        while not self.planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Planning service not available, waiting...')

        # Subscribe to joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, 10
        )

        # Wait for first joint state (max 5s)
        timeout = 5.0
        start_time = self.get_clock().now().to_msg().sec
        while self.current_joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().to_msg().sec - start_time > timeout:
                raise RuntimeError('No joint state received')

        self.velocity_scaling = 0.3
        self.acceleration_scaling = 0.3
        self.is_moving = False

    def _joint_state_callback(self, msg):
        self.current_joint_state = msg

    def move_to(self, position_name):
        """Plan and execute motion to a named position. Returns True on success."""
        if self.is_moving:
            self.get_logger().warn('Robot is already moving.')
            return False

        position = self.positions.get(position_name)
        if not position:
            self.get_logger().error(f'Unknown position: {position_name}')
            self.get_logger().info(f'Available positions: {list(self.positions.keys())}')
            return False

        target_angles = [np.deg2rad(a) for a in position['angles']]
        self.is_moving = True
        self.get_logger().info(f'Moving to {position_name}')

        for attempt in range(1, 4):
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = self.group_name
            request.motion_plan_request.num_planning_attempts = 20
            request.motion_plan_request.allowed_planning_time = 10.0
            request.motion_plan_request.max_velocity_scaling_factor = self.velocity_scaling
            request.motion_plan_request.max_acceleration_scaling_factor = self.acceleration_scaling

            current_state = RobotState()
            current_state.joint_state = self.current_joint_state
            request.motion_plan_request.start_state = current_state

            goal_constraints = Constraints()
            for joint_name, angle in zip(self.joint_names, target_angles):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = angle
                jc.tolerance_above = 0.02
                jc.tolerance_below = 0.02
                jc.weight = 1.0
                goal_constraints.joint_constraints.append(jc)
            request.motion_plan_request.goal_constraints.append(goal_constraints)

            future = self.planning_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None and future.result().motion_plan_response.error_code.val == 1:
                trajectory = future.result().motion_plan_response.trajectory.joint_trajectory
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory = trajectory
                send_future = self._action_client.send_goal_async(goal_msg)
                send_future.add_done_callback(lambda f: self._goal_response_callback(f, position_name))
                return True
            else:
                self.get_logger().error(f'Planning attempt {attempt}/3 failed for {position_name}')

        self.get_logger().error(f'Motion plan failed for {position_name}')
        self.is_moving = False
        return False

    def _goal_response_callback(self, future, position_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for {position_name}')
            self.is_moving = False
            return
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._result_callback(f, position_name)
        )

    def _result_callback(self, future, position_name):
        self.get_logger().info(f'Motion to {position_name} completed.')
        self.is_moving = False

    def wait_until_done(self):
        while self.is_moving and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)