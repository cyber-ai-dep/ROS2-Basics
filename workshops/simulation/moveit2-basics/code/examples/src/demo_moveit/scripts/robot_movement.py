#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from sensor_msgs.msg import JointState
import numpy as np


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover_node')

        # Name of the ROS2 controller managing the arm joints
        self.controller_name = 'demo_arm_controller'

        # Joint names must match exactly what's defined in the URDF/SRDF
        self.joint_names = ['joint1', 'joint2', 'joint3']

        # Action client to send trajectory goals to the controller
        self._action_client = ActionClient(self, FollowJointTrajectory, f'/{self.controller_name}/follow_joint_trajectory')
        self._action_client.wait_for_server()

        # Service client to request motion plans from MoveIt
        self.planning_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        while not self.planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Planning service not available, waiting...')

        # Stores the latest joint state received from the robot
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, 10
        )

        # Wait up to 5 seconds for the first joint state before proceeding
        timeout = 5.0
        start_time = self.get_clock().now().to_msg().sec
        while self.current_joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now().to_msg().sec - start_time > timeout:
                raise RuntimeError('No joint state received')

        # Scale down speed and acceleration for safer movement (0.0 - 1.0)
        self.velocity_scaling = 0.3
        self.acceleration_scaling = 0.3

        # Flag to prevent sending multiple goals simultaneously
        self.is_moving = False 

        # Predefined named positions — angles in degrees for all 3 joints [J1, J2, J3]
        self.positions = {
            'home_pos':       {'angles': [0.0,   -60.0, -63.0]},  # Safe starting position
            'position_1':     {'angles': [110.0, -60.0, -63.0]},  # Target position 1
            'postposition_1': {'angles': [110.0, -90.0, -63.0]},  # Retreat after position 1
            'position_2':     {'angles': [-110.0, -60.0, -63.0]},  # Target position 2
            'postposition_2': {'angles': [-110.0, -90.0, -63.0]},  # Retreat after position 2

        }

    def _joint_state_callback(self, msg):
        # Continuously update the current joint state from the robot
        self.current_joint_state = msg

    def move_to(self, position_name):
        """
        Request a motion plan and execute it to reach a named position.
        Returns True if the goal was sent successfully, False otherwise.
        """
        if self.is_moving:
            self.get_logger().warn('Robot is already moving.')
            return False

        position = self.positions.get(position_name)
        if not position:
            self.get_logger().error(f'Unknown position: {position_name}')
            return False

        # Convert target angles from degrees to radians (required by MoveIt)
        target_angles = [np.deg2rad(a) for a in position['angles']]
        self.is_moving = True
        self.get_logger().info(f'Moving to {position_name}')

        # Retry planning up to 3 times in case of transient failures
        for attempt in range(1, 4):
            request = GetMotionPlan.Request()

            # Must match the planning group name defined in the SRDF
            request.motion_plan_request.group_name = 'demo_arm'
            request.motion_plan_request.num_planning_attempts = 20
            request.motion_plan_request.allowed_planning_time = 10.0
            request.motion_plan_request.max_velocity_scaling_factor = self.velocity_scaling
            request.motion_plan_request.max_acceleration_scaling_factor = self.acceleration_scaling

            # Use the current robot state as the start of the planned trajectory
            current_state = RobotState()
            current_state.joint_state = self.current_joint_state
            request.motion_plan_request.start_state = current_state

            # Build joint goal constraints — each joint must reach its target angle
            goal_constraints = Constraints()
            for joint_name, angle in zip(self.joint_names, target_angles):
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = angle
                jc.tolerance_above = 0.02  # ~1.1° tolerance
                jc.tolerance_below = 0.02
                jc.weight = 1.0
                goal_constraints.joint_constraints.append(jc)
            request.motion_plan_request.goal_constraints.append(goal_constraints)

            # Send planning request and wait for result
            future = self.planning_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            # error_code.val == 1 means SUCCESS in MoveIt
            if future.result() is not None and future.result().motion_plan_response.error_code.val == 1:
                trajectory = future.result().motion_plan_response.trajectory.joint_trajectory
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory = trajectory

                # Send the planned trajectory to the controller for execution
                send_future = self._action_client.send_goal_async(goal_msg)
                send_future.add_done_callback(lambda f: self._goal_response_callback(f, position_name))
                return True
            else:
                self.get_logger().error(f'Planning attempt {attempt}/3 failed for {position_name}')

        self.get_logger().error(f'Motion plan failed for {position_name}')
        self.is_moving = False
        return False

    def _goal_response_callback(self, future, position_name):
        """Called when the controller accepts or rejects the trajectory goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for {position_name}')
            self.is_moving = False
            return
        # Goal accepted — wait for execution to finish
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._result_callback(f, position_name)
        )

    def _result_callback(self, future, position_name):
        """Called when the robot finishes executing the trajectory."""
        self.get_logger().info(f'Motion to {position_name} completed.')
        self.is_moving = False

    def wait_until_done(self):
        """Block the calling code until the current movement finishes."""
        while self.is_moving and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)