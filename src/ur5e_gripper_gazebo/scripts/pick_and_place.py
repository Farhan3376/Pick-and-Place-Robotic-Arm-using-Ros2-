#!/usr/bin/env python3
"""
Pick and Place Demo for UR5e with Robotiq 2F-85 Gripper
Sends trajectory commands to arm and gripper controllers
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math


class PickAndPlaceNode(Node):
    """Node for executing pick and place operations with UR5e and Robotiq gripper."""
    
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Action clients for arm and gripper controllers
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # UR5e joint names
        self.arm_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Robotiq gripper joint name
        self.gripper_joints = ['robotiq_85_left_knuckle_joint']
        
        # Predefined poses (joint angles in radians)
        # Cube at (0.40, 0.20, 0.82), Place at (0.40, -0.25, 0.805)
        # Robot base at (0, 0, 0.8)
        self.poses = {
            'home': [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            'pre_pick': [0.46, -1.0, 1.2, -1.77, -1.57, 0.40],
            'pick': [0.46, -0.75, 1.4, -2.22, -1.57, 0.40],
            'pre_place': [-0.56, -1.0, 1.2, -1.77, -1.57, -0.40],
            'place': [-0.56, -0.75, 1.4, -2.22, -1.57, -0.40],
        }
        
        self.get_logger().info('Pick and Place Node initialized')
        
    def wait_for_servers(self, timeout_sec=10.0):
        """Wait for action servers to be available."""
        self.get_logger().info('Waiting for action servers...')
        
        if not self.arm_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Arm controller action server not available!')
            return False
            
        if not self.gripper_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Gripper controller action server not available!')
            return False
            
        self.get_logger().info('Action servers ready!')
        return True
        
    def send_arm_trajectory(self, positions, duration_sec=3.0):
        """Send a trajectory to the arm controller."""
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.arm_joints)
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'Sending arm trajectory to positions: {[f"{p:.2f}" for p in positions]}')
        
        future = self.arm_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm trajectory goal rejected!')
            return False
            
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Arm trajectory completed successfully!')
            return True
        else:
            self.get_logger().warning(f'Arm trajectory finished with error code: {result.result.error_code}')
            return True  # Continue anyway
            
    def send_gripper_command(self, position, duration_sec=1.0):
        """
        Send a command to the gripper.
        position: 0.0 = open, 0.8 = closed
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        action = "Closing" if position > 0.4 else "Opening"
        self.get_logger().info(f'{action} gripper to position: {position:.2f}')
        
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
            
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        self.get_logger().info(f'Gripper {action.lower()} complete!')
        return True
        
    def open_gripper(self):
        """Open the gripper fully."""
        return self.send_gripper_command(0.0)
        
    def close_gripper(self):
        """Close the gripper to grasp an object."""
        return self.send_gripper_command(0.5)  # Partial close for gripping
        
    def move_to_pose(self, pose_name, duration_sec=3.0):
        """Move arm to a predefined pose."""
        if pose_name not in self.poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False
            
        self.get_logger().info(f'Moving to pose: {pose_name}')
        return self.send_arm_trajectory(self.poses[pose_name], duration_sec)
        
    def execute_pick_and_place(self):
        """Execute the full pick and place sequence."""
        self.get_logger().info('='*50)
        self.get_logger().info('Starting Pick and Place Sequence')
        self.get_logger().info('='*50)
        
        # Step 1: Move to home position
        self.get_logger().info('\n--- Step 1: Moving to home position ---')
        if not self.move_to_pose('home', 4.0):
            return False
        time.sleep(0.5)
        
        # Step 2: Open gripper
        self.get_logger().info('\n--- Step 2: Opening gripper ---')
        if not self.open_gripper():
            return False
        time.sleep(0.5)
        
        # Step 3: Move to pre-pick position
        self.get_logger().info('\n--- Step 3: Moving to pre-pick position ---')
        if not self.move_to_pose('pre_pick', 3.0):
            return False
        time.sleep(0.5)
        
        # Step 4: Move to pick position (lower to object)
        self.get_logger().info('\n--- Step 4: Moving to pick position ---')
        if not self.move_to_pose('pick', 2.0):
            return False
        time.sleep(0.5)
        
        # Step 5: Close gripper to grasp object
        self.get_logger().info('\n--- Step 5: Closing gripper to grasp object ---')
        if not self.close_gripper():
            return False
        time.sleep(0.5)
        
        # Step 6: Lift object (back to pre-pick)
        self.get_logger().info('\n--- Step 6: Lifting object ---')
        if not self.move_to_pose('pre_pick', 2.0):
            return False
        time.sleep(0.5)
        
        # Step 7: Move to pre-place position
        self.get_logger().info('\n--- Step 7: Moving to pre-place position ---')
        if not self.move_to_pose('pre_place', 3.0):
            return False
        time.sleep(0.5)
        
        # Step 8: Move to place position (lower to target)
        self.get_logger().info('\n--- Step 8: Moving to place position ---')
        if not self.move_to_pose('place', 2.0):
            return False
        time.sleep(0.5)
        
        # Step 9: Open gripper to release object
        self.get_logger().info('\n--- Step 9: Opening gripper to release object ---')
        if not self.open_gripper():
            return False
        time.sleep(0.5)
        
        # Step 10: Move back to pre-place
        self.get_logger().info('\n--- Step 10: Retreating from place position ---')
        if not self.move_to_pose('pre_place', 2.0):
            return False
        time.sleep(0.5)
        
        # Step 11: Return to home position
        self.get_logger().info('\n--- Step 11: Returning to home position ---')
        if not self.move_to_pose('home', 3.0):
            return False
            
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Pick and Place Sequence Complete!')
        self.get_logger().info('='*50)
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = PickAndPlaceNode()
    
    try:
        # Wait for action servers
        if not node.wait_for_servers(timeout_sec=30.0):
            node.get_logger().error('Failed to connect to action servers. Is the simulation running?')
            return
            
        # Execute pick and place
        node.execute_pick_and_place()
        
    except KeyboardInterrupt:
        node.get_logger().info('Pick and place interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
