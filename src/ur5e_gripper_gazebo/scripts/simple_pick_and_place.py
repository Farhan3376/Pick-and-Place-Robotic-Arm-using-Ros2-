#!/usr/bin/env python3
"""
Pick and Place Demo for UR5e with Robotiq 2F-85 Gripper
Uses action clients to properly wait for trajectory completion (dynamic behavior)
Robot is mounted on a stand at z=0.8, work table in front
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PickAndPlace(Node):
    """Pick and place using action clients for proper trajectory completion waiting."""
    
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Action clients for arm and gripper
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
        
        # Joint names
        self.arm_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.gripper_joints = ['robotiq_85_left_knuckle_joint']
        
        # Predefined poses - adjusted for robot on stand
        # Robot base at (0,0,0.8), table surface at z=0.8
        self.poses = {
            'home': [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            'ready': [0.0, -1.2, 1.4, -1.77, -1.57, 0.0],
            'pre_pick': [0.5, -1.0, 1.3, -1.87, -1.57, 0.55],
            'pick': [0.5, -0.7, 1.5, -2.37, -1.57, 0.55],
            # Intermediate transport position (neutral, higher up)
            'transport': [0.0, -1.2, 1.2, -1.57, -1.57, 0.0],
            'pre_place': [-0.6, -1.0, 1.3, -1.87, -1.57, -0.55],
            'place': [-0.6, -0.7, 1.5, -2.37, -1.57, -0.55],
        }
        
        self.get_logger().info('Pick and Place Node initialized')
        
    def wait_for_servers(self, timeout_sec=5.0):
        """Wait for action servers to be available."""
        self.get_logger().info('Waiting for action servers...')
        if not self.arm_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Arm action server not available!')
            return False
        if not self.gripper_action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Gripper action server not available!')
            return False
        self.get_logger().info('Action servers ready!')
        return True
        
    def move_arm(self, positions, duration_sec=3.0):
        """Move arm and wait for completion (dynamic)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(self.arm_joints)
        point.accelerations = [0.0] * len(self.arm_joints)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points.append(point)
        
        self.get_logger().info(f'Moving arm to: {[f"{p:.2f}" for p in positions]}')
        
        # Send goal and wait for result
        future = self.arm_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected!')
            return False
            
        # Wait for the action to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 10.0)
        
        self.get_logger().info('Arm movement complete!')
        return True
        
    def move_gripper(self, position, duration_sec=2.0):
        """Move gripper and wait for completion (dynamic)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joints
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        goal.trajectory.points.append(point)
        
        action = "Closing" if position > 0.3 else "Opening"
        self.get_logger().info(f'{action} gripper to: {position:.2f}')
        
        # Send goal and wait for result
        future = self.gripper_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
            
        # Wait for the action to complete
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)
        
        self.get_logger().info(f'Gripper {action.lower()} complete!')
        return True
        
    def open_gripper(self):
        """Open gripper fully."""
        return self.move_gripper(0.0)
        
    def close_gripper(self):
        """Close gripper for grasping."""
        return self.move_gripper(0.5)
        
    def go_to_pose(self, pose_name, duration_sec=3.0):
        """Move to a named pose and wait for completion."""
        if pose_name in self.poses:
            self.get_logger().info(f'==> Moving to: {pose_name}')
            return self.move_arm(self.poses[pose_name], duration_sec)
        else:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False
            
    def run_pick_and_place(self):
        """Execute full pick and place sequence with dynamic waiting."""
        if not self.wait_for_servers():
            return False
            
        self.get_logger().info('='*50)
        self.get_logger().info('STARTING PICK AND PLACE SEQUENCE')
        self.get_logger().info('='*50)
        
        # 1. Home position
        self.get_logger().info('\n[1/11] Going to home position')
        self.go_to_pose('home', 2.0)
        
        # 2. Open gripper
        self.get_logger().info('\n[2/11] Opening gripper')
        self.open_gripper()
        
        # 3. Move to ready position
        self.get_logger().info('\n[3/11] Moving to ready position')
        self.go_to_pose('ready', 2.0)
        
        # 4. Move to pre-pick
        self.get_logger().info('\n[4/11] Moving to pre-pick position')
        self.go_to_pose('pre_pick', 3.0)
        
        # 5. Lower to pick position
        self.get_logger().info('\n[5/11] Lowering to pick position')
        self.go_to_pose('pick', 2.0)
        time.sleep(0.5)  # Settle before grasping
        
        # 6. Close gripper to grasp
        self.get_logger().info('\n[6/11] Closing gripper to grasp object')
        self.close_gripper()
        
        # 7. Lift object (VERY slow and gentle)
        self.get_logger().info('\n[7/12] Lifting object gently')
        self.go_to_pose('pre_pick', 4.0)  # Very slow lift
        time.sleep(1.5)  # Wait after lift to stabilize
        
        # 8. Move to neutral transport position (first step)
        self.get_logger().info('\n[8/12] Moving to transport position')
        self.go_to_pose('transport', 0.5)  # Move to neutral first
        time.sleep(1.0)  # Stabilize at neutral
        
        # 9. Move to pre-place (second step)
        self.get_logger().info('\n[9/12] Moving to pre-place position slowly')
        self.go_to_pose('pre_place', 0.5)  # Slower final approach
        time.sleep(1.5)  # Wait after transport to stabilize
        
        # 10. Lower to place
        self.get_logger().info('\n[10/12] Lowering to place position')
        self.go_to_pose('place', 0.5)
        time.sleep(0.5)  # Settle before releasing
        
        # 11. Open gripper to release
        self.get_logger().info('\n[11/12] Opening gripper to release object')
        self.open_gripper()
        time.sleep(2.0)  # Wait for cube to settle after release
        
        # 12. Return home
        self.get_logger().info('\n[12/12] Returning to home position')
        self.go_to_pose('home', 4.0)
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('PICK AND PLACE COMPLETE!')
        self.get_logger().info('='*50)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    
    try:
        node.run_pick_and_place()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
