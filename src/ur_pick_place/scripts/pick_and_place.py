#!/usr/bin/env python3
"""
Pick and Place Script for UR5e with Robotiq 2F-85 Gripper
Uses MoveIt2 for motion planning and trajectory execution
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from std_msgs.msg import Header

import math
import time


class PickAndPlaceNode(Node):
    """ROS2 Node for Pick and Place operations with UR5e and Robotiq gripper"""

    def __init__(self):
        super().__init__('pick_and_place_node')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # MoveGroup action client
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            'move_action',
            callback_group=self.callback_group
        )
        
        # Gripper action client
        self._gripper_client = ActionClient(
            self,
            GripperCommand,
            'robotiq_gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )
        
        # Planning parameters
        self.planning_group = "ur_manipulator"
        self.end_effector_link = "tool0"
        self.planning_frame = "base_link"
        
        # Gripper parameters
        self.gripper_open_position = 0.0  # Fully open
        self.gripper_closed_position = 0.8  # Closed for grasping
        
        # Predefined positions
        self.home_pose = self._create_pose(0.3, 0.0, 0.5, 0.0, math.pi, 0.0)
        
        # Object positions on table
        self.pick_positions = {
            'red_cube': self._create_pose(0.4, 0.15, 0.45, 0.0, math.pi, 0.0),
            'green_cube': self._create_pose(0.4, -0.15, 0.45, 0.0, math.pi, 0.0),
            'blue_cylinder': self._create_pose(0.6, 0.0, 0.45, 0.0, math.pi, 0.0),
            'yellow_sphere': self._create_pose(0.5, 0.3, 0.45, 0.0, math.pi, 0.0),
        }
        
        # Place target position
        self.place_position = self._create_pose(0.5, -0.4, 0.45, 0.0, math.pi, 0.0)
        
        # Pre-grasp offset (above the object)
        self.pre_grasp_offset = 0.1
        
        self.get_logger().info("Pick and Place Node initialized")

    def _create_pose(self, x, y, z, roll, pitch, yaw):
        """Create a Pose message from position and RPY orientation"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        
        # Convert RPY to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        pose.orientation = Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy
        )
        
        return pose

    def wait_for_servers(self, timeout=10.0):
        """Wait for action servers to be available"""
        self.get_logger().info("Waiting for MoveGroup action server...")
        if not self._move_group_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("MoveGroup action server not available!")
            return False
        self.get_logger().info("MoveGroup server available!")
        
        self.get_logger().info("Waiting for Gripper action server...")
        if not self._gripper_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().warn("Gripper action server not available - using simulated gripper")
        else:
            self.get_logger().info("Gripper server available!")
        
        return True

    def move_to_pose(self, target_pose: Pose, wait=True):
        """Move the robot to a target pose using MoveIt2"""
        self.get_logger().info(f"Moving to pose: x={target_pose.position.x:.3f}, "
                              f"y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
        
        # Create motion plan request
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.planning_group
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.3
        motion_plan_request.max_acceleration_scaling_factor = 0.3
        
        # Goal constraints
        goal_constraints = Constraints()
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.planning_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives.append(primitive)
        
        primitive_pose = Pose()
        primitive_pose.position = target_pose.position
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        
        goal_constraints.position_constraints.append(position_constraint)
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.planning_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        
        goal_constraints.orientation_constraints.append(orientation_constraint)
        
        motion_plan_request.goal_constraints.append(goal_constraints)
        
        # Create MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request = motion_plan_request
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True
        goal_msg.planning_options.replan_attempts = 3
        
        # Send goal
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        
        if wait:
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Motion planning goal rejected!")
                return False
            
            self.get_logger().info("Motion plan accepted, executing...")
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result().result
            
            if result.error_code.val == 1:  # SUCCESS
                self.get_logger().info("Motion executed successfully!")
                return True
            else:
                self.get_logger().error(f"Motion failed with error code: {result.error_code.val}")
                return False
        
        return True

    def control_gripper(self, position: float, max_effort: float = 50.0, wait=True):
        """Control the gripper position"""
        self.get_logger().info(f"Setting gripper position to: {position:.3f}")
        
        if not self._gripper_client.server_is_ready():
            self.get_logger().warn("Gripper server not ready, simulating gripper action")
            time.sleep(1.0)
            return True
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        
        if wait:
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Gripper goal rejected!")
                return False
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            self.get_logger().info("Gripper action completed!")
            return True
        
        return True

    def open_gripper(self):
        """Open the gripper"""
        self.get_logger().info("Opening gripper...")
        return self.control_gripper(self.gripper_open_position)

    def close_gripper(self):
        """Close the gripper to grasp an object"""
        self.get_logger().info("Closing gripper...")
        return self.control_gripper(self.gripper_closed_position)

    def pick_object(self, object_name: str):
        """Execute pick sequence for an object"""
        if object_name not in self.pick_positions:
            self.get_logger().error(f"Unknown object: {object_name}")
            return False
        
        pick_pose = self.pick_positions[object_name]
        
        self.get_logger().info(f"=== Picking up {object_name} ===")
        
        # Step 1: Open gripper
        self.open_gripper()
        
        # Step 2: Move to pre-grasp position (above the object)
        pre_grasp_pose = Pose()
        pre_grasp_pose.position.x = pick_pose.position.x
        pre_grasp_pose.position.y = pick_pose.position.y
        pre_grasp_pose.position.z = pick_pose.position.z + self.pre_grasp_offset
        pre_grasp_pose.orientation = pick_pose.orientation
        
        self.get_logger().info("Moving to pre-grasp position...")
        if not self.move_to_pose(pre_grasp_pose):
            return False
        
        # Step 3: Move down to grasp position
        self.get_logger().info("Moving to grasp position...")
        if not self.move_to_pose(pick_pose):
            return False
        
        # Step 4: Close gripper
        self.close_gripper()
        time.sleep(0.5)  # Wait for stable grasp
        
        # Step 5: Lift object
        lift_pose = Pose()
        lift_pose.position.x = pick_pose.position.x
        lift_pose.position.y = pick_pose.position.y
        lift_pose.position.z = pick_pose.position.z + self.pre_grasp_offset
        lift_pose.orientation = pick_pose.orientation
        
        self.get_logger().info("Lifting object...")
        if not self.move_to_pose(lift_pose):
            return False
        
        self.get_logger().info(f"Successfully picked up {object_name}!")
        return True

    def place_object(self):
        """Execute place sequence"""
        self.get_logger().info("=== Placing object ===")
        
        # Step 1: Move to pre-place position
        pre_place_pose = Pose()
        pre_place_pose.position.x = self.place_position.position.x
        pre_place_pose.position.y = self.place_position.position.y
        pre_place_pose.position.z = self.place_position.position.z + self.pre_grasp_offset
        pre_place_pose.orientation = self.place_position.orientation
        
        self.get_logger().info("Moving to pre-place position...")
        if not self.move_to_pose(pre_place_pose):
            return False
        
        # Step 2: Move down to place position
        self.get_logger().info("Moving to place position...")
        if not self.move_to_pose(self.place_position):
            return False
        
        # Step 3: Open gripper to release object
        self.open_gripper()
        time.sleep(0.5)
        
        # Step 4: Retreat
        self.get_logger().info("Retreating...")
        if not self.move_to_pose(pre_place_pose):
            return False
        
        self.get_logger().info("Object placed successfully!")
        return True

    def go_home(self):
        """Move to home position"""
        self.get_logger().info("Moving to home position...")
        return self.move_to_pose(self.home_pose)

    def execute_pick_and_place(self, object_name: str):
        """Execute complete pick and place sequence"""
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"Starting Pick and Place for: {object_name}")
        self.get_logger().info(f"{'='*50}\n")
        
        # Go home first
        if not self.go_home():
            self.get_logger().error("Failed to move to home position!")
            return False
        
        # Pick the object
        if not self.pick_object(object_name):
            self.get_logger().error(f"Failed to pick {object_name}!")
            return False
        
        # Place the object
        if not self.place_object():
            self.get_logger().error("Failed to place object!")
            return False
        
        # Return home
        if not self.go_home():
            self.get_logger().error("Failed to return home!")
            return False
        
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"Pick and Place for {object_name} COMPLETED!")
        self.get_logger().info(f"{'='*50}\n")
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = PickAndPlaceNode()
    
    try:
        # Wait for action servers
        if not node.wait_for_servers(timeout=30.0):
            node.get_logger().error("Required servers not available. Exiting.")
            return
        
        # Execute pick and place for red cube
        node.execute_pick_and_place('red_cube')
        
        # Can also pick other objects:
        # node.execute_pick_and_place('green_cube')
        # node.execute_pick_and_place('blue_cylinder')
        # node.execute_pick_and_place('yellow_sphere')
        
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
