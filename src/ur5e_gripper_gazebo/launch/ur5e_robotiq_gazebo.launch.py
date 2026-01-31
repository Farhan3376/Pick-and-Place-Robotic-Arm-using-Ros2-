#!/usr/bin/env python3
"""
Launch file for UR5e with Robotiq 2F-85 gripper in Gazebo simulation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Get package directories
    pkg_ur5e_gripper = get_package_share_directory('ur5e_gripper_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robotiq_description = get_package_share_directory('robotiq_description')
    pkg_ur_description = get_package_share_directory('ur_description')
    
    # Launch configurations
    ur_type = LaunchConfiguration('ur_type')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    
    # Paths
    urdf_file = os.path.join(pkg_ur5e_gripper, 'urdf', 'ur5e_robotiq.urdf.xacro')
    controllers_file = os.path.join(pkg_ur5e_gripper, 'config', 'ur5e_robotiq_controllers.yaml')
    initial_positions_file = os.path.join(pkg_ur5e_gripper, 'config', 'initial_positions.yaml')
    world_file_path = os.path.join(pkg_ur5e_gripper, 'worlds', 'pick_place_world.sdf')
    
    # Set GZ_SIM_RESOURCE_PATH for mesh loading
    # This allows Gazebo to find package:// resources
    gz_resource_path = os.path.dirname(pkg_robotiq_description)
    ur_resource_path = os.path.dirname(pkg_ur_description)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f"{gz_resource_path}:{ur_resource_path}:{existing_gz_path}" if existing_gz_path else f"{gz_resource_path}:{ur_resource_path}"
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_path
    )
    
    # Generate robot description from xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        urdf_file, ' ',
        'ur_type:=', ur_type, ' ',
        'name:=ur', ' ',
        'prefix:=', ' ',
        'sim_ignition:=true', ' ',
        'simulation_controllers:=', controllers_file, ' ',
        'initial_positions_file:=', initial_positions_file,
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Gazebo Simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file_path}',
        }.items(),
    )
    
    # Spawn robot in Gazebo (on top of robot stand at z=0.8)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'ur5e_robotiq',
            '-x', '0',
            '-y', '0',
            '-z', '0.8',
            '-allow_renaming', 'true',
        ],
    )
    
    # ROS-Gazebo Bridge for clock
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Gripper Controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay controller spawning until joint_state_broadcaster is active
    delay_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    delay_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    
    nodes_to_start = [
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        gz_ros_bridge,
        joint_state_broadcaster_spawner,
        delay_arm_controller,
        delay_gripper_controller,
    ]
    
    return nodes_to_start


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Type of UR robot (ur3, ur5, ur10, ur3e, ur5e, ur10e, ur16e, ur20, ur30)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='pick_place_world.sdf',
            description='World file to load in Gazebo',
        ),
    ]
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
