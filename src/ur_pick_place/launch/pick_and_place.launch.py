# Pick and Place Launch File for UR5e with Robotiq 2F-85
# Launches MoveIt2 and the pick and place node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type of UR robot",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    
    # Include MoveIt launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "launch_rviz": "true",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )
    
    # Pick and Place node (delayed start to wait for MoveIt)
    pick_and_place_node = TimerAction(
        period=10.0,  # Wait 10 seconds for MoveIt to be ready
        actions=[
            Node(
                package="ur_pick_place",
                executable="pick_and_place.py",
                name="pick_and_place_node",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            )
        ],
    )
    
    return LaunchDescription(
        declared_arguments + [
            moveit_launch,
            pick_and_place_node,
        ]
    )
