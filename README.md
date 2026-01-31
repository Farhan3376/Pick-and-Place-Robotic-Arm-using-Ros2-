# UR5e Robot Arm with Robotiq Gripper - Gazebo Simulation

A ROS 2 Humble simulation of the Universal Robots UR5e robot arm with Robotiq 85 gripper, featuring a complete pick-and-place demonstration in Gazebo Fortress.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-orange)
![License](https://img.shields.io/badge/License-MIT-green)

## Features

- 🤖 **UR5e Robot Arm** - 6-DOF industrial robot arm simulation
- 🔧 **Robotiq 85 Gripper** - Two-finger adaptive gripper with mimic joints
- 📦 **Pick and Place Demo** - Automated cube manipulation sequence
- ⚙️ **ros2_control Integration** - Joint trajectory controllers for arm and gripper
- 🎮 **Action-based Control** - Dynamic trajectory execution with proper completion waiting

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress (Ignition Gazebo 6)
- gz_ros2_control

### Install Dependencies

```bash
# ROS 2 Humble (if not installed)
sudo apt install ros-humble-desktop

# Gazebo and ROS 2 control packages
sudo apt install ros-humble-ros-gz ros-humble-gz-ros2-control
sudo apt install ros-humble-ros2-controllers ros-humble-controller-manager
sudo apt install ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller
```

## Installation

```bash
# Create workspace
mkdir -p ~/Universal_robot_arm/src
cd ~/Universal_robot_arm

# Clone repository 
git clone https://github.com/Farhan3376/Pick-and-Place-Robotic-Arm-using-Ros2-.git


# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Usage

### Launch Simulation

```bash
# Terminal 1: Start Gazebo simulation with robot
ros2 launch ur5e_gripper_gazebo ur5e_robotiq_gazebo.launch.py
```

### Run Pick and Place Demo

```bash
# Terminal 2: Run automated pick and place sequence
python3 ~/Universal_robot_arm/src/ur5e_gripper_gazebo/scripts/simple_pick_and_place.py
```

The demo performs a 14-step sequence:
1. Move to home position
2. Open gripper
3. Move to ready position
4. Move to pre-pick position
5. Lower to pick position
6. Close gripper (grasp cube)
7. Lift object gently
8. Move to transport position
9. Move to pre-place position
10. Lower to place position
11. Open gripper (release cube)
12. Move straight up
13. Return to ready position
14. Return to home position

## Project Structure

```
src/ur5e_gripper_gazebo/
├── config/
│   └── ur5e_robotiq_controllers.yaml  # Controller configuration
├── launch/
│   └── ur5e_robotiq_gazebo.launch.py  # Main launch file
├── scripts/
│   ├── simple_pick_and_place.py       # Pick and place demo
│   └── pick_and_place.py              # Alternative demo script
├── urdf/
│   └── ur5e_robotiq.urdf.xacro        # Robot description
├── worlds/
│   └── pick_place_world.sdf           # Gazebo world with table and objects
└── meshes/                             # Robot mesh files
```

## Configuration

### Controller Configuration

The robot uses two joint trajectory controllers:
- **arm_controller**: Controls 6 arm joints
- **gripper_controller**: Controls gripper with mimic joints

### World Setup

- **Robot Stand**: Raises robot base to z=0.8m
- **Work Table**: Table surface at z=0.65m
- **Red Cube**: 4cm cube for manipulation
- **Place Platform**: Target location for cube

## Customization

### Adjust Pick/Place Positions

Edit the poses in `simple_pick_and_place.py`:

```python
self.poses = {
    'home': [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    'pre_pick': [0.5, -0.8, 1.0, -1.77, -1.57, 0.55],
    'pick': [0.5, -0.5, 1.3, -2.37, -1.57, 0.55],
    # ... etc
}
```

### Adjust Movement Speed

Change duration parameters in `go_to_pose()` calls:

```python
self.go_to_pose('pre_pick', 4.0)  # 4 seconds to reach position
```

### Adjust Gripper Force

Modify gripper closing position (0.0 = open, 0.8 = fully closed):

```python
def close_gripper(self):
    return self.move_gripper(0.5)  # Moderate grip
```

## License

MIT License - Feel free to use and modify for your projects.

## Acknowledgments

- [Universal Robots](https://www.universal-robots.com/) - UR5e robot design
- [Robotiq](https://robotiq.com/) - Gripper design
- [ROS 2](https://docs.ros.org/) - Robotics middleware
- [Gazebo](https://gazebosim.org/) - Physics simulation
