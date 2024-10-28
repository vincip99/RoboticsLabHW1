# RoboticsLabHW1 #
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Building your robot manipulator

## Available Packages in this Repository ##
- `arm_description` - robot description and configuration files
- `arm_gazebo` - launch and run-time configurations
- `arm_control` - hardware interfaces for communication with the robot
- `arm_controller` - implementation of dedicated controllers

## Getting Started

```shell
cd ~/ros2_ws
git clone https://github.com/vincip99/RoboticsLabHW1.git
rosdep install --ignore-src --from-paths . -y -r
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
source install/setup.bash
```
## Usage
1. **URDF Model and Rviz Visualization**
```shell
ros2 launch arm_description display.launch.py
```

2. **Gazebo Simulation**
```shell
ros2 launch arm_gazebo arm_gazebo.launch.py
```

3. **Camera Sensor**
```shell
ros2 launch arm_gazebo arm_gazebo.launch
```
in another terminal
```shell
ros2 rqt_image_view rqt_image_view
```

4. **ROS custom node**
```shell
ros2 launch arm_gazebo arm_gazebo.launch
```
in another terminal
```shell
ros2 run arm_controller arm_controller_node
```