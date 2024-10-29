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
   - **arm_description**: Contains the URDF model of the robotic arm.
   - **Launch file**: `display.launch.py` to load and visualize the robot in Rviz2.
   - **Collision Meshes**: Replaced with primitive shapes for efficient collision detection.
```shell
ros2 launch arm_description display.launch.py
```

2. **Gazebo Simulation**
   - **arm_gazebo**: Includes the launch file `arm_world.launch.py` to spawn the robot in a Gazebo world.
   - **Hardware interface and Controllers**: Configured joint interfaces and controllers for joint control.
   - **Control Configuration**: `arm_control.launch.py` and `arm_control.yaml` to manage joint position controllers.
```shell
ros2 launch arm_gazebo arm_gazebo.launch.py
```

3. **Camera Sensor**
   - Added a camera sensor to the robot's URDF.
   - Configured the camera plugin in Gazebo and verified image publishing with `rqt_image_view`
```shell
ros2 launch arm_gazebo arm_gazebo.launch.py
```
in another terminal
```shell
ros2 run rqt_image_view rqt_image_view
```

4. **ROS custom node**
   - **arm_controller**: A ROS2 C++ node to read joint states and send position commands.
   - **Functionality**: Publishes custom commands to actuate joints and prints joint states to the terminal.
```shell
ros2 launch arm_gazebo arm_gazebo.launch.py
```
in another terminal
```shell
ros2 run arm_controller arm_controller_node --ros-args -p joint_positions:="[ , , , ]"
```