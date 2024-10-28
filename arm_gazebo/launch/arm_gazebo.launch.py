import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
 
def generate_launch_description():

    arm_world = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('arm_gazebo'), 'launch'),
         '/arm_world.launch.py'])
      )
    arm_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('arm_control'), 'launch'),
         '/arm_control.launch.py'])
      )

    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )

    nodes_to_start = [
        arm_world,
        arm_control,
        bridge_camera
    ]

    return LaunchDescription(nodes_to_start) 