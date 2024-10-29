from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Path to your controller configuration YAML file
    controller_config = os.path.join(
        get_package_share_directory("arm_control"),
        "config",
        "arm_control.yaml"
    )

    # Controller manager node, with parameters pointing to the YAML configuration
    # controller_manager = Node(
    #    package="controller_manager",
    #    executable="ros2_control_node",
    #    parameters=[controller_config]
    #)

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  
    )

    nodes_to_start = [
        #controller_manager,
        joint_state_broadcaster, 
        position_controller
    ]

    return LaunchDescription(nodes_to_start)