import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file_name = 'arm.urdf'
    urdf = os.path.join(
        get_package_share_directory('arm_description'), 'urdf',
        urdf_file_name
    )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_description_links = {"robot_description": robot_desc}

    # RViz config
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", 
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "arm_description.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz."
        )
    )

    # Nodes
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description_links,
                    {"use_sim_time": True},
            ],
        remappings=[('/robot_description', '/robot_description')]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

    return LaunchDescription(declared_arguments +
        [joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])