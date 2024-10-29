from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Create a node to manage joint state broadcasting
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Create a node for managing a position controller
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  
    )
    
    return LaunchDescription([
        joint_state_broadcaster, 
        position_controller
    ])