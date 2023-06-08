from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    launch = LaunchDescription()

    node1 = Node(
        package='rabbit_control',
        name='nmpc_rabbit',
        executable='nmpc_omni',
    )

    node2 = Node(
        package='rabbit_control',
        name='beizer_node',
        executable='beizer_path'
    )

    launch.add_action(node2)
    launch.add_action(node1)

    return launch