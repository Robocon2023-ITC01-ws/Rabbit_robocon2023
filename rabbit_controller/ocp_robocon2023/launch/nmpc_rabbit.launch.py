from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    node = Node(
        package='ocp_robocon2023',
        name = 'nmpc_node',
        executable= 'nmpc_omni',
    )

    ld.add_action(node)

    return ld