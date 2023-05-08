from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    node1 = Node(
        package='ocp_robocon2023',
        name = 'nmpc_node',
        executable= 'nmpc_omni_v2',
    )

    node2 = Node(
        package='ocp_robocon2023',
        name = 'beizer_node',
        executable= 'beizer_path2',
    )

    ld.add_action(node2)
    ld.add_action(node1)

    return ld
