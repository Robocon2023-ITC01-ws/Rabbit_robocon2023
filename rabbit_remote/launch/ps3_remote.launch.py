from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    node1 = Node(
        package='rabbit_remote',
        name='joy_node',
        executable='ps3_nrf'
    )


    node2 = Node(
        package='rabbit_remote',
        name = 'PS3_Node',
        executable= 'ps3_node',
    )

    ld.add_action(node1)
    ld.add_action(node2)

    return ld
