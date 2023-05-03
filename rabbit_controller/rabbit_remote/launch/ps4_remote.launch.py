from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    node1 = Node(
        package='joy',
        name='joy_node',
        executable='joy_node'
    )


    node2 = Node(
        package='rabbit_remote',
        name = 'PS4_Node',
        executable= 'ps4_node',
    )

    ld.add_action(node1)
    ld.add_action(node2)

    return ld
