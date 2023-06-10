from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    launch_description = LaunchDescription()

    node1 = Node(
        package='rabbit_remote',
        name='ps4_remote',
        executable='ps4_node'
    )

    node2 = Node(
        package='joy',
        name='joy_controller',
        executable='joy_node'
    )

    launch_description.add_action(node1)
    launch_description.add_action(node2)

    return launch_description
