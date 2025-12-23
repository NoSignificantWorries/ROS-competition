from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():    
    sensors_node = Node(
        package='autorace_core_comand13',
        executable='sensors',
        name='sensors_node',
        output='screen'
    )

    drive_node = Node(
        package='autorace_core_comand13',
        executable='drive',
        name='drive_node',
        output='screen'
    )

    return LaunchDescription([
        sensors_node,
        drive_node
    ])
