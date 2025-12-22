from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():    
    move_robot = Node(
        package='autorace_core_comand13',
        executable='move_robot',      # узел Worker с /team/*
        name='worker_node',
        output='screen'
    )

    return LaunchDescription([
        move_robot
    ])
