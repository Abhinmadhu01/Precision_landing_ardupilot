import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_flight_stack',
            executable='navigation_server.py',
            name='navigation_server',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='autonomous_flight_stack',
            executable='landing_server.py',
            name='landing_server',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='autonomous_flight_stack',
            executable='mission_executive.py',
            name='mission_executive',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': False}]
        )
    ])
