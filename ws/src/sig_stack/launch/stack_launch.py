from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    odom_remap = ('odom', '/ego_racecar/odom')

    return LaunchDescription([
        Node(
            package='sig_stack',
            executable='global_planner',
            name='global_planner',
        ),
        Node(
            package='sig_stack',
            executable='local_planner',
            name='local_planner',
            remappings=[odom_remap],
        ),
        Node(
            package='sig_stack',
            executable='control',
            name='control',
            remappings=[odom_remap],
        ),
    ])
