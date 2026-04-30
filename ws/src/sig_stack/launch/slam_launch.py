from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'base_to_laser_tf',
            # x y z roll pitch yaw parent_frame child_frame (meters)
            arguments = ['0.2', '0.0', '0.1', '0', '0', '0', 'base_link', 'laser'],
            output = 'screen'
        ),

        Node(
            package = 'slam_toolbox',
            executable = 'async_slam_toolbox_node',
            name = 'slam_toolbox',
            output = 'screen',
            parameters = [{
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'use_sim_time': False
            }]
        )
    ])