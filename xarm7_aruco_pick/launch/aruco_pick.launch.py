from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm7_aruco_pick',
            executable='aruco_detector_node',
            name='aruco_detector',
            output='screen',
            parameters=['config/aruco_params.yaml']
        )
    ])
