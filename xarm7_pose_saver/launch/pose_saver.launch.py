from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xarm7_pose_saver',
            executable='pose_saver',
            name='pose_saver',
            output='screen'
        )
    ])
