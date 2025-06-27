from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    # Launch only your planner (not the full moveit+gazebo config)
    xarm_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_planner'),
                'launch',
                'xarm7_planner_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'hw_ns': hw_ns
        }.items(),
    )

    # # Launch RViz with bringup's RViz config
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', PathJoinSubstitution([
    #         FindPackageShare('xarm7_bringup'),
    #         'rviz',
    #         'xarm7_bringup_config.rviz'
    #     ])],
    #     output='screen'
    # )


    return LaunchDescription([
        DeclareLaunchArgument(
            'hw_ns',
            default_value='xarm',
            description='Hardware namespace for the robot'
        ),
        xarm_planner_launch
    ])
