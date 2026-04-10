import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('genz_lio')
    default_cfg = os.path.join(pkg_dir, 'config')
    default_rviz = os.path.join(pkg_dir, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path  = LaunchConfiguration('config_path')
    config_file  = LaunchConfiguration('config_file')
    rviz_use     = LaunchConfiguration('rviz')
    rviz_cfg     = LaunchConfiguration('rviz_cfg')

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('config_path',  default_value=default_cfg),
        DeclareLaunchArgument('config_file',  default_value='r_campus.yaml'),
        DeclareLaunchArgument('rviz',         default_value='true'),
        DeclareLaunchArgument('rviz_cfg',     default_value=default_rviz),

        Node(
            package='genz_lio',
            executable='genz_lio_node',
            name='genz_lio',
            parameters=[
                PathJoinSubstitution([config_path, config_file]),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(rviz_use),
        ),
    ])
    return ld
