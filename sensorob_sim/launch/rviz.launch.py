from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_rviz2_launch():


    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[],
        remappings=[],
    )


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('sensorob_moveit_config'))
    config_file = os.path.join(pkg_path, 'config', 'kinematics.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=config_file,
            description='Path to the custom RViz configuration file'
        ),
        generate_rviz2_launch(),
    ])
