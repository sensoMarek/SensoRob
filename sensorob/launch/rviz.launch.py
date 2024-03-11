from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_rviz2_launch():
    # Launch optional arguments
    sim_mode = LaunchConfiguration('sim_mode')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    moveit_config = MoveItConfigsBuilder("sensorob", package_name="sensorob_moveit_config").to_moveit_configs()

    return Node(package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', rviz_config_file],
                parameters=[moveit_config.planning_pipelines,
                            moveit_config.robot_description_kinematics,
                            {"use_sim_time": sim_mode}]
                )


def generate_launch_description():
    # Defaults
    rviz_config_file_default = get_package_share_directory('sensorob_moveit_config') + "/config/moveit.rviz"

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='False',
            description='Use sim time (perform sim_mode) if true'),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file_default,
            description='Path to the custom RViz configuration file'
        ),

        generate_rviz2_launch()
    ])
