import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    sim_mode = LaunchConfiguration('sim_mode')

    # Process the URDF file
    robot_description_file = os.path.join(get_package_share_directory('sensorob_description'), 'urdf', "sensorob.urdf.xacro")
    robot_description = Command(['xacro ', robot_description_file, ' sim_mode:=', sim_mode])

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description, 'use_sim_time': sim_mode}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.05", "0.05", "0.20", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='False',
            description='Use sim time (perform sim_mode) if true'),
        node_robot_state_publisher,
        static_tf
    ])
