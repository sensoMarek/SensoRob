import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    package_name = 'sensorob'
    sim_mode = LaunchConfiguration('sim_mode')

    robot_description_file = os.path.join(get_package_share_directory('sensorob_description'), 'urdf', "sensorob.urdf.xacro")
    robot_description = Command(['xacro ', robot_description_file, ' sim_mode:=', sim_mode])

    controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'sensorob_controllers.yaml')

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_params_file
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='False',
            description='Use sim time (perform sim_mode) if true'),
        ros2_control_node
    ])
