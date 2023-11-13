import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'sensorob_sim'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'sensorob'],
                        output='screen')

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_node",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="position_controller_node",
        arguments=["position_controller", "-c", "/controller_manager"],
    )

    effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="effort_controller_node",
        arguments=["effort_controller", "-c", "/controller_manager"],
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_trajectory_controller_node",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        joint_state_broadcaster,

        # effort_controller,
        # position_controller,
        joint_trajectory_controller,

    ])