import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():
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

    # position_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="position_controller_node",
    #     arguments=["position_controller", "-c", "/controller_manager"],
    # )
    #
    # effort_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     name="effort_controller_node",
    #     arguments=["effort_controller", "-c", "/controller_manager"],
    # )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_trajectory_controller_node",
        arguments=["sensorob_group_controller", "-c", "/controller_manager"],
    )

    # Delayed controllers section
    delayed_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_trajectory_controller],
        )
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_joint_trajectory_controller
    ])
