from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

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

    return LaunchDescription([
        joint_state_broadcaster,
        joint_trajectory_controller
    ])
