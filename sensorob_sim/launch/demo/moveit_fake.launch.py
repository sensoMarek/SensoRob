from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Includes
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * ros2_control_node + controller spawners
    """
    
    moveit_config = MoveItConfigsBuilder("sensorob", package_name="sensorob_moveit_config").to_moveit_configs()
    ld = LaunchDescription()

    # Given the published joint states, publish tf for the robot links
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="50.0"))

    # Given the published joint states, publish tf for the robot links and the robot description
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {
                    "publish_frequency": LaunchConfiguration("publish_frequency"),
                },
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the config to see the state of the move_group node
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            respawn=False,
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=rviz_parameters,
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld
