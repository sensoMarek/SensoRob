from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sensorob").to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument('using_fake_robot',
                              default_value='True',
                              description='(boolean) [True, False]\nWhether the robot is real or simulated'))
    

    ld.add_action(
        Node(
            name="trajectory_logger",
            package="sensorob_trajectory_logger",
            executable="trajectory_logger",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": LaunchConfiguration("using_fake_robot")}
            ],
        )
    )

    return ld
