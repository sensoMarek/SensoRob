from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sensorob").to_moveit_configs()

    num_of_joint_samples_arg = LaunchConfiguration("num_of_joint_samples"),
    computeIK_arg = LaunchConfiguration("computeIK"),
    computeFK_arg = LaunchConfiguration("computeFK"),

    # MoveGroupInterface demo executable
    ik_interface_node = Node(
        name="ik_interface",
        package="sensorob_ik_interface",
        executable="ik_interface",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
            {"num_of_joint_samples": num_of_joint_samples_arg},
            {"computeFK": computeFK_arg},
            {"computeIK": computeIK_arg}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("num_of_joint_samples",
                              default_value='6',
                              description='Number of samples'),
        DeclareLaunchArgument("computeIK_arg",
                              default_value='True',
                              description='If False, inverse kinematics will be not computed'),
        DeclareLaunchArgument("computeFK_arg",
                              default_value='True',
                              description='If False, forward kinematics will be not computed'),
        ik_interface_node])


