from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sensorob").to_moveit_configs()

    num_of_joint_samples_arg = LaunchConfiguration("num_of_joint_samples"),
    create_states_arg = LaunchConfiguration("create_states"),
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
            {"create_states": create_states_arg}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("num_of_joint_samples",
                              default_value='6',
                              description='Number of samples'),
        DeclareLaunchArgument("create_states",
                              default_value='True',
                              description='If true, assuming already created file "position.csv"'),
        ik_interface_node])


