from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sensorob").to_moveit_configs()

    # MoveGroupInterface demo executable
    ik_interface_node = Node(
        name="planner",
        package="sensorob_planner",
        executable="planner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        ik_interface_node])


