from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sensorob").to_moveit_configs()
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument("num_of_joint_samples",
                          default_value='6',
                          description='Number of joint samples.\n'
                                      'This number will represent how many total samples will be used for IK test '
                                      'given formula num_of_joint_samples^(num of used joints)')
    )

    ld.add_action(
        DeclareLaunchArgument("computeIK",
                              default_value='True',
                              description='If False, inverse kinematics will be not computed')
    )

    ld.add_action(
        DeclareLaunchArgument("computeFK",
                              default_value='True',
                              description='If False, forward kinematics will be not computed')
    )
    ld.add_action(
        DeclareLaunchArgument("logs_folder_path",
                              default_value='/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/ik',
                              description='The path to the log folder in your PC, '
                                          'where files will be stored when performing IK test')
    )

    ld.add_action(
        DeclareLaunchArgument('use_sim_time',
                              default_value='True',
                              description='Use sim time if true')
    )

    ld.add_action(
        DeclareLaunchArgument('timeout',
                              default_value='0.005',
                              description='Solver timeout [s]')
    )

    ik_interface_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
        {"num_of_joint_samples": LaunchConfiguration("num_of_joint_samples")},
        {"computeFK": LaunchConfiguration("computeFK")},
        {"computeIK": LaunchConfiguration("computeIK")},
        {"logs_folder_path": LaunchConfiguration("logs_folder_path")},
        {"timeout": LaunchConfiguration("timeout")}
    ]

    ld.add_action(
        Node(
            name="ik_interface",
            package="sensorob_ik",
            executable="ik_interface",
            output="screen",
            parameters=ik_interface_params,
        )
    )

    return ld
