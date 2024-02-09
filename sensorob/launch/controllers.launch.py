import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    package_name = 'sensorob'
    sim_mode = LaunchConfiguration('sim_mode')

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'sim_mode',
            default_value='False',
            description='Use sim time (perform sim_mode) if true'
        )
    )

    # if not sim mode, then gazebo is not spawned, so we need to launch ros2_control_node (otherwise gazebo launches it)
    if not sim_mode:
        # Process the URDF file
        robot_description_file = os.path.join(get_package_share_directory('sensorob_description'), 'urdf', "sensorob.urdf.xacro")
        robot_description = Command(['xacro ', robot_description_file, ' sim_mode:=', sim_mode])

        controller_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'sensorob_controllers.yaml')
        ld.add_action(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {'robot_description': robot_description},
                    {'use_sim_time': sim_mode},
                    controller_params_file
                ]
            )
        )

    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         name="position_controller_node",
    #         arguments=["position_controller", "-c", "/controller_manager"],
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         name="effort_controller_node",
    #         arguments=["effort_controller", "-c", "/controller_manager"],
    #     )
    # )


    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            name="joint_trajectory_controller_node",
            arguments=["sensorob_group_controller", "-c", "/controller_manager"],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            name="joint_state_broadcaster_node",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )
    )

    return ld
