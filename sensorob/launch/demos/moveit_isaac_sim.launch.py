from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch MoveIt stack connected to Isaac Sim bridge."""

    # Config
    package_name = 'sensorob'
    sim_mode = 'True'
    rviz_config_file = get_package_share_directory('sensorob_moveit_config') + "/config/moveit_sim.rviz"

    # Launches
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'sim_mode': sim_mode}.items()
    )

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory(package_name), 'launch', 'gazebo.launch.py'
    #     )])
    # )
    controller_manager_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'controller_control_node.launch.py'
        )]), launch_arguments={'sim_mode': sim_mode}.items()
    )

    controller_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'controller_spawner.launch.py'
        )]), launch_arguments={'sim_mode': sim_mode}.items()
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'moveit.launch.py'
        )]), launch_arguments={'sim_mode': sim_mode}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rviz.launch.py'
        )]), launch_arguments={
            'rviz_config_file': rviz_config_file,
            'sim_mode': sim_mode
        }.items()
    )

    isaac_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('isaac_bridge'), 'launch', 'bridge.launch.py'
        )]), launch_arguments={'output_topic': '/isaac/joint_commands'}.items()
    )

    isaac_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('isaac_bridge'), 'launch', 'state_publisher.launch.py'
        )]), launch_arguments={
            'isaac_topic': '/isaac/joint_states',
            'output_topic': '/joint_states'
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'DEBUG'),
        rsp,
        # gazebo,
        controller_manager_node,
        controller_spawner,
        isaac_bridge,
        moveit,
        isaac_state_publisher,
        rviz
    ])
