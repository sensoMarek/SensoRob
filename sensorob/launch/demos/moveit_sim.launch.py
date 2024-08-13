from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
     * robot_state_publisher
     * gazebo
     * controllers
     * move_group
     * rviz
    """

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'gazebo.launch.py'
        )])
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

    return LaunchDescription([
        rsp,
        gazebo,
        controller_spawner,
        moveit,
        rviz
    ])
