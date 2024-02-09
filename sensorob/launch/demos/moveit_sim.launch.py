from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Config
    package_name = 'sensorob'
    use_sim_time = 'True'
    rviz_config_file = get_package_share_directory('sensorob_moveit_config') + "/config/moveit_sim.rviz"

    # Launches
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'gazebo.launch.py'
        )])
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'moveit.launch.py'
        )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rviz.launch.py'
        )]), launch_arguments={'rviz_config_file': rviz_config_file,
                               'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        rsp,
        gazebo,
        moveit,
        rviz
    ])
