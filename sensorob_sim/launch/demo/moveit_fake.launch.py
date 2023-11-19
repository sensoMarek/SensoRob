from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_name = 'sensorob_moveit_config'

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'demo.launch.py'
        )])
    )

    return LaunchDescription([
        demo
    ])