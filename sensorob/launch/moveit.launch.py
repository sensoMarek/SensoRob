import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro
from moveit_configs_utils import MoveItConfigsBuilder


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print(absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def moveit_launch():
    DeclareLaunchArgument('use_sim_time', default_value='True', description='Use sim time if true'),
    moveit_config = MoveItConfigsBuilder("sensorob", package_name="sensorob_moveit_config").to_moveit_configs()

    # Config
    robot_description_file = get_package_share_directory('sensorob_description') + "/urdf/sensorob.urdf.xacro"
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    planning_pipeline_config = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1}}

    controllers_yaml = load_yaml('sensorob', 'config/moveit_controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml,
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    planning_scene_monitor_parameters = {"publish_robot_description_semantic": True,
                                         "allow_trajectory_execution": True,
                                         "publish_planning_scene": True,
                                         "publish_geometry_updates": True,
                                         "publish_state_updates": True,
                                         "publish_transforms_updates": True,
                                         "monitor_dynamics": False}

    move_group_params = [robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         planning_pipeline_config,
                         moveit_config.trajectory_execution,
                         moveit_controllers,
                         planning_scene_monitor_parameters,
                         {"use_sim_time": LaunchConfiguration('use_sim_time')}]

    # default
    return Node(package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=move_group_params
                )


def generate_launch_description():

    return LaunchDescription([
        moveit_launch()
    ])
