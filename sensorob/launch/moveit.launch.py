import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


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

    sim_mode = LaunchConfiguration('sim_mode')
    moveit_config = MoveItConfigsBuilder("sensorob", package_name="sensorob_moveit_config").to_moveit_configs()

    # Process the URDF file
    robot_description_file = os.path.join(get_package_share_directory('sensorob_description'), 'urdf', "sensorob.urdf.xacro")
    robot_description = Command(['xacro ', robot_description_file, ' sim_mode:=', sim_mode])

    stomp_planning_yaml = load_yaml('sensorob_moveit_config', 'config/stomp_planning.yaml')
    ompl_planning_yaml = load_yaml('sensorob_moveit_config', 'config/ompl_planning.yaml')
    ompl_defaults_yaml = load_yaml('sensorob_moveit_config', 'config/ompl_defaults.yaml')
    chomp_planning_yaml = load_yaml('sensorob_moveit_config', 'config/chomp_planning.yaml')
    # print(f"Loaded stomp_planning_yaml: {stomp_planning_yaml}")
    # planning_pipeline_config = {
    #     # 'move_group': stomp_planning_yaml,
    #     'move_group': ompl_planning_yaml
    #     # 'move_group': chomp_planning_yaml
    # }
    planning_pipeline_config = ompl_planning_yaml

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

    planning_pipelines = {
        'pipelines': ["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
    }

    joint_limits = load_yaml('sensorob_moveit_config', 'config/joint_limits.yaml')

    move_group_params = [robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         planning_pipeline_config,
                         planning_pipelines,
                         moveit_config.trajectory_execution,
                         moveit_controllers,
                         planning_scene_monitor_parameters,
                         joint_limits,
                         {"use_sim_time": LaunchConfiguration('sim_mode')}]

    # default
    return Node(package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=move_group_params
                )


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_mode',
            default_value='False',
            description='Use sim time (perform sim_mode) if true'),
        moveit_launch()
    ])
