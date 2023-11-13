from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'move_group': {'planning_plugin': 'ompl_interface/OMPLPlanner'}},
                {'robot_description': 'robot_description'},
            ],
            remappings=[
                # ('/joint_states', '/your_robot/joint_states'),
                # Add any other remappings you may need
            ],
        ),
    ])