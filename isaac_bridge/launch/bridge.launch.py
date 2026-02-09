from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_topic',
            default_value='/isaac/joint_commands',
            description='Topic to publish joint commands to.'
        ),
        Node(
            package='isaac_bridge',
            executable='bridge_node',
            name='isaac_bridge_node',
            output='screen',
            parameters=[{
                'output_topic': LaunchConfiguration('output_topic')
            }]
        )
    ])
