from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'isaac_topic',
            default_value='/isaac/joint_states',
            description='Topic to subscribe to for Isaac Sim joint states.'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/joint_states',
            description='Topic where bridge republishes filtered joint states.'
        ),
        Node(
            package='isaac_bridge',
            executable='state_publisher',
            name='isaac_state_publisher',
            output='screen',
            parameters=[{
                'isaac_topic': LaunchConfiguration('isaac_topic'),
                'output_topic': LaunchConfiguration('output_topic')
            }]
        )
    ])
