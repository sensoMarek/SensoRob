from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("sensorob").to_moveit_configs()

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument('num_rerun',
                              default_value='10',
                              description='[uint] Number of rerunning the whole planning process with logging for each scene')
    )

    ld.add_action(
        DeclareLaunchArgument('file_logging',
                              default_value='dont',
                              description='[dont, compact, full]\nLogging mode\n - dont:    no logging\n - compact: only logging the final result\n - full:    logging the whole planning process (including all reruns and all scenes)')
    )

    ld.add_action(
        DeclareLaunchArgument('planning',
                              default_value='non-constrained',
                              description='[dont, non-constrained, constrained]\nMode of the planner\n - dont:            no planning, only adding obstacles\n - non-constrained: planning non-constrained movements\n - constrained:     planning both constrained and non-constrained movements')
    )

    ld.add_action(
        DeclareLaunchArgument('planner_id',
                              default_value='RRTConnect',
                              description='[RRTConnect, RRTstar, RRT, TRRT, EST, LBTRRT, BiEST, STRIDE, BiTRRT, PRM, PRMstar, LazyPRMstar, FMT, PDST, STRIDE, BiEST, STRIDE, BiTRRT, STOMP, CHOMP, pilz-PTP, pilz-CIRC, pilz-LIN]\nID of the planner to be used for planning')
    )

    ld.add_action(
        Node(
            name="trajectory_logger",
            package="sensorob_trajectory_logger",
            executable="trajectory_logger",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
                {"num_rerun": LaunchConfiguration("num_rerun")},
                {"file_logging": LaunchConfiguration("file_logging")},
                {"planning": LaunchConfiguration("planning")},
                {"planner_id": LaunchConfiguration("planner_id")}
            ],
        )
    )

    return ld
