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
                              default_value='False',
                              description='[bool] If true, logging to file will be performed')
    )

    ld.add_action(
        DeclareLaunchArgument('planning',
                              default_value='non-constrained',
                              description='[constrained, non-constrained, dont]\nMode of the planner\n - constrained:     planning both constrained and non-constrained movements\n - non-constrained: planning non-constrained movements\n - dont:            no planning, only adding obstacles')
    )

    ld.add_action(
        DeclareLaunchArgument('planner_id',
                              default_value='RRTConnect',
                              description='[RRTConnect, RRTstar, RRT, RRTStar, TRRT, EST, LBTRRT, BiEST, STRIDE, BiTRRT, PRM, PRMstar, LazyPRMstar, FMT, PDST, STRIDE, BiEST, STRIDE, BiTRRT, STOMP, CHOMP, pilz-PTP, pilz-CIRC, pilz-LIN]\nID of the planner to be used for planning')
    )

    ld.add_action(
        Node(
            name="planner",
            package="sensorob_planner",
            executable="planner",
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
