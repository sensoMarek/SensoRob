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
        DeclareLaunchArgument('planner_id',
                              default_value='RRTConnect',
                              description='(string) [RRTConnect, RRTstar, RRT, TRRT, EST, LBTRRT, BiEST, STRIDE, BiTRRT, PRM, PRMstar, LazyPRMstar, FMT, PDST, STRIDE, BiEST, STRIDE, BiTRRT, STOMP, CHOMP, pilz-PTP, pilz-CIRC, pilz-LIN]\nID of the planner to be used for planning')
    )

    ld.add_action(
        DeclareLaunchArgument('using_fake_robot',
                              default_value='True',
                              description='(boolean) [True, False]\nWhether the robot is real or simulated'))
    
    ld.add_action(
        DeclareLaunchArgument('desired_frequency',
                              default_value='100.0',
                              description='(double) The desired frequency of the trajectory logger node'))

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
                {"use_sim_time": LaunchConfiguration("using_fake_robot")},
                {"planner_id": LaunchConfiguration("planner_id")},
                {"desired_frequency": LaunchConfiguration("desired_frequency")}
            ],
        )
    )

    return ld
