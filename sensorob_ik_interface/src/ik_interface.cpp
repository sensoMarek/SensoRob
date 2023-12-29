//
// Created by jakub on 27.11.2023.
//
#include "sensorob_ik_interface/ik_interface.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik_interface");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("ik_interface", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "sensorob_group";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link",
                                                        "ik_valid_points",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_IK", rvt::WHITE, rvt::MEDIUM);

    visual_tools.trigger();

    clog("Planning frame: " + move_group.getPlanningFrame(), LOGGER);
    clog("End effector link: " + move_group.getEndEffectorLink(), LOGGER);
    
    
    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    int num_of_joint_samples = 4;
    std::string file_pos_name = "/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/ik/position.csv";
    std::string file_time_name = "/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/ik/accurancy_and_time.csv";

    // compute and log translation and orientation (FK) of the end effector for a joint values seed
    fk::computeAndLogFK(move_group_node, move_group, PLANNING_GROUP, num_of_joint_samples, file_pos_name);

    // compute and log IK accurance and duration
    ik::computeAndLogIK(move_group_node, move_group, PLANNING_GROUP, std::pow(num_of_joint_samples,5), file_pos_name, file_time_name);

    // Visualize point in RViZ published on topic
    viz::visualizePoints(visual_tools, file_pos_name);

    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}
