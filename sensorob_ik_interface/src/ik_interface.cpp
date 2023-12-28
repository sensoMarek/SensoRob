#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Dense>
#include <geometry_msgs/geometry_msgs/msg/pose_stamped.h>
#include <rclcpp/clock.hpp>


#include "sensorob_ik_interface/ik_interface.h"
#include "sensorob_ik_interface/fk.h"
#include "sensorob_ik_interface/ik.h"

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
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "rviz_visual_tools",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    clog("Planning frame: " + move_group.getPlanningFrame());
    clog("End effector link: " + move_group.getEndEffectorLink());
    
    
    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::vector<std::string> joint_names = move_group.getActiveJoints();
    moveit::core::RobotStatePtr cur_state = move_group.getCurrentState(10);
    int num_of_joint_samples = 5;
    std::string file_pos_name = "/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/ik/position.csv";
    std::string file_time_name = "/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/ik/accurancy_and_time.csv";

    // compute and log translation and orientation (FK) of the end effector for a joint values seed
    fk::computeAndLogFK(move_group_node, move_group, PLANNING_GROUP, num_of_joint_samples, file_pos_name);
    // compute and log IK accurance and duration
    ik::computeAndLogIK(move_group, PLANNING_GROUP, std::pow(num_of_joint_samples,5), file_pos_name, file_time_name);

    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}

void clog(const std::string& data, std::string log_level) {
    if (log_level == "WARN") {
        RCLCPP_WARN(LOGGER, "%s", data.c_str());
    } else if (log_level == "ERROR") {
        RCLCPP_ERROR(LOGGER,"%s", data.c_str());
    } else {
        RCLCPP_INFO(LOGGER, "%s", data.c_str());
    }
}



