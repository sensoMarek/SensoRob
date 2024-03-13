//
// Created by jakub on 08.01.2024.
//
#include "sensorob_planner/planner.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planner");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("planner", node_options);

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
    visual_tools.publishText(text_pose, "PLANNER_test", rvt::WHITE, rvt::MEDIUM);

    visual_tools.trigger();

    clog("Planning frame: " + move_group.getPlanningFrame(), LOGGER);
    clog("End effector link: " + move_group.getEndEffectorLink(), LOGGER);

    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setStartStateToCurrentState();
    move_group.setNumPlanningAttempts(3);
    move_group.setPlanningTime(3);

    double trans_x = 0.05;  // m
    double trans_y = 0.05;  // m
    double trans_z = 0.20;  // m
    double box_size = 0.20;  // m
    double box_center_x = 0.00 + trans_x;  // m
    double box_center_y = 0.40 + trans_y;  // m
    double box_center_z = 0.55 + trans_z;  // m
    std::vector<geometry_msgs::msg::Pose> waypoints;

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = "world";
    ocm.orientation.w = sqrt(2)/2;
    ocm.orientation.x = -sqrt(2)/2;
    ocm.orientation.y = 0;
    ocm.orientation.z = 0;
    ocm.absolute_x_axis_tolerance = 0.2;
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2; // zmensit
    ocm.weight = 0.1;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);

    move_group.setPathConstraints(test_constraints);

//    center of the box
    geometry_msgs::msg::Pose pose1;
    pose1.position.x =  box_center_x;
    pose1.position.y =  box_center_y;
    pose1.position.z =  box_center_z;
    pose1.orientation.x = -sqrt(2)/2;
    pose1.orientation.y = 0;
    pose1.orientation.z = 0;
    pose1.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose1);

/*    geometry_msgs::msg::Pose pose2;
    pose2.position.x = 0.047154;
    pose2.position.y = 0.0511252;
    pose2.position.z = 1.42645;
    pose2.orientation.x = 0;
    pose2.orientation.y = 0;
    pose2.orientation.z = 0;
    pose2.orientation.w = 1;*/

    //    top front right
    geometry_msgs::msg::Pose pose2;
    pose2.position.x =  box_center_x -  box_size/2.0;
    pose2.position.y =  box_center_y + box_size/2.0;
    pose2.position.z =  box_center_z + box_size/2.0;
    pose2.orientation.x = -sqrt(2)/2;
    pose2.orientation.y = 0;
    pose2.orientation.z = 0;
    pose2.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose2);

    //    top front left
    geometry_msgs::msg::Pose pose3;
    pose3.position.x =  box_center_x +  box_size/2.0;
    pose3.position.y =  box_center_y + box_size/2.0;
    pose3.position.z =  box_center_z + box_size/2.0;
    pose3.orientation.x = -sqrt(2)/2;
    pose3.orientation.y = 0;
    pose3.orientation.z = 0;
    pose3.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose3);

    //    top back left
    geometry_msgs::msg::Pose pose4;
    pose4.position.x =  box_center_x +  box_size/2.0;
    pose4.position.y =  box_center_y - box_size/2.0;
    pose4.position.z =  box_center_z + box_size/2.0;
    pose4.orientation.x = -sqrt(2)/2;
    pose4.orientation.y = 0;
    pose4.orientation.z = 0;
    pose4.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose4);

    //    top back right
    geometry_msgs::msg::Pose pose5;
    pose5.position.x =  box_center_x -  box_size/2.0;
    pose5.position.y =  box_center_y - box_size/2.0;
    pose5.position.z =  box_center_z + box_size/2.0;
    pose5.orientation.x = -sqrt(2)/2;
    pose5.orientation.y = 0;
    pose5.orientation.z = 0;
    pose5.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose5);

    //    top front right
    waypoints.push_back(pose2);

    // bottom

    //    bottom front right
    geometry_msgs::msg::Pose pose6;
    pose6.position.x =  box_center_x -  box_size/2.0;
    pose6.position.y =  box_center_y + box_size/2.0;
    pose6.position.z =  box_center_z - box_size/2.0;
    pose6.orientation.x = -sqrt(2)/2;
    pose6.orientation.y = 0;
    pose6.orientation.z = 0;
    pose6.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose6);

    //    bottom front left
    geometry_msgs::msg::Pose pose7;
    pose7.position.x =  box_center_x +  box_size/2.0;
    pose7.position.y =  box_center_y + box_size/2.0;
    pose7.position.z =  box_center_z - box_size/2.0;
    pose7.orientation.x = -sqrt(2)/2;
    pose7.orientation.y = 0;
    pose7.orientation.z = 0;
    pose7.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose7);

    //    bottom back left
    geometry_msgs::msg::Pose pose8;
    pose8.position.x =  box_center_x +  box_size/2.0;
    pose8.position.y =  box_center_y - box_size/2.0;
    pose8.position.z =  box_center_z - box_size/2.0;
    pose8.orientation.x = -sqrt(2)/2;
    pose8.orientation.y = 0;
    pose8.orientation.z = 0;
    pose8.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose8);

    //    bottom back right
    geometry_msgs::msg::Pose pose9;
    pose9.position.x =  box_center_x -  box_size/2.0;
    pose9.position.y =  box_center_y - box_size/2.0;
    pose9.position.z =  box_center_z - box_size/2.0;
    pose9.orientation.x = -sqrt(2)/2;
    pose9.orientation.y = 0;
    pose9.orientation.z = 0;
    pose9.orientation.w = sqrt(2)/2;
    waypoints.push_back(pose9);

    //    bottom front right
    waypoints.push_back(pose6);

/*    move_group.setPoseTarget(pose1);

    std::vector<double> js = {0, -0.18, -0.71, 0.90, -1.19, 1.89};
    move_group.setJointValueTarget(js);*/

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    clog("Visualizing plan 4 (Cartesian path) (" + std::to_string(fraction * 100.0) + " achieved)", LOGGER);
    move_group.execute(trajectory);

/*    moveit::core::MoveItErrorCode response = move_group.plan(my_plan);
    if (response == moveit::core::MoveItErrorCode::SUCCESS) {
        clog("Executing trajectory", LOGGER);
        move_group.execute(my_plan);
    } else {
        clog("Planning not successful, error: " + std::to_string(response.val), LOGGER);
    }*/

    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}
