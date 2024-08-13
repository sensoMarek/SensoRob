//
// Created by jakub on 27.11.2023.
//
#include "sensorob_ik/ik_interface.h"

rclcpp::Logger LOGGER = rclcpp::get_logger("ik_interface");

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

    // Get the value from parameters
    int num_of_joint_samples = move_group_node->get_parameter("num_of_joint_samples").get_value<int>();
    bool computeIK = move_group_node->get_parameter("computeIK").get_value<bool>();
    bool computeFK = move_group_node->get_parameter("computeFK").get_value<bool>();
    double solver_timeout = move_group_node->get_parameter("timeout").get_value<double>();

    if (!computeFK){
        num_of_joint_samples = -1; // in this state we dont know ho many samples are in the file
    }

    
    std::string current_dir_name(get_current_dir_name());
    const std::string main_dir_name = file_logger::create_new_dir("src/SensoRob/sensorob_logs", current_dir_name, LOGGER);
    std::string logs_folder_path = file_logger::create_new_dir("ik_log_"+file_logger::get_current_time(), main_dir_name, LOGGER);
    RCLCPP_INFO(LOGGER, "Logs will be saved in %s", logs_folder_path.c_str());

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

    if (num_of_joint_samples > 25) {
        clog("num_of_joint_samples was set to "
             + std::to_string(num_of_joint_samples)
             + ", which means "
             + std::to_string(std::pow(num_of_joint_samples,5))
             + " states, what is too large number of robot configurations to process. "
               "Will use value 6 num_of_joint_states.", LOGGER, WARN);
        num_of_joint_samples = 6;
    }

    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    std::string file_pos_name = logs_folder_path + "/position.csv";
    std::string file_joint_name = logs_folder_path + "/joint.csv";
    std::string file_time_name = logs_folder_path + "/accurancy_and_time.csv";

    // compute and log translation and orientation (FK) of the end effector for a joint values seed
    if(computeFK) {
        fk::computeAndLogFK(move_group_node, move_group, PLANNING_GROUP, num_of_joint_samples, file_pos_name, file_joint_name);
    }

    // compute and log IK accurance and duration
    if (computeIK) {
        ik::computeAndLogIK(move_group_node, move_group, PLANNING_GROUP, file_pos_name, file_time_name, solver_timeout);
    }

    // Visualize point in RViZ published on topic
    viz::visualizePoints(visual_tools, file_pos_name, file_time_name);


    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}
