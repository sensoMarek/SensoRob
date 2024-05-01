#include "sensorob_trajectory_logger/launch_args_processor.h"

void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    double& desired_frequency,
    std::string& planner_id,
    std::string& planning_pipeline_id)
{
  
    // planner_id
    std::string planner_id_ = move_group_node->get_parameter("planner_id").get_value<std::string>();
    std::vector<std::string> ompl_planner_ids = {"RRTConnect", "RRT", "RRTstar", "TRRT", "EST", "LBTRRT", "BiEST", "STRIDE", "BiTRRT", "PRM", "PRMstar", "LazyPRMstar", "PDST", "STRIDE", "BiEST", "STRIDE", "BiTRRT"};
    std::vector<std::string> stomp_planner_ids = {"STOMP"};
    std::vector<std::string> chomp_planner_ids = {"CHOMP"};
    std::vector<std::string> pilz_planner_ids = {"PTP", "CIRC", "LIN"};

    if (std::find(ompl_planner_ids.begin(), ompl_planner_ids.end(), planner_id_) != ompl_planner_ids.end()) {
        planner_id = planner_id_;
        planning_pipeline_id = "";  // "ompl"; does not work
    } else if (std::find(stomp_planner_ids.begin(), stomp_planner_ids.end(), planner_id_) != stomp_planner_ids.end()) {
        planner_id = planner_id_;
        planning_pipeline_id = "stomp";
    } else if (std::find(chomp_planner_ids.begin(), chomp_planner_ids.end(), planner_id_) != chomp_planner_ids.end()) {
        planner_id = planner_id_;
        planning_pipeline_id = "chomp";
    } else if (std::find(pilz_planner_ids.begin(), pilz_planner_ids.end(), planner_id_) != pilz_planner_ids.end()) {
        planner_id = planner_id_;
        planning_pipeline_id = "pilz";
    } else {
        planner_id = "RRTConnect";
        planning_pipeline_id = "ompl";
        clog("Cannot find planning configuration for group 'sensorob_group' using planner '" + planner_id_ + "'. Will use defaults instead.", LOGGER, WARN);
    }

    std::string pp_id = !planning_pipeline_id.compare("")?"ompl":planning_pipeline_id;
    clog("Planner id: " + planner_id + ", planning pipeline id: " + pp_id, LOGGER);


    desired_frequency = move_group_node->get_parameter("desired_frequency").get_value<double>();
    if (desired_frequency <= 0.0) {
        desired_frequency = 100.0;
        clog("Desired frequency must be greater than 0.0. Using default value: " + std::to_string(desired_frequency), LOGGER, WARN);
    }
    else { clog("Desired frequency: " + std::to_string(desired_frequency), LOGGER);}
}