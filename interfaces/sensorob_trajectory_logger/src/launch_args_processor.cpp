#include "sensorob_trajectory_logger/launch_args_processor.h"

void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    uint& num_rerun,
    int& planning_mode,
    int& file_logging_mode,
    std::string& planner_id,
    std::string& planning_pipeline_id)
{
    uint max_num_rerun = 1000;

    // planning mode
    std::string planning_mode_arg = move_group_node->get_parameter("planning").get_value<std::string>();
    if (!planning_mode_arg.compare("dont")) {
        planning_mode = PlanningMode::NO_PLANNING;
        clog("Mode of the planner - no planning, only adding obstacles", LOGGER);
    } else if (!planning_mode_arg.compare("constrained")) {
        planning_mode = PlanningMode::C_PLANNING;
        clog("Mode of the planner - planning both constrained and non-constrained movements", LOGGER);
    } else if (!planning_mode_arg.compare("non-constrained")) {
        planning_mode = PlanningMode::NC_PLANNING;
        clog("Mode of the planner - planning non-constrained movements", LOGGER);
    } else {
        planning_mode = PlanningMode::NC_PLANNING;
        clog("Invalid argument, using value planning:=non-constrained", LOGGER, WARN);
        clog("Mode of the planner - planning non-constrained movements", LOGGER);
    }

    // num_rerun
    num_rerun = move_group_node->get_parameter("num_rerun").get_value<uint>();
    if (((num_rerun < 0) || (num_rerun > max_num_rerun)) && (planning_mode!=PlanningMode::NO_PLANNING)) { // is allowed planning
        num_rerun = max_num_rerun;
        file_logging_mode = FileLogging::NO_LOGGING;
        clog("Invalid inputs (num_rerun > max_num_rerun), changed to:\n - num_rerun: " + std::to_string(num_rerun) + "\n - file_logging: false", LOGGER);
    }

    if (planning_mode!=PlanningMode::NO_PLANNING) clog("Planning will be performed " + std::to_string(num_rerun) + "x", LOGGER);


    //file logging
    std::string file_logging_mode_arg = move_group_node->get_parameter("file_logging").get_value<std::string>();
    
    std::string s = "";
    if (!file_logging_mode_arg.compare("dont")) {
        file_logging_mode = FileLogging::NO_LOGGING;
        s+="No ";
    } else if (!file_logging_mode_arg.compare("compact") && planning_mode!=PlanningMode::NO_PLANNING) {
        file_logging_mode = FileLogging::COMPACT_LOGGING;
        s+="Compact ";
    } else if (!file_logging_mode_arg.compare("full") && planning_mode!=PlanningMode::NO_PLANNING) {
        file_logging_mode = FileLogging::FULL_LOGGING;
        s+="Full ";
    } else {
        file_logging_mode = FileLogging::NO_LOGGING;
        clog("Invalid argument, using value file_logging:=dont", LOGGER, WARN);
        s+="No ";
    }
    
    s += "file logging";
    clog(s, LOGGER);

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

}