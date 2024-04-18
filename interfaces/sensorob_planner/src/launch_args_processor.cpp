#include "sensorob_planner/launch_args_processor.h"

void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    uint& num_rerun,
    bool& allow_nc_planning,
    bool& allow_c_planning,
    bool& allow_file_logging,
    std::string& planner_id,
    std::string& planning_pipeline_id)
{
    uint max_num_rerun = 1000;

    // planning mode
    std::string planning_mode = move_group_node->get_parameter("planning").get_value<std::string>();
    if (!planning_mode.compare("dont")) {
        allow_nc_planning = false;
        allow_c_planning = false;
        clog("Mode of the planner - no planning, only adding obstacles", LOGGER);
    } else if (!planning_mode.compare("constrained")) {
        allow_nc_planning = true;
        allow_c_planning = true;
        clog("Mode of the planner - planning both constrained and non-constrained movements", LOGGER);
    } else if (!planning_mode.compare("non-constrained")) {
        allow_nc_planning = true;
        allow_c_planning = false;
        clog("Mode of the planner - planning non-constrained movements", LOGGER);
    } else {
        allow_nc_planning = true;
        allow_c_planning = false;
        clog("Invalid argument, using value planning:=non-constrained", LOGGER, WARN);
        clog("Mode of the planner - planning non-constrained movements", LOGGER);
    }

    // num_rerun
    num_rerun = move_group_node->get_parameter("num_rerun").get_value<uint>();
    if (num_rerun > max_num_rerun && (!allow_nc_planning && !allow_c_planning)) { // is allowed planning
        num_rerun = max_num_rerun;
        allow_file_logging = false;
        clog("Invalid inputs (num_rerun > max_num_rerun), changed to:\n - num_rerun: " + std::to_string(num_rerun) + "\n - file_logging: false", LOGGER);
    }

    //file logging
    allow_file_logging = move_group_node->get_parameter("file_logging").get_value<bool>();
    clog("Planning will be performed " + std::to_string(num_rerun) + "x", LOGGER);
    std::string s("File logging is ");
    s += allow_file_logging ? "allowed": "not allowed";
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