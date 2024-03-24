#include "sensorob_planner/launch_args_processor.h"

void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    uint& num_rerun,
    bool& allow_nc_planning,
    bool& allow_c_planning,
    bool& allow_file_logging)
{
    uint max_num_rerun = 100;

    // launch args
    num_rerun = move_group_node->get_parameter("num_rerun").get_value<uint>();
    allow_file_logging = move_group_node->get_parameter("file_logging").get_value<bool>();
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

    if (num_rerun > max_num_rerun && (!allow_nc_planning && !allow_c_planning)) { // is allowed planning
        num_rerun = max_num_rerun;
        allow_file_logging = false;
        clog("Invalid inputs (num_rerun > max_num_rerun), changed to:\n - num_rerun: " + std::to_string(num_rerun) + "\n - file_logging: false", LOGGER);
    }

    clog("Planning will be performed " + std::to_string(num_rerun) + "x", LOGGER);
    std::string s("File logging is ");
    s += allow_file_logging ? "allowed": "not allowed";
    clog(s, LOGGER);

}