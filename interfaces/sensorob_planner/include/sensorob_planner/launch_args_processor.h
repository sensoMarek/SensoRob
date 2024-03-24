
#ifndef SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H
#define SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H

#include <moveit/move_group_interface/move_group_interface.h>
#include "sensorob_planner/logger.h"

void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    uint& num_rerun,
    bool& allow_nc_planning,
    bool& allow_c_planning,
    bool& allow_file_logging,
    std::string& planner_id,
    std::string& planning_pipeline_id);

#endif //SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H
// Path: src/SensoRob/interfaces/sensorob_planner/include/sensorob_planner/launch_args_processor.h
// Compare this snippet from src/SensoRob/interfaces/sensorob_planner/src/planner.cpp:
//