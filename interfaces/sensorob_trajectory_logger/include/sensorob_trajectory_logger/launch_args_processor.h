
#ifndef SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H
#define SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H

#include <moveit/move_group_interface/move_group_interface.h>
#include "sensorob_trajectory_logger/logger.h"

enum PlanningMode {
    NO_PLANNING,
    NC_PLANNING,
    C_PLANNING
};

enum FileLogging {
    NO_LOGGING,
    COMPACT_LOGGING,
    FULL_LOGGING
};

void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    double& desired_frequency,
    std::string& planner_id,
    std::string& planning_pipeline_id);

#endif //SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H
// Path: src/SensoRob/interfaces/sensorob_trajectory_logger/include/sensorob_trajectory_logger/launch_args_processor.h
// Compare this snippet from src/SensoRob/interfaces/sensorob_trajectory_logger/src/planner.cpp:
//