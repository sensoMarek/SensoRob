
#ifndef SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H
#define SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H

#include <moveit/move_group_interface/move_group_interface.h>
#include "sensorob_trajectory_logger/logger.h"



void process_launch_args(
    const std::shared_ptr<rclcpp::Node>& move_group_node, 
    const rclcpp::Logger& LOGGER,
    double& desired_frequency,
    std::string& planner_id,
    std::string& planning_pipeline_id,
    int& mode);

#endif //SENSOROB_PLANNER_LAUNCH_ARGS_PROCESSOR_H
