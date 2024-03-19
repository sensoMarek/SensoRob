#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <algorithm>

#include "sensorob_planner/logger.h"

namespace file_logger {

int logTool(
	moveit::planning_interface::MoveGroupInterface& move_group, 
	const std::string planning_group,
	const moveit::planning_interface::MoveGroupInterface::Plan& plan,  
    std::string& dir_name,
    rclcpp::Logger& LOGGER
);

int logTrajectory(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    [[maybe_unused]] const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    std::string& dir_name,
    rclcpp::Logger& LOGGER
);

int logSummary(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    std::string& dir_name,
    rclcpp::Logger& LOGGER
);

void logAll(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string scene_dir_path,
    const std::string scene_dir_name,
    rclcpp::Logger& LOGGER
);

std::string get_current_time();
std::string create_new_dir(const std::string name, const std::string path, rclcpp::Logger& LOGGER);
std::fstream open_file(
    const std::string file_name, 
    std::string& dir_name,
    rclcpp::Logger& LOGGER
);

}