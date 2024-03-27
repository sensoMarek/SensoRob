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

struct trajectory_attributes
{
    double tool_distance = -1;
    double joint_distance = -1;
    double planning_time = -1;
    double trajectory_time = -1;
    int number_of_points = -1;
    std::string dir_name = "";
};


int logTool(
	moveit::planning_interface::MoveGroupInterface& move_group, 
	const std::string planning_group,
	const moveit::planning_interface::MoveGroupInterface::Plan& plan,  
    const std::string dir_name,
    rclcpp::Logger& LOGGER
);

int logTrajectory(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    [[maybe_unused]] const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string dir_name,
    rclcpp::Logger& LOGGER
);

trajectory_attributes logSummary(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string dir_name,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER
);

trajectory_attributes logAll(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string scene_dir_path,
    const std::string scene_dir_name,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER
);

std::string get_current_time();
std::string create_new_dir(const std::string name, const std::string path, rclcpp::Logger& LOGGER);
std::fstream open_file(
    const std::string file_name, 
    const std::string dir_name,
    rclcpp::Logger& LOGGER
);

std::string trajectory_attributes_to_string(const trajectory_attributes& attributes);
int log_struct(
    const std::vector<trajectory_attributes> traj_attributes_vector, 
    const  uint num_rerun,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER
);
}