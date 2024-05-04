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
#include <cstdlib>

namespace file_logger {


int logTrajectory(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    [[maybe_unused]] const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string dir_name,
    const std::string file_name,
    rclcpp::Logger& LOGGER
);

void transformJointStatesToPose(
   const std::string home_dir_path,
   const std::string input_file_name,
   const std::string output_file_name,
   const std::string PLANNING_GROUP,
   const moveit::planning_interface::MoveGroupInterface& move_group
);

void computeError(
   const std::string home_dir_path,
   const std::string input_file_name1,
   const std::string input_file_name2
);

void visualizeTrajectory(
   const std::string home_dir_path,
   const std::string input_file_name1,
   const std::string input_file_name2
);


std::string get_current_time();
std::string create_new_dir(const std::string name, const std::string path, rclcpp::Logger& LOGGER);
std::fstream open_file(
    const std::string file_name, 
    const std::string dir_name,
    rclcpp::Logger& LOGGER
);
}