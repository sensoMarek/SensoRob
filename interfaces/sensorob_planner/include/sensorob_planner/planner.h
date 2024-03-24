//
// Created by jakub on 08.01.2024.
//
#ifndef SENSOROB_PLANNER_PLANNER_H
#define SENSOROB_PLANNER_PLANNER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Dense>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

#include "sensorob_planner/logger.h"
#include "sensorob_planner/file_logger.h"
#include "sensorob_planner/obstacles.h"
#include "sensorob_planner/launch_args_processor.h"

rclcpp::Logger LOGGER = rclcpp::get_logger("planner");

uint num_rerun;
bool allow_file_logging;
std::string planning_mode;
std::string planner_id, planning_pipeline_id;
bool allow_nc_planning, allow_c_planning;
std::vector<moveit_msgs::msg::CollisionObject> objects;
std::vector<std::string> object_ids;
std::vector<std::string> environment_object_ids;
int result;
static const std::string PLANNING_GROUP = "sensorob_group";

// joint value targets
std::vector<double> jointValueTarget1 = {0, 1.04585, -1.50948, 0, -0.98084, 0};
std::vector<double> jointValueTarget2 = {0, 0.47508, -1.25613, 0, -0.15657, 0};

void addObjectsToScene(
    moveit::planning_interface::PlanningSceneInterface& planning_scene, 
    std::vector<moveit_msgs::msg::CollisionObject>& objects, 
    std::vector<std::string>& object_ids
);

void plan_cycle(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    moveit::core::RobotStatePtr robot_state, 
    const std::string home_dir_path, 
    const std::string dir_name);

#endif //SENSOROB_PLANNER_PLANNER_H
