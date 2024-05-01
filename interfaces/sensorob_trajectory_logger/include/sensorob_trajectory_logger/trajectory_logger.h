//
// Created by jakub on 08.01.2024.
//
#ifndef SENSOROB_PLANNER_PLANNER_H
#define SENSOROB_PLANNER_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
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

#include "sensorob_trajectory_logger/logger.h"
#include "sensorob_trajectory_logger/file_logger.h"
#include "sensorob_trajectory_logger/obstacles.h"
#include "sensorob_trajectory_logger/launch_args_processor.h"


rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_logger");

double desired_frequency;
std::string planner_id;
std::string planning_pipeline_id;
std::vector<moveit_msgs::msg::CollisionObject> objects;
std::vector<std::string> object_ids;
std::vector<std::string> environment_object_ids;
int result;
static const std::string PLANNING_GROUP = "sensorob_group";

// joint value targets
std::vector<double> jointValueTarget1 = {1.7453, 0.9250, -0.1222, -0.3491,  0.5760, -1.2566};
std::vector<double> jointValueTarget2 = {0,      1.2915, -1.0123,       0, -0.7156,  1.4661};

std::vector<double> jointValueTargetA = {-0.5585, 1.0996, -1.2915, -0.7156, -0.9425, 0.5236};
std::vector<double> jointValueTargetB = {0.0, 0.0, -0.2, 0.0, 0.0, 0.2};

void addObjectsToScene(
    moveit::planning_interface::PlanningSceneInterface& planning_scene, 
    std::vector<moveit_msgs::msg::CollisionObject>& objects, 
    std::vector<std::string>& object_ids
);

void plan_cycle(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    moveit_visual_tools::MoveItVisualTools visual_tools, 
    const std::string home_dir_path, 
    const std::string dir_name);

#endif //SENSOROB_PLANNER_PLANNER_H
