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
int mode;
moveit_msgs::msg::RobotTrajectory trajectory;
moveit::core::MoveItErrorCode success = moveit::core::MoveItErrorCode::FAILURE;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
static const std::string PLANNING_GROUP = "sensorob_group";

std::vector<double> jointValueTargetHome = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // home position
std::vector<double> jointValueTargetA = {0.76185090, 0.481911, -1.156589, 1.4889827, -0.765009,1.2892323};
std::vector<double> jointValueTargetB = {0.0, 0.0, -1.0, 0.0, 0.0, 1.0};

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
