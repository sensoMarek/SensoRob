//
// Created by jakub on 08.01.2024.
//
#ifndef SENSOROB_TRAJECTORY_LOGGER_H
#define SENSOROB_TRAJECTORY_LOGGER_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Dense>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <iostream>

#include "sensorob_trajectory_logger/file_logger.h"
#include "sensorob_trajectory_logger/obstacles.h"


rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_logger");

enum MovementMode {
    JOINT_SPACE = 0,
    CARTESIAN_SPACE = 1
};

struct Waypoint {
std::string name = "";
std::vector<double> coordinates;
};
std::vector<Waypoint> waypoints;

moveit_msgs::msg::RobotTrajectory trajectory;
moveit::core::MoveItErrorCode success = moveit::core::MoveItErrorCode::FAILURE;
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
static const std::string PLANNING_GROUP = "sensorob_group";
std::vector<double> jointValueTarget1; 
std::vector<double> jointValueTarget2;
int movement_mode;
int desired_frequency;
std::string planner_id;
std::string planning_pipeline_id;
double max_velocity_scaling_factor = 0.1;
double max_acceleration_scaling_factor = 0.1;

std::vector<std::string> environment_object_ids;
std::vector<moveit_msgs::msg::CollisionObject> objects;

int loadConfigData();

void addObjectsToScene(
    moveit::planning_interface::PlanningSceneInterface& planning_scene, 
    std::vector<moveit_msgs::msg::CollisionObject>& objects, 
    std::vector<std::string>& object_ids
);

int moveRobotToStartState(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> jointValueTarget);

void createPathFromWaypoints(std::vector<geometry_msgs::msg::Pose>& wps, const Eigen::Affine3d &end_effector_state);

#endif //SENSOROB_TRAJECTORY_LOGGER_H
