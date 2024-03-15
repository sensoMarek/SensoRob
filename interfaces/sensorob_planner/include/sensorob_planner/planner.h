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
int create_environment(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects);

int create_scene_1(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects);

int create_scene_2(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects);


void addObjectsToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene, std::vector<moveit_msgs::msg::CollisionObject>& objects, std::vector<std::string>& object_ids);

#endif //SENSOROB_PLANNER_PLANNER_H
