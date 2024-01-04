//
// Created by jakub on 29.12.2023.
//
#ifndef SENSOROB_IK_INTERFACE_VIZ_H
#define SENSOROB_IK_INTERFACE_VIZ_H


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "sensorob_ik_interface/logger.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

namespace viz {

    int visualizePoints(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        std::string& file_pos_name);

    int load_points(std::string& file_pos_name,
                     std::vector<geometry_msgs::msg::Point>& pose_points);
    int transform_points(std::vector<geometry_msgs::msg::Point>& pose_points,
                         std::vector<geometry_msgs::msg::Point>& pose_points_transformed);
    void display_all_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                            std::vector<geometry_msgs::msg::Point>& pose_points);
    void display_planes_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                            std::vector<geometry_msgs::msg::Point>& pose_points,
                            std::vector<geometry_msgs::msg::Point>& pose_points_transformed);
}

#endif //SENSOROB_IK_INTERFACE_VIZ_H
