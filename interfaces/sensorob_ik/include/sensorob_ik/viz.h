//
// Created by jakub on 29.12.2023.
//
#ifndef SENSOROB_IK_INTERFACE_VIZ_H
#define SENSOROB_IK_INTERFACE_VIZ_H


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include "sensorob_ik/logger.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

namespace viz {

    int visualizePoints(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        std::string& file_pos_name,
                        std::string& file_time_name);

    int load_points(std::string& file_pos_name,
                     std::vector<geometry_msgs::msg::Point>& pose_points);
    int load_missing_points(std::string& file_time_name,
                    std::vector<geometry_msgs::msg::Point>& pose_points,
                    std::vector<geometry_msgs::msg::Point>& pose_points_missing);
    int transform_points(std::vector<geometry_msgs::msg::Point>& pose_points,
                         double trans_x = 0.05,
                         double trans_y = 0.05,
                         double trans_z = 0.20);
    void display_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                            std::vector<geometry_msgs::msg::Point>& pose_points,
                            std::string& logText);
    void display_planes_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                               std::vector<geometry_msgs::msg::Point>& pose_points,
                               double bandwidth = 0.05,
                               double offset_x = 0.00,
                               double offset_y = 0.00,
                               double offset_z = 0.20);
}

#endif //SENSOROB_IK_INTERFACE_VIZ_H
