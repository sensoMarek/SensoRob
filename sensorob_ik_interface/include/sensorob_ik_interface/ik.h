//
// Created by jakub on 4.12.2023.
//

#ifndef SENSOROB_IK_INTERFACE_IK_H
#define SENSOROB_IK_INTERFACE_IK_H

#define ERROR "ERROR"
#define WARN "WARN"
#define INFO "INFO"

#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

namespace ik {
    int computeAndLogIK(const std::shared_ptr<rclcpp::Node>& move_group_node,
                        const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& planning_group,
                        int num_provided_samples,
                        std::string file_pos_name,
                        std::string file_time_name);
    void clog(const std::string&, std::string log_level = "INFO");

}

#endif //SENSOROB_IK_INTERFACE_IK_H
