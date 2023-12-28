//
// Created by jakub on 4.12.2023.
//

#ifndef SENSOROB_IK_INTERFACE_FK_H
#define SENSOROB_IK_INTERFACE_FK_H

#define ERROR "ERROR"
#define WARN "WARN"
#define INFO "INFO"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/srv/get_state_validity.hpp>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>
namespace fk {
    int computeAndLogFK(const std::shared_ptr<rclcpp::Node>& move_group_node,
                        const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& planning_group,
                        int num_of_joint_samples,
                        std::string file_name);
    std::vector<double> interpolate(double start, double end, int n);
    void clog(const std::string&, std::string log_level = "INFO");

}
#endif //SENSOROB_IK_INTERFACE_FK_H
