//
// Created by jakub on 4.12.2023.
//

#ifndef SENSOROB_IK_INTERFACE_IK_H
#define SENSOROB_IK_INTERFACE_IK_H

#define ERROR "ERROR"
#define WARN "WARN"
#define INFO "INFO"

#include <moveit/move_group_interface/move_group_interface.h>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

namespace ik {
    int computeAndLogIK(const std::string& planning_group,
                        const moveit::core::JointModelGroup* joint_model_group,
                        const moveit::core::RobotStatePtr& cur_state,
                        int num_provided_samples,
                        std::string file_pos_name,
                        std::string file_time_name);
    void clog(const std::string&, std::string log_level = "INFO");

}

#endif //SENSOROB_IK_INTERFACE_IK_H
