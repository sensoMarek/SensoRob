//
// Created by jakub on 4.12.2023.
//

#ifndef SENSOROB_IK_INTERFACE_FK_H
#define SENSOROB_IK_INTERFACE_FK_H

#include <moveit/move_group_interface/move_group_interface.h>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>
namespace fk {
    int computeAndLogFK(const std::string& PLANNING_GROUP, const moveit::core::JointModelGroup* joint_model_group, const moveit::core::RobotStatePtr& cur_state);
    std::vector<double> interpolate(double start, double end, int n);

}
#endif //SENSOROB_IK_INTERFACE_FK_H
