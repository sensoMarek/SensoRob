//
// Created by jakub on 29.12.2023.
//
#ifndef SENSOROB_IK_INTERFACE_VIZ_H
#define SENSOROB_IK_INTERFACE_VIZ_H


#include <moveit_visual_tools/moveit_visual_tools.h>

#include "sensorob_ik_interface/logger.h"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

namespace viz {
    int visualizePoints(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        std::string file_pos_name);
}

#endif //SENSOROB_IK_INTERFACE_VIZ_H
