//
// Created by jakub on 27.11.2023.
//

#ifndef SENSOROB_IK_INTERFACE_IK_INTERFACE_H
#define SENSOROB_IK_INTERFACE_IK_INTERFACE_H

#define ERROR "ERROR"
#define WARN "WARN"
#define INFO "INFO"

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream>

void clog(const std::string&, std::string log_level = "INFO");

#endif //SENSOROB_IK_INTERFACE_IK_INTERFACE_H
