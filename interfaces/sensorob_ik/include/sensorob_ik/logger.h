//
// Created by jakub on 29.12.2023.
//
#ifndef SENSOROB_IK_INTERFACE_LOGGER_H
#define SENSOROB_IK_INTERFACE_LOGGER_H

#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#define ERROR "ERROR"
#define WARN "WARN"
#define INFO "INFO"

void clog(const std::string& data, const rclcpp::Logger& LOGGER, const std::string& log_level="INFO");

#endif //SENSOROB_IK_INTERFACE_LOGGER_H
