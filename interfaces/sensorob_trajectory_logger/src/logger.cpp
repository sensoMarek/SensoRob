//
// Created by jakub on 29.12.2023.
//
#include "sensorob_trajectory_logger/logger.h"

void clog(const std::string& data, const rclcpp::Logger& LOGGER, const std::string& log_level) {
    if (log_level == "WARN") {
        RCLCPP_WARN(LOGGER, "%s", data.c_str());
    } else if (log_level == "ERROR") {
        RCLCPP_ERROR(LOGGER,"%s", data.c_str());
    } else {
        RCLCPP_INFO(LOGGER, "%s", data.c_str());
    }
}
