//
// Created by jakub on 29.12.2023.
//

#include "sensorob_ik_interface/viz.h"

namespace viz {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("viz");

    int visualizePoints(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        std::string file_pos_name) {

        // Create vector of points published on marker topic to visualize in RViZ
        std::vector<geometry_msgs::msg::Point> pose_points;

        // Open file for reading (with translation and orientation data of end effector)
        std::fstream file_pos(file_pos_name, std::ios::in);
        if (!file_pos.is_open()) {
            clog("File file_pos is not successfully opened, exiting!", ERROR);
            return -1;
        }
        clog("File file_pos opened");

        // Read lines from the file
        clog("Reading points");
        std::string line;
        while (std::getline(file_pos, line)) {
            // Use a stringstream to parse doubles from the line
            std::istringstream iss(line);
            std::vector<double> lineDoubles;
            double num;

            while (iss >> num) {
                lineDoubles.push_back(num);

                // Check for a newline character
                if (iss.peek() == '\n')
                    break;
            }

            // Create pose from file in world frame
            geometry_msgs::msg::Point point;
            point.x = lineDoubles[0];
            point.y = lineDoubles[1];
            point.z = lineDoubles[2];

            pose_points.push_back(point);

        }
        clog("Publishing points on topic");

        // Visualize IK points in RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.points = pose_points;
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.color.a = 0.6;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        visual_tools.publishMarker(marker);
        visual_tools.trigger();

        // Close file for reading
        file_pos.close();
        if (!file_pos.is_open()) {
            clog("File file_pos closed.");
        } else {
            clog("File file_pos not opened", ERROR);
            return -2;
        }

        return 0;
    }


    void clog(const std::string& data, std::string log_level) {
        if (log_level == "WARN") {
            RCLCPP_WARN(LOGGER, "%s", data.c_str());
        } else if (log_level == "ERROR") {
            RCLCPP_ERROR(LOGGER,"%s", data.c_str());
        } else {
            RCLCPP_INFO(LOGGER, "%s", data.c_str());
        }
    }
}