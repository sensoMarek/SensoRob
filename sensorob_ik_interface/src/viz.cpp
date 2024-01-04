//
// Created by jakub on 29.12.2023.
//
#include "sensorob_ik_interface/viz.h"

namespace viz {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("viz");

    int visualizePoints(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        std::string& file_pos_name) {

        // Create vector of points published on marker topic to visualize in RViZ
        std::vector<geometry_msgs::msg::Point> pose_points;
        bool success_code = load_points(file_pos_name, pose_points);

        // if reading from file is not successful
        if (success_code != 0) return -1;

        //transform point to base_link frame
        std::vector<geometry_msgs::msg::Point> pose_points_transformed;
        success_code = transform_points(pose_points, pose_points_transformed);

        // if transforming points is not successful
        if (success_code != 0) return -1;

        // publish all points
        display_all_points(visual_tools, pose_points);

        // publish only axes in selected bandwidth of each plane sequentially
        display_planes_points(visual_tools, pose_points, pose_points_transformed);



        return 0;
    }

    int load_points(std::string& file_pos_name,
                     std::vector<geometry_msgs::msg::Point>& pose_points) {
        // Open file for reading (with translation and orientation data of end effector)
        std::fstream file_pos(file_pos_name, std::ios::in);
        if (!file_pos.is_open()) {
            clog("File file_pos is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File file_pos opened", LOGGER);

        // Read lines from the file
        clog("Reading points", LOGGER);
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

        // Close file for reading
        file_pos.close();
        if (!file_pos.is_open()) {
            clog("File file_pos closed.", LOGGER);
        } else {
            clog("File file_pos not opened", LOGGER, ERROR);
            return -2;
        }

        return 0;
    }


    int transform_points(std::vector<geometry_msgs::msg::Point>& pose_points,
                         std::vector<geometry_msgs::msg::Point>& pose_points_transformed) {

        for (const auto &point : pose_points) {
            geometry_msgs::msg::Point point_transformed;

            point_transformed.x = point.x - 0.05;
            point_transformed.y = point.y - 0.05;
            point_transformed.z = point.z - 0.20;

            pose_points_transformed.push_back(point_transformed);
        }

        // redundant
        if (pose_points.size() != pose_points_transformed.size()) return -2;

        return 0;
    }

    void display_all_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                            std::vector<geometry_msgs::msg::Point>& pose_points) {
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to display all valid points.");
        clog("Publishing all points ", LOGGER);

        // Visualize IK points in RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
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
    }

    void display_planes_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                               std::vector<geometry_msgs::msg::Point>& pose_points,
                               std::vector<geometry_msgs::msg::Point>& pose_points_transformed) {
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to xy-plane points.");
        clog("Publishing xy-plane points ", LOGGER);
    }


}