//
// Created by jakub on 29.12.2023.
//
#include "sensorob_ik/viz.h"

namespace viz {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("viz");

    int visualizePoints(moveit_visual_tools::MoveItVisualTools& visual_tools,
                        std::string& file_pos_name,
                        std::string& file_time_name) {
        //computation path

        // Create vector of points published on marker topic to visualize in RViZ
        std::vector<geometry_msgs::msg::Point> pose_points;
        bool success_code = load_points(file_pos_name, pose_points);
        if (success_code != 0) return -1;

        // transform point to base_link frame
        std::vector<geometry_msgs::msg::Point> pose_points_transformed;
        success_code = transform_points(pose_points);
        if (success_code != 0) return -1;

        std::vector<geometry_msgs::msg::Point> pose_points_missing;
        bool success_code_missing_points = load_missing_points(file_time_name, pose_points, pose_points_missing);

        // display part
        // publish all points
        std::string logText = "valid";
        display_points(visual_tools, pose_points, logText);

        // publish missing points
        if (success_code_missing_points == 0) {
            logText =  "missing";
            display_points(visual_tools, pose_points_missing, logText);
        }

        // publish only axes in selected bandwidth of each plane sequentially
        display_planes_points(visual_tools, pose_points);

        return 0;
    }

    int load_points(std::string& file_pos_name,
                     std::vector<geometry_msgs::msg::Point>& pose_points) {

        // Open file for reading (with translation and orientation data of end effector)
        std::fstream file(file_pos_name, std::ios::in);
        if (!file.is_open()) {
            clog("File is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File opened", LOGGER);

        // Read lines from the file
        clog("Gathering all points", LOGGER);
        std::string line;
        while (std::getline(file, line)) {
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
        file.close();
        if (!file.is_open()) {
            clog("File closed.", LOGGER);
        } else {
            clog("File not closed", LOGGER, ERROR);
            return -2;
        }

        return 0;
    }

    int transform_points(std::vector<geometry_msgs::msg::Point>& pose_points,
                         double trans_x,
                         double trans_y,
                         double trans_z) {
        std::vector<geometry_msgs::msg::Point> pose_points_transformed;

        for (const auto &point : pose_points) {
            geometry_msgs::msg::Point point_transformed;

            point_transformed.x = point.x - trans_x;
            point_transformed.y = point.y - trans_y;
            point_transformed.z = point.z - trans_z;

            pose_points_transformed.push_back(point_transformed);
        }

        // redundant
        if (pose_points.size() != pose_points_transformed.size()) return -2;

        pose_points.clear();
        pose_points = pose_points_transformed;

        return 0;
    }

    int load_missing_points(std::string& file_time_name,
                            std::vector<geometry_msgs::msg::Point>& pose_points,
                            std::vector<geometry_msgs::msg::Point>& pose_points_missing) {

        std::fstream file(file_time_name, std::ios::in);
        if (!file.is_open()) {
            clog("File is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File opened", LOGGER);

        // Read lines from the file
        clog("Gathering missing points", LOGGER);
        std::string line;
        int counter = 0;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string numberAsString;
            double firstNumber;

            if (std::getline(iss, numberAsString, ' ')) { // blank space separated values
                try {
                    firstNumber = std::stod(numberAsString); // convert to double
                } catch (const std::invalid_argument& e) {
                    clog(e.what(), LOGGER, ERROR);
                    firstNumber = 0;

                } catch (const std::out_of_range& e) {
                    clog(e.what(), LOGGER, ERROR);
                    firstNumber = 0;
                }
                counter++;
            }

            if (firstNumber == -1.0) {
                /*clog("Missing point found!", LOGGER);*/
                pose_points_missing.push_back(pose_points[counter]);
            }
        }

        // Close file for reading
        file.close();
        if (!file.is_open()) {
            clog("File closed.", LOGGER);
        } else {
            clog("File not closed", LOGGER, ERROR);
            return -2;
        }

        if (pose_points_missing.empty()) {
            clog("All poses were solved by IK solver, no missing point found!", LOGGER);
            return -3;
        } /*else {
            clog("Found " + std::to_string(pose_points_missing.size()) + " missing points.", LOGGER);
        }*/

        return 0;
    }


    void display_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                            std::vector<geometry_msgs::msg::Point>& pose_points,
                            std::string& logText) {
//        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to display " + logText + " points.");
        clog("Publishing " + std::to_string(pose_points.size()) + " " +  logText + " points.", LOGGER);

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
    }

    void display_planes_points(moveit_visual_tools::MoveItVisualTools& visual_tools,
                               std::vector<geometry_msgs::msg::Point>& pose_points,
                               double bandwidth,
                               double offset_x,
                               double offset_y,
                               double offset_z) {

        std::vector<geometry_msgs::msg::Point> points_xy, points_yz, points_xz;

        for (const auto point : pose_points) {

            // xy
            if (point.z > (offset_z - bandwidth / 2.0) &&
                point.z < (offset_z + bandwidth / 2.0)) {
                points_xy.push_back(point);
            }

            // yz
            if (point.x > (offset_x - bandwidth / 2.0) &&
                point.x < (offset_x + bandwidth / 2.0)) {
                points_yz.push_back(point);
            }

            // xz
            if (point.y > (offset_y - bandwidth / 2.0) &&
                point.y < (offset_y + bandwidth / 2.0)) {
                points_xz.push_back(point);
            }
        }

        // Visualize IK points in RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.005;
        marker.scale.y = 0.005;
        marker.color.a = 0.6;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // xy
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to xy-plane points.");
        clog("Publishing xy-plane points ", LOGGER);
        marker.points = points_xy;
        visual_tools.publishMarker(marker);
        visual_tools.trigger();

        // yz
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to yz-plane points.");
        clog("Publishing yz-plane points ", LOGGER);
        marker.points = points_yz;
        visual_tools.publishMarker(marker);
        visual_tools.trigger();

        // xz
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui to xz-plane points.");
        clog("Publishing xz-plane points ", LOGGER);
        marker.points = points_xz;
        visual_tools.publishMarker(marker);
        visual_tools.trigger();

    }


}