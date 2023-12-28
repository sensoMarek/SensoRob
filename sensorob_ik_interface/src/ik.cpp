//
// Created by jakub on 4.12.2023.
//

#include "sensorob_ik_interface/ik.h"


namespace ik {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik");

    int computeAndLogIK(const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& planning_group,
                        int num_total_samples,
                        std::string file_pos_name,
                        std::string file_time_name) {

        const moveit::core::JointModelGroup* joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(planning_group);
        moveit::core::RobotStatePtr cur_state = move_group.getCurrentState(10);

        kinematics::KinematicsQueryOptions o;
        o.return_approximate_solution = false;
        std::vector<double> joint_values_ik;
        int num_valid_samples = 0;
        int num_found_samples = 0;
        double timeout = 0.1; // t[s]


        // Open file for reading (with translation and orientation data of end effector)
        std::fstream file_pos(file_pos_name, std::ios::in);
        if (!file_pos.is_open()) {
            clog("File file_pos is not successfully opened, exiting!", ERROR);
            return -1;
        }
        clog("File file_pos opened");

        // Open file for writing (logging)
        std::fstream file_time;
        file_time.open(file_time_name, std::ios::out);
        if (!file_time.is_open()) {
            clog("File file_time is not successfully opened, exiting!", ERROR);
            return -1;
        }
        clog("File file_time opened");

        // Read lines from the file
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

            // Create pose read from file
            geometry_msgs::msg::PoseStamped pose_desired;
            pose_desired.pose.position.x = lineDoubles[0];
            pose_desired.pose.position.y = lineDoubles[1];
            pose_desired.pose.position.z = lineDoubles[2];
            pose_desired.header.frame_id = "base_link";
            pose_desired.header.stamp = rclcpp::Clock().now();

            // Measure the duration of IK computation
            rclcpp::Time start_time = rclcpp::Clock().now();
            bool ik_found = cur_state->setFromIK(joint_model_group, pose_desired.pose, timeout,
                                                 moveit::core::GroupStateValidityCallbackFn(), o);
            rclcpp::Time end_time = rclcpp::Clock().now();
            rclcpp::Duration duration = end_time - start_time;

            // If IK found, validate and log results
            if (ik_found) {
                num_found_samples++;
                cur_state->copyJointGroupPositions(joint_model_group, joint_values_ik);

                cur_state->setJointGroupPositions(planning_group, joint_values_ik);
                const Eigen::Affine3d &pose_found = cur_state->getGlobalLinkTransform("link_6");

                double accurancy = sqrt(pow(pose_desired.pose.position.x - pose_found.translation().x(), 2) +
                                        pow(pose_desired.pose.position.y - pose_found.translation().y(), 2) +
                                        pow(pose_desired.pose.position.z - pose_found.translation().z(), 2));
                file_time << accurancy << " " << duration.seconds() << std::endl;
            } else {
                /*clog("Did not find IK solution", ERROR);*/
                file_time << -1 << " " << -1 << std::endl;
            }

            // Counter
            num_valid_samples++;
            if (num_valid_samples%100==0) clog("Processed " + std::to_string(num_valid_samples) + " samples.");
        }

        clog("Processed " + std::to_string(num_valid_samples) + " samples.");
        clog("Accurancy and duration of IK computation saved.");
        clog("IK end-effector states: \n"
             "Total: " + std::to_string(num_total_samples) + "\n"
             "Valid: " + std::to_string(num_valid_samples) + "\n"
             "Found: " + std::to_string(num_found_samples));

        // Close file for reading
        file_pos.close();
        if (!file_pos.is_open()) {
            clog("File file_pos closed.");
        } else {
            clog("File file_pos not opened", ERROR);
            return -2;
        }

        // Close file for writing
        file_time.close();
        if (!file_time.is_open()) {
            clog("File file_time closed.");
        } else {
            clog("File file_time not opened", ERROR);
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