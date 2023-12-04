//
// Created by jakub on 4.12.2023.
//

#include "sensorob_ik_interface/ik.h"


namespace ik {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik");

    int computeAndLogIK(const std::string& planning_group,
                        const moveit::core::JointModelGroup* joint_model_group,
                        const moveit::core::RobotStatePtr& cur_state,
                        int num_provided_samples,
                        std::string file_pos_name,
                        std::string file_time_name) {

        kinematics::KinematicsQueryOptions o;
        o.return_approximate_solution = false;
        std::vector<double> joint_values_ik;
        int num_processed_samples = 0;
        int num_success = 0;
        int timeout = 0.6; // t[s]


        // open file for reading (with translation and orientation data of end effector)
        std::fstream file_pos(file_pos_name, std::ios::in);
        if (!file_pos.is_open()) {
            clog("File file_pos is not successfully opened, exiting!", ERROR);
            return -1;
        }
        clog("File file_pos opened");

        // open file for writing (logging)
        std::fstream file_time;
        file_time.open(file_time_name, std::ios::out);
        if (!file_time.is_open()) {
            clog("File file_time is not successfully opened, exiting!", ERROR);
            return -1;
        }
        clog("File file_time opened");

        // read lines from the file
        std::string line;
        while (std::getline(file_pos, line)) {
            // use a stringstream to parse doubles from the line
            std::istringstream iss(line);
            std::vector<double> lineDoubles;
            double num;

            while (iss >> num) {
                lineDoubles.push_back(num);

                // check for a newline character
                if (iss.peek() == '\n')
                    break;
            }
            /* for (const auto& number : lineDoubles) {
                std::cout << number << " ";
            }
            std::cout << std::endl;*/

            // add to counter
            num_processed_samples++;
            if (num_processed_samples%50==0) clog("Processed " + std::to_string(num_processed_samples) + " samples.");

            // create pose read from file
            geometry_msgs::msg::PoseStamped pose_desired;
            pose_desired.pose.position.x = lineDoubles[0];
            pose_desired.pose.position.y = lineDoubles[1];
            pose_desired.pose.position.z = lineDoubles[2];
            pose_desired.header.frame_id = "base_link";
            pose_desired.header.stamp = rclcpp::Clock().now();

            // measure the duration of IK computation
            rclcpp::Time start_time = rclcpp::Clock().now();
            bool ik_found = cur_state->setFromIK(joint_model_group, pose_desired.pose, timeout,
                                                 moveit::core::GroupStateValidityCallbackFn(), o);
            rclcpp::Time end_time = rclcpp::Clock().now();
            rclcpp::Duration duration = end_time - start_time;

            // if IK found, validate and log results
            if (ik_found) {
                num_success++;
                cur_state->copyJointGroupPositions(joint_model_group, joint_values_ik);

                cur_state->setJointGroupPositions(planning_group, joint_values_ik);
                const Eigen::Affine3d &pose_found = cur_state->getGlobalLinkTransform("link_6");

                double accurancy = sqrt(abs(pose_desired.pose.position.x - pose_found.translation().x()) +
                                        abs(pose_desired.pose.position.y - pose_found.translation().y()) +
                                        abs(pose_desired.pose.position.z - pose_found.translation().z()));
                file_time << accurancy << " " << duration.seconds() << std::endl;
            } else {
//                clog("Did not find IK solution", ERROR);
                file_time << 0 << " " << 0 << std::endl;
            }
        }

        clog("Accurancy and duration of IK computation saved.");
        clog("IK end-effector states: \n"
             "Provided:   " + std::to_string(num_provided_samples) + "\n"
             "Processed:" + std::to_string(num_processed_samples) + "\n"
             "Computed:" + std::to_string(num_success));

        // close file for reading
        file_pos.close();
        if (!file_pos.is_open()) {
            clog("File file_pos closed.");
        } else {
            clog("File file_pos not opened", ERROR);
            return -2;
        }

        // close file for writing
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