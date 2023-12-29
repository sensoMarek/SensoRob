//
// Created by jakub on 4.12.2023.
//
#include "sensorob_ik_interface/fk.h"

namespace fk {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("fk");


    int computeAndLogFK(const std::shared_ptr<rclcpp::Node>& move_group_node,
                        const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& planning_group,
                        int num_of_joint_samples,
                        const std::string& file_name) {

        const moveit::core::JointModelGroup* joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(planning_group);
        moveit::core::RobotStatePtr cur_state = move_group.getCurrentState(10);
        const std::vector<std::string>& joint_names = move_group.getActiveJoints();
        int num_processed_samples = 0;

        rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr validity_client;
        validity_client = move_group_node->create_client<moveit_msgs::srv::GetStateValidity>("check_state_validity");

        // Wait for the service to become available
        while (!validity_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                clog("Interrupted while waiting for the service. Exiting.", LOGGER, ERROR);
                break;
            }
            clog("Service not available, waiting again...", LOGGER);
        }
        clog("'check_state_validity' service is available", LOGGER);

        // Create a request
        auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        request->group_name = planning_group;
        request->robot_state.joint_state.name = joint_names;
        request->robot_state.joint_state.header.frame_id = "base_link";

        // Get joint model bounds (min, max)
        std::vector<const moveit::core::JointModel::Bounds*> jmb = joint_model_group->getActiveJointModelsBounds();
        std::vector<std::vector<double>> joint_limits;
        joint_limits.resize(6, std::vector<double>(2, 0.0));
        for (unsigned long i=0; i<6; i++) {
            joint_limits[i][0] = jmb[i]->data()->max_position_ - 0.0000001;
            joint_limits[i][1] = jmb[i]->data()->min_position_ + 0.0000001;
        }

        // Create a vector of all robot configurations
        clog("Creating robot states", LOGGER);
        std::vector<std::vector<double>> joint_samples;
        for (unsigned long i=0; i<6; i++) {
            joint_samples.push_back(interpolate(std::min(joint_limits[i][0],joint_limits[i][1]),
                                                std::max(joint_limits[i][0],joint_limits[i][1]),
                                                num_of_joint_samples));
        }

        std::vector<double> joint_states;
        std::fstream file(file_name, std::ios::out);

        if (!file.is_open()) {
            clog("File is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File opened", LOGGER);

        // Iterate over all states and check its validity (self-collision point of view)
        clog("Validating and saving valid robot states", LOGGER);
        for (int i0 = 0; i0 < num_of_joint_samples; i0++) {
            for (int i1 = 0; i1 < num_of_joint_samples; i1++) {
                for (int i2 = 0; i2 < num_of_joint_samples; i2++) {
                    for (int i3 = 0; i3 < num_of_joint_samples; i3++) {
                        for (int i4 = 0; i4 < num_of_joint_samples; i4++) {
                            joint_states.clear();
                            joint_states = {joint_samples[0][i0],
                                            joint_samples[1][i1],
                                            joint_samples[2][i2],
                                            joint_samples[3][i3],
                                            joint_samples[4][i4],
                                            0.0};

                            // Set current join states and send asynchronous service request, then wait for being ready
                            request->robot_state.joint_state.position = joint_states;
                            auto future = validity_client->async_send_request(request);
                            future.wait();

                            // Get response
                            auto response = future.get();
                            if (response->valid) {
                                /*clog("Joint values are valid!");*/

                                cur_state->setJointGroupPositions(planning_group, joint_states);
                                const Eigen::Affine3d &end_effector_state = cur_state->getGlobalLinkTransform("link_6");

                                Eigen::Quaterniond quaternion(end_effector_state.rotation());
                                file << end_effector_state.translation().x() << " "
                                     << end_effector_state.translation().y() << " "
                                     << end_effector_state.translation().z() << " "
                                     << quaternion.w() << " "
                                     << quaternion.x() << " "
                                     << quaternion.y() << " "
                                     << quaternion.z() << " "
                                     << std::endl;
                            }
                            /*else {
                                clog("Joint values are not valid!", WARN);
                            }*/

                            num_processed_samples++;
                            if (num_processed_samples%100==0) clog("Processed " + std::to_string(num_processed_samples) + " samples.", LOGGER);
                        }
                    }
                }
            }
        }

        // Close file
        clog("Processed " + std::to_string(num_processed_samples) + " samples.", LOGGER);
        clog("Translation and orientation saved.", LOGGER);
        file.close();
        if (!file.is_open()) {
            clog("File closed.", LOGGER);
        } else {
            clog("File not opened", LOGGER, ERROR);
            return -2;
        }

        return 0;
    }


    std::vector<double> interpolate(double start, double end, int n) {
        std::vector<double> result;
        double step = std::abs(end - start) / (n - 1);

        for (int i = 0; i < n; ++i) {
//            result.push_back(std::round((start + i * step)*1000000.0)/1000000.0); //round to 6 decimal places
            result.push_back(start + i * step);
        }

        return result;
    }
}
