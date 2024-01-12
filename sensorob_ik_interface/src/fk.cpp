//
// Created by jakub on 4.12.2023.
//
#include "sensorob_ik_interface/fk.h"

namespace fk {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("fk");

    int computeAndLogFK(const std::shared_ptr<rclcpp::Node>& move_group_node,
                        const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& planning_group,
                        int num_of_samples,
                        const std::string& file_pos_name,
                        const std::string& file_joint_name) {

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

        std::vector<double> joint_states;
        std::fstream file_pos(file_pos_name, std::ios::out);

        if (!file_pos.is_open()) {
            clog("File file_pos_name is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File file_pos_name opened", LOGGER);

        std::fstream file_joint(file_joint_name, std::ios::out);

        if (!file_pos.is_open()) {
            clog("File file_joint_name is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File file_joint_name opened", LOGGER);
        

        // Iterate over all states and check its validity (self-collision point of view)
        clog("Validating and saving valid robot states", LOGGER);
        int i =0;
        while (i< num_of_samples) {
//        for (int i = 0; i < num_of_samples; i++) {
            cur_state->setToRandomPositions();
            joint_states.clear();
            cur_state->copyJointGroupPositions(planning_group, joint_states);

            // Set current join states and send asynchronous service request, then wait for being ready
            request->robot_state.joint_state.position = joint_states;
            auto future = validity_client->async_send_request(request);
            future.wait();

            // Get response
            auto response = future.get();
            if (response->valid) {
                i++;
                /*clog("Joint values are valid!");*/

                cur_state->setJointGroupPositions(planning_group, joint_states);
                const Eigen::Affine3d &end_effector_state = cur_state->getGlobalLinkTransform("link_6");

                Eigen::Quaterniond quaternion(end_effector_state.rotation());

                /*log position and orientation of end effector*/
                file_pos << end_effector_state.translation().x() << " "
                     << end_effector_state.translation().y() << " "
                     << end_effector_state.translation().z() << " "
                     << quaternion.w() << " "
                     << quaternion.x() << " "
                     << quaternion.y() << " "
                     << quaternion.z() << " "
                     << std::endl;

                /*log joint states position*/
                file_joint << joint_states[0] << " "
                         << joint_states[1] << " "
                         << joint_states[2] << " "
                         << joint_states[3] << " "
                         << joint_states[4] << " "
                         << joint_states[5] << " "
                         //new
                         << std::endl;
            }
            /*else {
                clog("Joint values are not valid!", WARN);
            }*/

            num_processed_samples++;
            if (num_processed_samples%100==0) clog("Processed " + std::to_string(num_processed_samples) + " samples.", LOGGER);

        }

        // Close files
        clog("Processed " + std::to_string(num_processed_samples) + " samples.", LOGGER);
        clog("Translation and orientation saved.", LOGGER);
        file_pos.close();
        if (!file_pos.is_open()) {
            clog("File file_pos_name closed.", LOGGER);
        } else {
            clog("File file_pos_name not closed", LOGGER, ERROR);
            return -2;
        }

        file_joint.close();
        if (!file_joint.is_open()) {
            clog("File file_joint_name closed.", LOGGER);
        } else {
            clog("File file_joint_name not closed", LOGGER, ERROR);
            return -2;
        }

        return 0;
    }
}
