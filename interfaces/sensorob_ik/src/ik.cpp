//
// Created by jakub on 4.12.2023.
//
#include "sensorob_ik/ik.h"

namespace ik {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik");

    int computeAndLogIK(const std::shared_ptr<rclcpp::Node>& move_group_node,
                        const moveit::planning_interface::MoveGroupInterface& move_group,
                        const std::string& planning_group,
                        const std::string& file_pos_name,
                        const std::string& file_time_name,
                        const double& solver_timeout) {

        const moveit::core::JointModelGroup* joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(planning_group);
        moveit::core::RobotStatePtr cur_state = move_group.getCurrentState(10);

        // Create a tf2_ros::Buffer with a clock
        tf2_ros::Buffer tf_buffer(move_group_node->get_clock());

        // Create a tf2_ros::TransformListener
        tf2_ros::TransformListener tf_listener(tf_buffer, move_group_node);

        // Select move group and IK algorithm
        const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
        clog("solver->getDefaultTimeout(): " + std::to_string(solver->getDefaultTimeout()), LOGGER);
        clog("joint_model_group->getDefaultIKTimeout(): " + std::to_string(joint_model_group->getDefaultIKTimeout()), LOGGER);

        std::vector<double> joint_values_ik;
        int num_valid_samples = 0;
        int num_found_samples = 0;

        // Open file for reading (with translation and orientation data of end effector)
        std::fstream file_pos(file_pos_name, std::ios::in);
        if (!file_pos.is_open()) {
            clog("File file_pos is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File file_pos opened", LOGGER);

        // Open file for writing (logging)
        std::fstream file_time;
        file_time.open(file_time_name, std::ios::out);
        if (!file_time.is_open()) {
            clog("File file_time is not successfully opened, exiting!", LOGGER, ERROR);
            return -1;
        }
        clog("File file_time opened", LOGGER);

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

            // Create pose from file in world frame
            geometry_msgs::msg::Pose pose_desired;
            pose_desired.position.x = lineDoubles[0];
            pose_desired.position.y = lineDoubles[1];
            pose_desired.position.z = lineDoubles[2];
            pose_desired.orientation.w = lineDoubles[3];
            pose_desired.orientation.x = lineDoubles[4];
            pose_desired.orientation.y = lineDoubles[5];
            pose_desired.orientation.z = lineDoubles[6];

            // Perform the transformation to the "world" frame, so we can compute the accuracy of found IK solutions
            geometry_msgs::msg::PoseStamped pose_desired_world;
            geometry_msgs::msg::PoseStamped pose_desired_base_link;
            pose_desired_world.pose = pose_desired;
            pose_desired_world.header.frame_id = "world";

            // Transformation base_link -> world
            try {
                // Wait for the transform to become available with a timeout
                tf_buffer.canTransform("base_link", "world", tf2::TimePointZero, std::chrono::seconds(10));

                tf_buffer.transform(pose_desired_world, pose_desired_base_link, "base_link");
            } catch (tf2::TransformException &ex) {
                clog(ex.what(), LOGGER);
                clog("Failed to transform pose" + std::to_string(num_valid_samples), LOGGER, ERROR );
//                pose_desired_world = pose_desired_base_link;
                continue;
            }

            // Variables for getPositionIK function
            std::vector<geometry_msgs::msg::Pose> ik_poses = {pose_desired_base_link.pose};
            std::vector< double > solution;
            std::vector<double> ik_seed_state = {0, 0, 0, 0, 0, 0};
            moveit_msgs::msg::MoveItErrorCodes result;
            kinematics::KinematicsQueryOptions o;
            o.return_approximate_solution = false;  // we do not want approx solutions

            // Measure the duration of IK computation
            rclcpp::Time start_time = rclcpp::Clock().now();

            // Compute inverse kinematics
            solver->searchPositionIK(pose_desired_base_link.pose, ik_seed_state, solver_timeout, solution, result, o);


            rclcpp::Time end_time = rclcpp::Clock().now();

            // Check if the computation was successful
            if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                /*throw std::runtime_error("Unable to compute IK. Error: " + std::to_string(result.kinematic_error));*/
                /*clog("Did not find IK solution", ERROR);*/
                file_time << -1 << " " << -1 << std::endl;
            }
            else {
                num_found_samples++;
                rclcpp::Duration duration = end_time - start_time;

                file_time << duration.seconds() << " ";

                /*clog("pose_desired_world: "+ std::to_string(pose_desired_world.pose.position.x)+" "+ std::to_string(pose_desired_world.pose.position.y)+" "+ std::to_string(pose_desired_world.pose.position.z));*/

                cur_state->setJointGroupPositions(planning_group, solution);
                const Eigen::Affine3d &pose_found = cur_state->getGlobalLinkTransform("link_6");
                  /*clog("pose_found:   "+ std::to_string(pose_found.translation().x())+" "+ std::to_string(pose_found.translation().y())+" "+ std::to_string(pose_found.translation().z()));*/

                double accuracy = sqrt( pow(pose_desired_world.pose.position.x - pose_found.translation().x(), 2) +
                                            pow(pose_desired_world.pose.position.y - pose_found.translation().y(), 2) +
                                            pow(pose_desired_world.pose.position.z - pose_found.translation().z(), 2));
                file_time << accuracy << " ";

                /*std::string tmp;
                for (const auto &joint : solution) {
                    tmp += " " + std::to_string(joint);
                }
                clog(tmp);*/

                file_time << std::endl;
            }

            // Counter
            num_valid_samples++;
            if (num_valid_samples%100==0) clog("Processed " + std::to_string(num_valid_samples) + " samples.", LOGGER);
        }

        clog("Processed " + std::to_string(num_valid_samples) + " samples.", LOGGER);
        clog("Duration and accuracy of found solutions saved.", LOGGER);
        clog("IK end-effector states: \n"
             "Valid: " + std::to_string(num_valid_samples) + "\n"
             "Found: " + std::to_string(num_found_samples), LOGGER);

        // Close file for reading
        file_pos.close();
        if (!file_pos.is_open()) {
            clog("File file_pos closed.", LOGGER);
        } else {
            clog("File file_pos not opened", LOGGER, ERROR);
            return -2;
        }

        // Close file for writing
        file_time.close();
        if (!file_time.is_open()) {
            clog("File file_time closed.", LOGGER);
        } else {
            clog("File file_time not opened", LOGGER, ERROR);
            return -2;
        }

        return 0;
    }

}
