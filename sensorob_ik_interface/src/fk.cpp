//
// Created by jakub on 4.12.2023.
//

#include "sensorob_ik_interface/fk.h"

namespace fk {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik");

    int computeAndLogFK(const std::string& PLANNING_GROUP, const moveit::core::JointModelGroup* joint_model_group, const moveit::core::RobotStatePtr& cur_state) {
        // Find the joint position limits
        // Get the active joint models and their bounds

        std::vector<const moveit::core::JointModel::Bounds*> jmb = joint_model_group->getActiveJointModelsBounds();
        std::vector<std::vector<double>> joint_limits; // 6 joints, each contains max and min position
        joint_limits.resize(6, std::vector<double>(2, 0.0));

        for (unsigned long i=0; i<6; i++) {
            joint_limits[i][0] = jmb[i]->data()->max_position_;
            joint_limits[i][1] = jmb[i]->data()->min_position_;
        }

        std::vector<std::vector<double>> joint_samples;
        double num_of_joint_samples=5;

        for (unsigned long i=0; i<6; i++) {
            joint_samples.push_back(interpolate(std::min(joint_limits[i][0],joint_limits[i][1]),
                                                std::max(joint_limits[i][0],joint_limits[i][1]),
                                                num_of_joint_samples));
        }

        for (unsigned long i=0; i<6; i++) {
//        RCLCPP_INFO(LOGGER, "joint_samples [%ld]: \n ", i);
            for (unsigned long j=0; j<joint_samples[i].size(); j++) {
//            RCLCPP_INFO(LOGGER, "[%ld/%zu] %.2f: \n ",j,joint_samples[i].size(), joint_samples[i][j]);
            }
        }

//        moveit::core::RobotStatePtr cur_state = move_group.getCurrentState(10);
        std::vector<double> joint_states;

        std::string filePosName = "/home/jakub/ros2_ws/src/SensoRob/sensorob_logs/pos.csv";
        std::fstream filePos;
        filePos.open(filePosName, std::ios::out);

        if (!filePos.is_open()) {
            RCLCPP_INFO(LOGGER, "File is not successfully opened, exiting!");
            return -1;
        }
        else {
            RCLCPP_INFO(LOGGER, "File opened.");
            filePos << "trans.x " << "trans.y " << "trans.z "
                    << "rot.w " << "rot.x " << "rot.y " << "rot.z "
                    << std::endl;

            // what am i doing?
            for (int i0 = 0; i0 < num_of_joint_samples; i0++) {  // joint1
                for (int i1 = 0; i1 < num_of_joint_samples; i1++) {  // joint2
                    for (int i2 = 0; i2 < num_of_joint_samples; i2++) {  // joint3
                        for (int i3 = 0; i3 < num_of_joint_samples; i3++) {  // joint4
                            for (int i4 = 0; i4 < num_of_joint_samples; i4++) {  // joint5
                                for (int i5 = 0; i5 < num_of_joint_samples; i5++) {  // joint6

                                    joint_states = {joint_samples[0][i1],
                                                    joint_samples[1][i1],
                                                    joint_samples[2][i2],
                                                    joint_samples[3][i3],
                                                    joint_samples[4][i4],
                                                    joint_samples[5][i5]};

                                    /*RCLCPP_INFO(LOGGER, "[%.2f], [%.2f], [%.2f], [%.2f], [%.2f], [%.2f]",
                                                joint_states[0],
                                                joint_states[1],
                                                joint_states[2],
                                                joint_states[3],
                                                joint_states[4],
                                                joint_states[5]);*/

                                    cur_state->setJointGroupPositions(PLANNING_GROUP, joint_states);
                                    const Eigen::Affine3d &end_effector_state = cur_state->getGlobalLinkTransform("link_6");

                                    /*RCLCPP_INFO(LOGGER, "Translation: x: [%.3f], y: [%.3f], z: [%.3f]",
                                                end_effector_state.translation().x(),
                                                end_effector_state.translation().y(),
                                                end_effector_state.translation().z());*/

                                    Eigen::Quaterniond quaternion(end_effector_state.rotation());
                                    /*RCLCPP_INFO(LOGGER, "Rotation: w: [%.3f], x: [%.3f], y: [%.3f], z: [%.3f]",
                                                quaternion.w(),
                                                quaternion.x(),
                                                quaternion.y(),
                                                quaternion.z());*/

                                    filePos << end_effector_state.translation().x() << " "
                                            << end_effector_state.translation().y() << " "
                                            << end_effector_state.translation().z() << " "
                                            << quaternion.w() << " "
                                            << quaternion.x() << " "
                                            << quaternion.y() << " "
                                            << quaternion.z() << " "
                                            << std::endl;
                                }
                            }
                        }
                    }
                }
            }

            filePos.close();
            if (!filePos.is_open()) {
                RCLCPP_INFO(LOGGER, "File closed.");
            } else {
                RCLCPP_INFO(LOGGER, "File not closed!");
                return -2;
            }

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
