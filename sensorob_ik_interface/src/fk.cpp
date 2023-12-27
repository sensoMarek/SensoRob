//
// Created by jakub on 4.12.2023.
//

#include "sensorob_ik_interface/fk.h"

namespace fk {
    static const rclcpp::Logger LOGGER = rclcpp::get_logger("fk");


    int computeAndLogFK(const std::string& planning_group,
                        const moveit::core::JointModelGroup* joint_model_group,
                        const moveit::core::RobotStatePtr& cur_state,
                        int num_of_joint_samples,
                        std::string file_name) {

        std::vector<const moveit::core::JointModel::Bounds*> jmb = joint_model_group->getActiveJointModelsBounds();
        std::vector<std::vector<double>> joint_limits; // only 5 joints used for space sampling, each contains max and min position

        joint_limits.resize(6, std::vector<double>(2, 0.0));
        for (unsigned long i=0; i<6; i++) {
            joint_limits[i][0] = jmb[i]->data()->max_position_;
            joint_limits[i][1] = jmb[i]->data()->min_position_;
        }

        std::vector<std::vector<double>> joint_samples;


        for (unsigned long i=0; i<5; i++) {
            joint_samples.push_back(interpolate(std::min(joint_limits[i][0],joint_limits[i][1]),
                                                std::max(joint_limits[i][0],joint_limits[i][1]),
                                                num_of_joint_samples));
        }

        for (unsigned long i=0; i<5; i++) {
//        clog( "joint_samples [%ld]: \n ", i);
            for (unsigned long j=0; j<joint_samples[i].size(); j++) {
//            clog( "[%ld/%zu] %.2f: \n ",j,joint_samples[i].size(), joint_samples[i][j]);
            }
        }

        std::vector<double> joint_states;
        std::fstream file(file_name, std::ios::out);

        if (!file.is_open()) {
            clog("File is not successfully opened, exiting!", ERROR);
            return -1;
        }

        clog("File opened");

        for (int i0 = 0; i0 < num_of_joint_samples; i0++) {  // joint1
            for (int i1 = 0; i1 < num_of_joint_samples; i1++) {  // joint2
                for (int i2 = 0; i2 < num_of_joint_samples; i2++) {  // joint3
                    for (int i3 = 0; i3 < num_of_joint_samples; i3++) {  // joint4
                        for (int i4 = 0; i4 < num_of_joint_samples; i4++) {  // joint5

                            joint_states = {joint_samples[0][i1],
                                            joint_samples[1][i1],
                                            joint_samples[2][i2],
                                            joint_samples[3][i3],
                                            joint_samples[4][i4],
                                            0.0};

                            /*clog( "[%.2f], [%.2f], [%.2f], [%.2f], [%.2f], [%.2f]",
                                        joint_states[0],
                                        joint_states[1],
                                        joint_states[2],
                                        joint_states[3],
                                        joint_states[4],
                                        joint_states[5]);*/

                            cur_state->setJointGroupPositions(planning_group, joint_states);
                            const Eigen::Affine3d &end_effector_state = cur_state->getGlobalLinkTransform("link_6");

                            /*clog( "Translation: x: [%.3f], y: [%.3f], z: [%.3f]",
                                        end_effector_state.translation().x(),
                                        end_effector_state.translation().y(),
                                        end_effector_state.translation().z());*/

                            Eigen::Quaterniond quaternion(end_effector_state.rotation());
                            /*clog( "Rotation: w: [%.3f], x: [%.3f], y: [%.3f], z: [%.3f]",
                                        quaternion.w(),
                                        quaternion.x(),
                                        quaternion.y(),
                                        quaternion.z());*/

                            file << end_effector_state.translation().x() << " "
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

        clog("Translation and orientation saved.");

        file.close();
        if (!file.is_open()) {
            clog("File closed.");
        } else {
            clog("File not opened", ERROR);
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
