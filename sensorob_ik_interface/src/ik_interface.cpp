#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <Eigen/Dense>

#include "sensorob_ik_interface/ik_interface.h"


// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ik");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("ik_interface", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "sensorob_group";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    visual_tools.trigger();

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");


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

    moveit::core::RobotStatePtr cur_state = move_group.getCurrentState(10);
    std::vector<double> joint_states;

    // what am i doing?
    for (int i0=0; i0<num_of_joint_samples; i0++) {  // joint1
        for (int i1=0; i1<num_of_joint_samples; i1++) {  // joint2
            for (int i2=0; i2<num_of_joint_samples; i2++) {  // joint3
                for (int i3=0; i3<num_of_joint_samples; i3++) {  // joint4
                    for (int i4=0; i4<num_of_joint_samples; i4++) {  // joint5
                        for (int i5=0; i5<num_of_joint_samples; i5++) {  // joint6

                            joint_states = {joint_samples[0][i1],
                                            joint_samples[1][i1],
                                            joint_samples[2][i2],
                                            joint_samples[3][i3],
                                            joint_samples[4][i4],
                                            joint_samples[5][i5]};

                            RCLCPP_INFO(LOGGER, "[%.2f], [%.2f], [%.2f], [%.2f], [%.2f], [%.2f]",
                                        joint_states[0],
                                        joint_states[1],
                                        joint_states[2],
                                        joint_states[3],
                                        joint_states[4],
                                        joint_states[5]);

                            cur_state->setJointGroupPositions(PLANNING_GROUP, joint_states);
                            const Eigen::Affine3d &end_effector_state = cur_state->getGlobalLinkTransform("link_6");

                            RCLCPP_INFO(LOGGER, "Translation: x: [%.3f], y: [%.3f], z: [%.3f]",
                                        end_effector_state.translation().x(),
                                        end_effector_state.translation().y(),
                                        end_effector_state.translation().z());

                            Eigen::Quaterniond quaternion(end_effector_state.rotation());
                            RCLCPP_INFO(LOGGER, "Rotation: w: [%.3f], x: [%.3f], y: [%.3f], z: [%.3f]",
                                        quaternion.w(),
                                        quaternion.x(),
                                        quaternion.y(),
                                        quaternion.z());
                        }
                    }
                }
            }
        }
    }







    // Planning to a joint-space goal
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    //
    // Next get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Now, let's modify one of the joints, plan to the new joint space goal, and visualize the plan.
    joint_group_positions[0] = -0.5;  // radians
    joint_group_positions[1] -= 0.2;

    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }

    // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
    // The default values are 10% (0.1).
    // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
    // or set explicit factors in your code if you need your robot to move faster.
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz:
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint_Space_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}

std::vector<double> interpolate(double start, double end, int n) {
    std::vector<double> result;
    double step = std::abs(end - start) / (n - 1);

    for (int i = 0; i < n; ++i) {
        result.push_back(std::round((start + i * step)*1000000.0)/1000000.0); //round to 6 decimal places
    }

    return result;
}

