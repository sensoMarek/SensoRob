
#include "sensorob_planner/file_logger.h"

namespace file_logger {

std::string get_current_time() {
    // Get the current system time point
    auto now = std::chrono::system_clock::now();

    // Convert the system time point to a time_t object
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);

    // Convert the time_t object to a tm struct for local time
    std::tm local_tm = *std::localtime(&now_time_t);

    // return the current date and time
    std::stringstream sst;
    sst << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S");
    return sst.str();
}


std::string create_new_dir(const std::string name, const std::string path, rclcpp::Logger& LOGGER) {
    std::string dir_name = path + "/" + name;    
    // Check if the directory already exists
    if (access(dir_name.c_str(), F_OK) == -1) {
        // Create the directory
        if (mkdir(dir_name.c_str(), 0777) == 0) {
            // clog("Directory " + dir_name + " created successfully", LOGGER);
            return dir_name;
        } else {
            clog("Failed to create directory " + dir_name, LOGGER, ERROR);
        }
    } 
    // else {
    //     std::cerr << "Directory already exists: " << dir_name << std::endl;
    // }

    return dir_name;

}


std::fstream open_file(
    const std::string file_name, 
    const std::string dir_name,
    rclcpp::Logger& LOGGER) 
    {
    std::fstream file(dir_name + "/" + file_name, std::ios::out);
    if (!file.is_open()) {
        clog("File " + file_name + " is not successfully opened, no logging!!", LOGGER, ERROR);
    } 
    // else {
    //     clog("File " + file_name + " opened", LOGGER);
    // }

    return file;
}


int logTool(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string dir_name,
    rclcpp::Logger& LOGGER) 
{
    std::fstream file = open_file("tool.txt", dir_name, LOGGER);

    if (!file.is_open()) return -1;
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
    std::string endEffectorLink = move_group.getEndEffectorLink();
    moveit_msgs::msg::RobotTrajectory traj = plan.trajectory_;

    for (uint j=0; j<traj.joint_trajectory.points.size(); j++){
        robot_state->setJointGroupActivePositions(joint_model_group, traj.joint_trajectory.points[j].positions);
        robot_state->updateLinkTransforms();
        const Eigen::Isometry3d& end_effector_state = robot_state->getFrameTransform(endEffectorLink);
        geometry_msgs::msg::Pose end_effector_pose =  tf2::toMsg(end_effector_state);

        tf2::Quaternion q(
                end_effector_pose.orientation.x,
                end_effector_pose.orientation.y,
                end_effector_pose.orientation.z,
                end_effector_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        tf2Scalar roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // zapis do suboru
        file << "point: " << j << std::endl;
        file << end_effector_pose.position.x << " " << end_effector_pose.position.y << " " << end_effector_pose.position.z << "\n";
        file << roll << " " << pitch << " " << yaw << "\n";
        file << std::fixed << std::setprecision(16) << plan.trajectory_.joint_trajectory.points[j].time_from_start.sec + plan.trajectory_.joint_trajectory.points[j].time_from_start.nanosec/1e9 << std::endl;
    }

    file.close();
    return 0;
}


int logTrajectory(
    [[maybe_unused]] moveit::planning_interface::MoveGroupInterface& move_group, 
    [[maybe_unused]] const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string dir_name,
    rclcpp::Logger& LOGGER) 
{
    std::fstream file = open_file("trajectory.txt", dir_name, LOGGER);

    if (!file.is_open()) return -1;

    for (uint j=0; j<plan.trajectory_.joint_trajectory.points.size(); j++) {
        // point
        file << "point: " << j << std::endl;   

        // pos
        for (uint i=0; i<6; i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].positions[i] << " ";
            if (i==(6-1)) {
                file << std::endl;
            }    
        }

        // vel
        for (uint i=0; i<6; i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].velocities[i] << " ";
            if (i==(6-1)) {
                file << std::endl;
            }    
        }

        // acc
        for (uint i=0; i<6; i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].accelerations[i] << " ";
            if (i==(6-1)) {
                file << std::endl;
            }    
        }

        // time from start
        file << std::fixed << std::setprecision(16) << plan.trajectory_.joint_trajectory.points[j].time_from_start.sec + plan.trajectory_.joint_trajectory.points[j].time_from_start.nanosec/1e9 << std::endl;
    }

    file.close();
    return 0;
}

trajectory_attributes logSummary(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string dir_name,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER) 
{
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
    const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(planning_group);
    std::string endEffectorLink = move_group.getEndEffectorLink();
    moveit_msgs::msg::RobotTrajectory traj = plan.trajectory_;

    // cartesian path distance of the tool  
    double path_distance = 0.0;
    robot_state->setJointGroupActivePositions(joint_model_group, traj.joint_trajectory.points[0].positions);
    robot_state->updateLinkTransforms();
    const Eigen::Isometry3d& eef_state_old = robot_state->getFrameTransform(endEffectorLink);
    geometry_msgs::msg::Pose eef_pose_old =  tf2::toMsg(eef_state_old);

    for (uint j=1; j<traj.joint_trajectory.points.size(); j++){
        robot_state->setJointGroupActivePositions(joint_model_group, traj.joint_trajectory.points[j].positions);
        robot_state->updateLinkTransforms();
        const Eigen::Isometry3d& eef_state = robot_state->getFrameTransform(endEffectorLink);
        geometry_msgs::msg::Pose eef_pose =  tf2::toMsg(eef_state);


        path_distance += sqrt( 
            pow((eef_pose.position.x - eef_pose_old.position.x), 2) + 
            pow((eef_pose.position.y - eef_pose_old.position.y), 2) + 
            pow((eef_pose.position.z - eef_pose_old.position.z), 2) 
        );

        eef_pose_old = eef_pose;
    }

    // joint distance + max vel + max acc
    std::vector<double> joints_distance; joints_distance.resize(6); std::fill(joints_distance.begin(), joints_distance.end(), 0.0);
    std::vector<double> max_joint_vel; max_joint_vel.resize(6); std::fill(max_joint_vel.begin(), max_joint_vel.end(), 0.0);
    std::vector<double> max_joint_acc; max_joint_acc.resize(6); std::fill(max_joint_acc.begin(), max_joint_acc.end(), 0.0);
    
    std::vector<double> joints_pos_old = plan.trajectory_.joint_trajectory.points[0].positions;
    double joints_distance_sum = 0.0;

    for (uint j=1; j<plan.trajectory_.joint_trajectory.points.size(); j++) {
        
        for (uint i=0; i<6; i++) {
            // joint distance
            joints_distance[i] += abs(plan.trajectory_.joint_trajectory.points[j].positions[i] - plan.trajectory_.joint_trajectory.points[j-1].positions[i]);

            // max vel, max acc
            if (abs(plan.trajectory_.joint_trajectory.points[j].velocities[i]) > max_joint_vel[i]) max_joint_vel[i] = abs(plan.trajectory_.joint_trajectory.points[j].velocities[i]);
            if (abs(plan.trajectory_.joint_trajectory.points[j].accelerations[i]) > max_joint_acc[i]) max_joint_acc[i] = abs(plan.trajectory_.joint_trajectory.points[j].accelerations[i]);
        }
    }

    for (uint i=0; i<joints_distance.size(); i++) {
        joints_distance_sum += abs(joints_distance[i]);
    }

    double trajectory_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec/1e9;
    if (allow_file_logging) {
        std::fstream file = open_file("summary.txt", dir_name, LOGGER);
        if (file.is_open()) {
            // logging
            file << "Planning time [s]: " << std::fixed << std::setprecision(8) << plan.planning_time_ << std::endl;
            file << "Trajectory time [s]: " << std::fixed << std::setprecision(6) << trajectory_time << std::endl;
            file << "Number of points [-]: " << plan.trajectory_.joint_trajectory.points.size() << std::endl;
            file << "Tool path distance [m]: " << std::fixed << std::setprecision(6) << path_distance << std::endl;
            
            file << "Joint distance [rad]" << std::endl;
            for (uint i=0; i<joints_distance.size(); i++) {
                file << "  - joint " << i+1 << " : " << std::fixed << std::setprecision(6) << joints_distance[i] << std::endl;
            }
            
            file << "Sum joint distance [rad]: " << std::fixed << std::setprecision(6) << joints_distance_sum << std::endl;
            
            file << "Joint max abs vel [rad/s]" << std::endl;
            for (uint i=0; i<max_joint_vel.size(); i++) {
                file << "  - joint " << i+1 << " : " << std::fixed << std::setprecision(6) << max_joint_vel[i] << std::endl;
            }
            
            file << "Joint max abs acc [rad/s^2]" << std::endl;
            for (uint i=0; i<max_joint_acc.size(); i++) {
                file << "  - joint " << i+1 << " : " << std::fixed << std::setprecision(6) << max_joint_acc[i] << std::endl;
            }

            file.close();
        }
        else {
            clog("Failed to open file summary.txt", LOGGER, ERROR);
        }
    }

    trajectory_attributes traj_attributes;
    traj_attributes.tool_distance = path_distance;
    traj_attributes.joint_distance = joints_distance_sum;
    traj_attributes.planning_time = plan.planning_time_;
    traj_attributes.number_of_points = plan.trajectory_.joint_trajectory.points.size();
    traj_attributes.trajectory_time = trajectory_time;

    return traj_attributes;

}

trajectory_attributes logAll(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string scene_dir_path,
    const std::string scene_dir_name,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER) 
{
    std::string dir_name;

    if (allow_file_logging){
        // create subdir
        dir_name = create_new_dir(scene_dir_name, scene_dir_path, LOGGER);

        // log to files
        logTool(move_group, planning_group, plan, dir_name, LOGGER);
        logTrajectory(move_group, planning_group, plan, dir_name, LOGGER);
    }


    trajectory_attributes traj_attributes = logSummary(move_group, planning_group, plan, dir_name, allow_file_logging, LOGGER);
    traj_attributes.dir_name = scene_dir_path;

    return traj_attributes;

}

std::string trajectory_attributes_to_string(const trajectory_attributes& attributes) {
    std::stringstream ss;
    ss << "Planning time: " << attributes.planning_time << ", ";
    ss << "Trajectory time: " << attributes.trajectory_time << ", ";
    ss << "Tool distance: " << attributes.tool_distance << ", ";
    ss << "Joint distance: " << std::abs(attributes.joint_distance) << ", ";
    ss << "Number of points: " << attributes.number_of_points;
    
    return ss.str();
}

int log_struct(
    const std::vector<trajectory_attributes> traj_attributes_vector, 
    const  uint num_rerun,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER) 
    {

    double path_distance = 0.0;
    double joints_distance_sum = 0.0;
    double trajectory_time = 0.0;
    double planning_time = 0.0;
    double number_of_points = 0.0;

    try {
        if (traj_attributes_vector.size() == 0) {
            throw std::invalid_argument("Empty vector");
        }
    } catch (const std::invalid_argument& e) {
        clog("Empty vector of trajectory attributes", LOGGER, ERROR);
        return -1;
    }
    
    double num_success = traj_attributes_vector.size();

    for (uint i=0; i<(uint)num_success; i++) {
        path_distance += traj_attributes_vector[i].tool_distance;
        joints_distance_sum += traj_attributes_vector[i].joint_distance;
        trajectory_time += traj_attributes_vector[i].trajectory_time;
        planning_time += traj_attributes_vector[i].planning_time;
        number_of_points += traj_attributes_vector[i].number_of_points;
    }

    trajectory_attributes final_traj_attributes;
    final_traj_attributes.tool_distance = path_distance/num_success;
    final_traj_attributes.joint_distance = joints_distance_sum/num_success;
    final_traj_attributes.trajectory_time = trajectory_time/num_success;
    final_traj_attributes.planning_time = planning_time/num_success;
    final_traj_attributes.number_of_points = number_of_points/num_success;

    if (allow_file_logging) {
        final_traj_attributes.dir_name = traj_attributes_vector[0].dir_name;
        std::fstream file = open_file("final_summary.txt",  final_traj_attributes.dir_name, LOGGER);
        if (file.is_open()) {
            // logging
            file << "Number of successful runs [-]: " << (int)num_success << "/" << num_rerun << std::endl;
            file << "Planning time [s]: " << std::fixed << std::setprecision(8) << final_traj_attributes.planning_time << std::endl;
            file << "Trajectory time [s]: " << std::fixed << std::setprecision(6) << final_traj_attributes.trajectory_time << std::endl;
            file << "Number of points [-]: " << final_traj_attributes.number_of_points << std::endl;
            file << "Tool path distance [m]: " << std::fixed << std::setprecision(6) << final_traj_attributes.tool_distance << std::endl;
            file << "Joint distance [rad]: " << std::fixed << std::setprecision(6) << final_traj_attributes.joint_distance << std::endl;
            file.close();
        }
        else {
            clog("Failed to open file final_summary.txt", LOGGER, ERROR);
        }
        file.close();
    }

    clog("Average results:", LOGGER);
    clog(trajectory_attributes_to_string(final_traj_attributes), LOGGER);

    return 0;
}

int log_vectors(
    const std::vector<trajectory_attributes> traj_attributes_vector, 
    [[maybe_unused]] const  uint num_rerun,
    bool allow_file_logging,
    rclcpp::Logger& LOGGER) 
    {

    if (!allow_file_logging) return -1;

    try {
        if (traj_attributes_vector.size() == 0) {
            throw std::invalid_argument("Empty vector");
        }
    } catch (const std::invalid_argument& e) {
        clog("Empty vector of trajectory attributes", LOGGER, ERROR);
        return -1;
    }

    std::fstream file = open_file("vectors.txt",  traj_attributes_vector[0].dir_name, LOGGER);
    if (file.is_open()) {
        file << "tool_distance" << " ";
        file << "joint_distance" << " ";
        file << "trajectory_time" << " ";
        file << "planning_time" << " ";
        file << "number_of_points" << std::endl;
        
        for (uint i=0; i<(uint)traj_attributes_vector.size(); i++) {
            file << traj_attributes_vector[i].tool_distance << " ";
            file << traj_attributes_vector[i].joint_distance << " ";
            file << traj_attributes_vector[i].trajectory_time << " ";
            file << traj_attributes_vector[i].planning_time << " ";
            file << traj_attributes_vector[i].number_of_points << std::endl;
        }
    }
    else {
        clog("Failed to open file vectors.txt", LOGGER, ERROR);
    }

    file.close();

    return 0;
}


}