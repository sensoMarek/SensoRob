
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
    std::string& dir_name,
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
    std::string& dir_name,
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
    moveit::planning_interface::MoveGroupInterface& move_group, 
    [[maybe_unused]] const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    std::string& dir_name,
    rclcpp::Logger& LOGGER) 
{
    std::fstream file = open_file("trajectory.txt", dir_name, LOGGER);

    if (!file.is_open()) return -1;

    for (uint j=0; j<plan.trajectory_.joint_trajectory.points.size(); j++) {
        // point
        file << "point: " << j << std::endl;   

        // pos
        for (uint i=0; i<move_group.getJoints().size(); i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].positions[i] << " ";
            if (i==(move_group.getJoints().size()-1)) {
                file << std::endl;
            }    
        }

        // vel
        for (uint i=0; i<move_group.getJoints().size(); i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].velocities[i] << " ";
            if (i==(move_group.getJoints().size()-1)) {
                file << std::endl;
            }    
        }

        // acc
        for (uint i=0; i<move_group.getJoints().size(); i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].accelerations[i] << " ";
            if (i==(move_group.getJoints().size()-1)) {
                file << std::endl;
            }    
        }

        // time from start
        file << std::fixed << std::setprecision(16) << plan.trajectory_.joint_trajectory.points[j].time_from_start.sec + plan.trajectory_.joint_trajectory.points[j].time_from_start.nanosec/1e9 << std::endl;
    }

    file.close();
    return 0;
}

int logSummary(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    std::string& dir_name,
    rclcpp::Logger& LOGGER) 
{
    std::fstream file = open_file("summary.txt", dir_name, LOGGER);

    if (!file.is_open()) return -1;
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
    std::vector<double> joints_distance; joints_distance.resize(move_group.getJoints().size()); std::fill(joints_distance.begin(), joints_distance.end(), 0.0);
    std::vector<double> max_joint_vel; max_joint_vel.resize(move_group.getJoints().size()); std::fill(max_joint_vel.begin(), max_joint_vel.end(), 0.0);
    std::vector<double> max_joint_acc; max_joint_acc.resize(move_group.getJoints().size()); std::fill(max_joint_acc.begin(), max_joint_acc.end(), 0.0);
    
    std::vector<double> joints_pos_old = plan.trajectory_.joint_trajectory.points[0].positions;
    double joints_distance_sum;

    for (uint j=1; j<plan.trajectory_.joint_trajectory.points.size(); j++) {
        
        for (uint i=0; i<move_group.getJoints().size(); i++) {
            // joint distance
            joints_distance[i] += abs(plan.trajectory_.joint_trajectory.points[j].positions[i] - plan.trajectory_.joint_trajectory.points[j-1].positions[i]);

            // max vel, max acc
            if (abs(plan.trajectory_.joint_trajectory.points[j].velocities[i]) > max_joint_vel[i]) max_joint_vel[i] = abs(plan.trajectory_.joint_trajectory.points[j].velocities[i]);
            if (abs(plan.trajectory_.joint_trajectory.points[j].accelerations[i]) > max_joint_acc[i]) max_joint_acc[i] = abs(plan.trajectory_.joint_trajectory.points[j].accelerations[i]);
        }
    }

    // logging
    file << "Planning time [s]: " << std::fixed << std::setprecision(8) << plan.planning_time_ << std::endl;
    file << "Number of points [-]: " << plan.trajectory_.joint_trajectory.points.size() << std::endl;
    file << "Tool path distance [m]: " << std::fixed << std::setprecision(6) << path_distance << std::endl;
    
    file << "Joint distance [rad]" << std::endl;
    for (uint i=0; i<joints_distance.size(); i++) {
        file << "  - joint " << i+1 << " : " << std::fixed << std::setprecision(6) << joints_distance[i] << std::endl;
        joints_distance_sum += joints_distance[i];
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
    return 0;
}

void logAll(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const std::string planning_group,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string scene_dir_path,
    const std::string scene_dir_name,
    rclcpp::Logger& LOGGER) 
{
    // create subdir
    std::string dir_name = create_new_dir(scene_dir_name, scene_dir_path, LOGGER);

    // log to files
    logTool(move_group, planning_group, plan, dir_name, LOGGER);
    logTrajectory(move_group, planning_group, plan, dir_name, LOGGER);
    logSummary(move_group, planning_group, plan, dir_name, LOGGER);

}


}