
#include "sensorob_trajectory_logger/file_logger.h"

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
        clog("File " + dir_name+"/"+file_name + " is not successfully opened, no logging!!", LOGGER, ERROR);
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
    std::fstream file = open_file("planned_joint_states.txt", dir_name, LOGGER);

    if (!file.is_open()) return -1;

    file << "joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 timestamp[ms]" << std::endl;

    for (uint j=0; j<plan.trajectory_.joint_trajectory.points.size(); j++) {

        // pos
        for (uint i=0; i<6; i++) {
            file << std::fixed << std::setprecision(16)<< plan.trajectory_.joint_trajectory.points[j].positions[i] << " ";
            if (i==(6-1)) {
                file << plan.trajectory_.joint_trajectory.points[j].time_from_start.sec + plan.trajectory_.joint_trajectory.points[j].time_from_start.nanosec/1e6 << std::endl;
            }    
        }
    }

    file.close();
    return 0;
}


}