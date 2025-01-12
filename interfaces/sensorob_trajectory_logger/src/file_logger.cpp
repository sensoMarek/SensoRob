
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
            return dir_name;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("trajectory_logger.file_logger"), "Failed to create directory %s", dir_name.c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("trajectory_logger.file_logger"),"Failed to open file %s", file_name.c_str());
    } 


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
    const std::string file_name,
    const std::string mode,
    rclcpp::Logger& LOGGER) 
{
    std::fstream file = open_file(file_name, dir_name, LOGGER);

    if (!file.is_open()) return -1;

    file << "joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 timestamp[ms]" << std::endl;

    for (uint j=0; j<plan.trajectory_.joint_trajectory.points.size(); j++) {

        // pos
        for (uint i=0; i<6; i++) {
            double data = 0;
            if (!mode.compare("position")) {
                data = plan.trajectory_.joint_trajectory.points[j].positions[i];
            } else if (!mode.compare("velocity")) {
                data = plan.trajectory_.joint_trajectory.points[j].velocities[i];   
            }
            file << std::fixed << std::setprecision(16)<< data << " ";
            if (i==(6-1)) {
                file << plan.trajectory_.joint_trajectory.points[j].time_from_start.sec*1000+ plan.trajectory_.joint_trajectory.points[j].time_from_start.nanosec/1e6 << std::endl;
            }    
        }
    }

    file.close();
    return 0;
}

void transformJointStatesToPose(
   const std::string home_dir_path,
   const std::string input_file_name,
   const std::string output_file_name,
   const std::string PLANNING_GROUP,
   const moveit::planning_interface::MoveGroupInterface& move_group
) {

  std::fstream input_file(home_dir_path+"/"+input_file_name, std::ios::in);
  if (!input_file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("trajectory_logger"), "Could not open input file");
      return;
  }

  std::fstream output_file(home_dir_path+"/"+output_file_name, std::ios::out);
  if (!output_file.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("trajectory_logger"), "Could not open output file");
      return;
  }

  // create robot state
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
  const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  std::string endEffectorLink = move_group.getEndEffectorLink();
  std::vector<double> loaded_vector;

  std::string line;
  std::getline(input_file, line);
  std::istringstream iss(line);
  std::vector<std::string> headers(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());

  // Reorder the headers
  std::vector<std::string> ordered_headers = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "timestamp[ms]"};
  // std::vector<std::string> new_header = {"X", "Y", "Z", "roll", "pitch", "yaw", "timestamp[ms]"};
  // for (const auto& header : ordered_headers) {
  //     output_file << header << " ";
  // }
  // output_file << std::endl;

  // Reorder the data and compute pose
  while (std::getline(input_file, line)) {
      std::istringstream iss(line);
      std::vector<double> data(std::istream_iterator<double>{iss}, std::istream_iterator<double>());
      for (const auto& header : ordered_headers) {
          auto it = std::find(headers.begin(), headers.end(), header);
          
          if (it != headers.end()) {
            int index = std::distance(headers.begin(), it);
            loaded_vector.push_back(data[index]); // joint states + timestamp
          }
      }


      std::vector<double> joint_states;
      for (auto it = loaded_vector.begin(); it != loaded_vector.end() - 1; ++it) {
        // Process the current element pointed to by the iterator (*it)
        joint_states.push_back(*it);
      }

      robot_state->setJointGroupActivePositions(joint_model_group, joint_states);
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

      // Log position and Euler angles of end effector
      output_file << std::fixed << std::setprecision(16) << end_effector_state.translation().x() << " "
        << std::fixed << std::setprecision(16) << end_effector_state.translation().y() << " "
        << std::fixed << std::setprecision(16) << end_effector_state.translation().z() << " "
        << std::fixed << std::setprecision(16) << roll << " "
        << std::fixed << std::setprecision(16) << pitch << " "
        << std::fixed << std::setprecision(16) << yaw << " ";
      output_file  << std::fixed << std::setprecision(0)<< loaded_vector.back() << std::endl;
      loaded_vector.clear(); 
  }
}

void computeError(
   const std::string home_dir_path,
   const std::string input_file_name1,
   const std::string input_file_name2
) {

  std::fstream file1(home_dir_path+"/"+input_file_name1, std::ios::in);
  if (!file1.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("trajectory_logger"), "Could not open file1");
      return;
  }

  std::fstream file2(home_dir_path+"/"+input_file_name2, std::ios::in);
  if (!file2.is_open()) {
      RCLCPP_ERROR(rclcpp::get_logger("trajectory_logger"), "Could not open file2");
      return;
  }

  RCLCPP_INFO(rclcpp::get_logger("trajectory_logger"), "Computing errors...");
  std::vector<double> errors = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<std::string> header = {"X [m]      ", "Y [m]      ", "Z [m]      ", "Roll [rad] ", "Pitch [rad]", "Yaw [rad]  ", "Timestamp [ms]"};
  std::vector<double> max_errors = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // max errors for each column

  std::string line1, line2;
  while (std::getline(file1, line1)) {
      std::istringstream iss1(line1);
      std::vector<double> data1(std::istream_iterator<double>{iss1}, std::istream_iterator<double>());

      double min_diff = std::numeric_limits<double>::max();
      std::vector<double> closest_data2;

      // Reset file2 to the beginning
      file2.clear();
      file2.seekg(0, std::ios::beg);

      while (std::getline(file2, line2)) {
          std::istringstream iss2(line2);
          std::vector<double> data2(std::istream_iterator<double>{iss2}, std::istream_iterator<double>());

          double diff = std::abs(data1.back() - data2.back());
          if (diff < min_diff) {
              min_diff = diff;
              closest_data2 = data2;
          }
      }

      // RCLCPP_INFO(rclcpp::get_logger("trajectory_logger"), "Closest point: %f", closest_data2.back());
      // Compute and print the absolute errors
      for (size_t i = 0; i < data1.size() - 1; ++i) {
        double error = std::pow((data1[i] - closest_data2[i]), 2);
        errors[i] += error;
        if (error > max_errors[i]) {
          max_errors[i] = error;
        }
      }
  }

  RCLCPP_INFO(rclcpp::get_logger("trajectory_logger"), "Computing errors... done"); 

  // Compute the root mean square error
  for (size_t i = 0; i < errors.size(); ++i) {
    errors[i] = std::sqrt(errors[i]);
    RCLCPP_INFO(rclcpp::get_logger("trajectory_logger"), "%s: root mean square error: %.4f, max error: %.4f", header[i].c_str(), errors[i], std::sqrt(max_errors[i]));
  }

}

void visualizeTrajectory(
   const std::string home_dir_path,
   const std::string input_file_name1,
   const std::string input_file_name2
) {
  RCLCPP_INFO(rclcpp::get_logger("trajectory_logger.trajectory_visualizator"), "Visualizing trajectory...");
  std::string current_dir_name(get_current_dir_name());
  current_dir_name += "/src/SensoRob/interfaces/sensorob_trajectory_logger/src/python_visualizer/";
  std::string command = "python3 " + current_dir_name + "trajectory_visualizer.py " + home_dir_path + " " + input_file_name1 + " " + input_file_name2;
  std::system(command.c_str());
}

void visualizeJointStates(
   const std::string home_dir_path,
   const std::string input_file_name1,
   const std::string input_file_name2,
   const std::string title
) {
  RCLCPP_INFO(rclcpp::get_logger("trajectory_logger.joint_states_visualizator"), "Visualizing joint states...");
  std::string current_dir_name(get_current_dir_name());
  current_dir_name += "/src/SensoRob/interfaces/sensorob_trajectory_logger/src/python_visualizer/";
  
  std::string command = "python3 " + current_dir_name + "joint_states_visualizer.py " + title + " " + home_dir_path + " " + input_file_name1 + " " + input_file_name2;
  std::system(command.c_str());
}


}