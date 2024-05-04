//
// Created by jakub on 08.01.2024.
//
#include "sensorob_trajectory_logger/trajectory_logger.h"

class JointStateListener
{
public:
  rclcpp::Node::SharedPtr node_;

  JointStateListener()
  {
    node_ = rclcpp::Node::make_shared("joint_state_listener");
    subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&JointStateListener::jointStateListenerCallback, this, std::placeholders::_1));

    subscription_hz_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&JointStateListener::frequencyCallback, this, std::placeholders::_1));
    
    
  }

  ~JointStateListener()
  {
    if (file.is_open())
    {
      file.close();
    }
  }

  void setLogJointStates(bool log)
  {
    log_joint_states = log;
  }

  void openFile(std::string path, std::string file_name)
  {
    home_dir_path = path;
    file = file_logger::open_file(file_name, home_dir_path, LOGGER);
  }

  uint getNumberOfLoggedJointStates()
  {
    return joint_states_counter / frequency_ratio;
  }

  bool isFrequencySet()
  {
    return frequency != 0.0;
  }

  void setDesiredFrequency(double freq)
  {
    frequency_ratio = std::round(frequency/freq);
    if (frequency_ratio == 0) frequency_ratio = 1;
     
     RCLCPP_INFO(rclcpp::get_logger("joint_state_listener"), "Actual frequency: %.1f, desired frequency: %.1f, frequency_ratio: %d", frequency, freq, frequency_ratio);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_hz_;
  sensor_msgs::msg::JointState actual_joint_states;
  bool log_joint_states = false;
  std::string home_dir_path;
  std::fstream file;
  bool header_written = false;
  uint joint_states_counter = 0;
  std::chrono::steady_clock::time_point start_time;
  bool start_time_set = false;
  double frequency = 0.0;
  uint frequency_ratio = 0;


  void frequencyCallback([[maybe_unused]]const sensor_msgs::msg::JointState::SharedPtr msg)
  {     
    // Set start time if not set
    if (!start_time_set) {
      start_time =  std::chrono::steady_clock::now();
      start_time_set = true;
    }

    // Compute duration
    if (joint_states_counter >= 400) {
      auto now = std::chrono::steady_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
      RCLCPP_INFO(rclcpp::get_logger("joint_state_listener"), "Number of received joint states: %d,  duration: %ld ms", joint_states_counter, duration);
      frequency = joint_states_counter / (double)duration * 1000.0;
      RCLCPP_INFO(rclcpp::get_logger("joint_state_listener"), "Frequency: %f", frequency);
      start_time_set = false;
      joint_states_counter = 0;
      subscription_hz_.reset();
      return;
    }
  
    joint_states_counter++;
    
  }

  void jointStateListenerCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!log_joint_states) {
    return;
    }

    // count number of written joint states
    joint_states_counter++;

    // Log joint states only every frequency_ratio-th message
    if (joint_states_counter % frequency_ratio != 0) {
      return;
    } 

    // Set start time if not set
    if (!start_time_set) {
      start_time =  std::chrono::steady_clock::now();
      start_time_set = true;
    }

    actual_joint_states = *msg;



    // Compute duration
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    // Write header to file
    if (!header_written && file.is_open()) {
        for (const auto& state : actual_joint_states.name) {
            file << state << " ";
        }
        file << "timestamp[ms]" << std::endl;
        header_written = true;
    }

    // Log joint states to file here
    std::stringstream ss;
    for (const auto& state : actual_joint_states.name) {
      auto it = std::find(actual_joint_states.name.begin(), actual_joint_states.name.end(), state);
      if (it != actual_joint_states.name.end()) {
        int index = std::distance(actual_joint_states.name.begin(), it);
        ss << std::fixed << std::setprecision(16) << actual_joint_states.position[index] << " ";
      }
    }

    // Add duration to a stringstream
    ss << duration << std::endl; 
    
    if (file.is_open()) {
        file << ss.str();
    }
  }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("trajectory_logger", node_options);

    JointStateListener joint_state_listener;
    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(joint_state_listener.node_);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Load the configuration data from config file
    loadConfigData();

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
    robot_state->setToDefaultValues();

    move_group.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor);
    move_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world",
                                                        "ik_valid_points",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    move_group.setPlannerId(planner_id);
    move_group.setPlanningPipelineId(planning_pipeline_id);

    while (!joint_state_listener.isFrequencySet()) {
        RCLCPP_INFO(rclcpp::get_logger("joint_state_listener"), "Waiting for frequency to be set...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    joint_state_listener.setDesiredFrequency(desired_frequency);


    // Start the demo
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to start");

    // logging
    std::string home_dir_path;
    std::string current_dir_name(get_current_dir_name());
    const std::string main_dir_name = file_logger::create_new_dir("src/SensoRob/sensorob_logs", current_dir_name, LOGGER);
    home_dir_path = file_logger::create_new_dir("trajectory_log_"+file_logger::get_current_time(), main_dir_name, LOGGER);
    joint_state_listener.openFile(home_dir_path, "executed_joint_states.txt");

    // Add objects to the scene
    if (!obstacles::create_environment(move_group.getPlanningFrame(), objects)) {
        RCLCPP_INFO(LOGGER, "Environment created successfully");
    } else {
        RCLCPP_ERROR(LOGGER, "Error creating environment");
        rclcpp::shutdown();
        return -1;
    }

    for (auto object : objects) {
      std::vector<moveit_msgs::msg::CollisionObject> vector_col;
      vector_col.push_back(object);
      planning_scene.addCollisionObjects(vector_col);
      environment_object_ids.push_back(object.id);
      sleep(1);       
    }
    sleep(1);

    // Move the robot to the start state
    moveRobotToStartState(move_group, jointValueTarget1);

    if (movement_mode == MovementMode::JOINT_SPACE) { // Joint space planning
      // Set the joint target
      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(jointValueTarget2);

      // Plan the motion
      success = move_group.plan(my_plan);

    } else // Cartesian space planning
    {
      robot_state->setJointGroupPositions("sensorob_group", jointValueTarget1);
      move_group.setStartState(*robot_state);
      const Eigen::Affine3d &end_effector_state = robot_state->getGlobalLinkTransform(move_group.getEndEffectorLink());

      std::vector<geometry_msgs::msg::Pose> wps;

      createPathFromWaypoints(wps, end_effector_state);
      RCLCPP_INFO(LOGGER, "Created path from waypoints");

      const double jump_threshold = 0.0;
      const double eef_step = 0.001;
      bool avoid_collisions = false;
      double fraction = move_group.computeCartesianPath(wps, eef_step, jump_threshold, trajectory, avoid_collisions);
      if (fraction == 1.0) {
          // RCLCPP_INFO(LOGGER, "Path computed successfully. Moving the robot.");
          success = moveit::core::MoveItErrorCode::SUCCESS;
      } else {
          RCLCPP_ERROR(LOGGER, "Path computation failed with fraction: %.2f", fraction);
          rclcpp::shutdown();
          return -1;
      }

      RCLCPP_INFO(LOGGER, "Cartesian path -  %.2f%% achieved", fraction * 100.0);
    }
    
  
    if (success==moveit::core::MoveItErrorCode::SUCCESS) {
        // Set flag to start logging before execution
        joint_state_listener.setLogJointStates(true);

        // Execute the planned trajectory
        if (movement_mode == MovementMode::JOINT_SPACE) {
          move_group.execute(my_plan);  
        } else {
          move_group.execute(trajectory);
          my_plan.trajectory_ = trajectory;  
        }

        // Add a delay before stopping logging
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        // Reset flag to stop logging after execution
        joint_state_listener.setLogJointStates(false); 

        RCLCPP_INFO(rclcpp::get_logger("joint_state_listener"), "Number of joint states logged: %d", joint_state_listener.getNumberOfLoggedJointStates());
        file_logger::logTrajectory(move_group, PLANNING_GROUP, my_plan, home_dir_path, "planned_joint_states.txt", LOGGER);
        // joint_state_thread.join();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger(""), "Planning failed!");
        rclcpp::shutdown();
        return -1;
    }

    // transformJointStatesToPose();
    file_logger::transformJointStatesToPose(home_dir_path, "planned_joint_states.txt", "planned_poses.txt", PLANNING_GROUP, move_group);
    file_logger::transformJointStatesToPose(home_dir_path, "executed_joint_states.txt", "executed_poses.txt", PLANNING_GROUP, move_group);

    // compute the error in eef pose
    file_logger::computeError(home_dir_path, "planned_poses.txt", "executed_poses.txt");

    // visualize the trajectory
    file_logger::visualizeTrajectory(home_dir_path, "planned_poses.txt", "executed_poses.txt");


    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");
    planning_scene.removeCollisionObjects(environment_object_ids);
      RCLCPP_INFO(LOGGER, "Environment cleared successfully");


    rclcpp::shutdown();
    return 0;
}

void createPathFromWaypoints(std::vector<geometry_msgs::msg::Pose>& wps, const Eigen::Affine3d &end_effector_state) {

  Eigen::Quaterniond quaternion(end_effector_state.rotation());

  geometry_msgs::msg::Pose pose1;
  pose1.position.x =  end_effector_state.translation().x();
  pose1.position.y =  end_effector_state.translation().y();
  pose1.position.z =  end_effector_state.translation().z();
  pose1.orientation.x = quaternion.x();
  pose1.orientation.y = quaternion.y();
  pose1.orientation.z = quaternion.z();
  pose1.orientation.w = quaternion.w();

  wps.push_back(pose1);
  geometry_msgs::msg::Pose pose = pose1;

  for (const auto& wp : waypoints) {
      pose.position.x += wp.coordinates[0];
      pose.position.y += wp.coordinates[1];
      pose.position.z += wp.coordinates[2];
      wps.push_back(pose);
  }

}

int moveRobotToStartState(moveit::planning_interface::MoveGroupInterface& move_group, std::vector<double> jointValueTarget) {
    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(jointValueTarget);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
    if (success != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
        rclcpp::shutdown();
        return -1;
    }
    move_group.execute(my_plan);
    return 0;
}

int loadConfigData() {

  rclcpp::Logger LOADER_LOGGER = rclcpp::get_logger(".config_loader");
  // Load the joint values from the config file
  std::string config_file_path = ament_index_cpp::get_package_share_directory("sensorob_trajectory_logger") + "/config/config.yaml";
  YAML::Node config = YAML::LoadFile(config_file_path);

    // movement mode
  if (!config["movement_mode"]) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'movement_mode' not found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }
  movement_mode = config["movement_mode"].as<int>();
  RCLCPP_INFO(LOADER_LOGGER, "Loaded movement_mode: %d", movement_mode);

  // desired frequency
  if (!config["desired_frequency"]) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'desired_frequency' not found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }
  desired_frequency = config["desired_frequency"].as<int>();
  RCLCPP_INFO(LOADER_LOGGER, "Loaded desired_frequency: %d", desired_frequency);



  // planner id
  if (!config["planner_id"]) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'planner_id' not found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }
  std::string planner_id_ = config["planner_id"].as<std::string>();
  RCLCPP_INFO(LOADER_LOGGER, "Loaded planner_id: %s", planner_id_.c_str());

  // check if the planner id is valid
  std::vector<std::string> ompl_planner_ids = {"RRTConnect", "RRT", "RRTstar", "TRRT", "EST", "LBTRRT", "BiEST", "STRIDE", "BiTRRT", "PRM", "PRMstar", "LazyPRMstar", "PDST", "STRIDE", "BiEST", "STRIDE", "BiTRRT"};
  std::vector<std::string> stomp_planner_ids = {"STOMP"};
  std::vector<std::string> chomp_planner_ids = {"CHOMP"};
  std::vector<std::string> pilz_planner_ids = {"PTP", "CIRC", "LIN"};

  if (std::find(ompl_planner_ids.begin(), ompl_planner_ids.end(), planner_id_) != ompl_planner_ids.end()) {
      planner_id = planner_id_;
      planning_pipeline_id = "";  // "ompl"; does not work
  } else if (std::find(stomp_planner_ids.begin(), stomp_planner_ids.end(), planner_id_) != stomp_planner_ids.end()) {
      planner_id = planner_id_;
      planning_pipeline_id = "stomp";
  } else if (std::find(chomp_planner_ids.begin(), chomp_planner_ids.end(), planner_id_) != chomp_planner_ids.end()) {
      planner_id = planner_id_;
      planning_pipeline_id = "chomp";
  } else if (std::find(pilz_planner_ids.begin(), pilz_planner_ids.end(), planner_id_) != pilz_planner_ids.end()) {
      planner_id = planner_id_;
      planning_pipeline_id = "pilz";
  } else {
      planner_id = "RRTConnect";
      planning_pipeline_id = "ompl";
      RCLCPP_WARN(LOADER_LOGGER, "Cannot find planning configuration for group 'sensorob_group' using planner %s . Will use defaults instead.", planner_id.c_str());
  }

  std::string pp_id = !planning_pipeline_id.compare("")?"ompl":planning_pipeline_id;
  RCLCPP_INFO(LOADER_LOGGER, "Planner id: %s, planning pipeline id: %s", planner_id.c_str(), pp_id.c_str());


  // max velocity scaling factor
  if (!config["max_velocity_scaling_factor"]) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'max_velocity_scaling_factor' not found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }
  max_velocity_scaling_factor = config["max_velocity_scaling_factor"].as<double>();
  RCLCPP_INFO(LOADER_LOGGER, "Loaded max_velocity_scaling_factor: %.2f", max_velocity_scaling_factor);

  // max acceleration scaling factor
  if (!config["max_acceleration_scaling_factor"]) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'max_acceleration_scaling_factor' not found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }
  max_acceleration_scaling_factor = config["max_acceleration_scaling_factor"].as<double>();
  RCLCPP_INFO(LOADER_LOGGER, "Loaded max_acceleration_scaling_factor: %.2f", max_acceleration_scaling_factor);

  // joint value target 1
  if (!config["joint_value_target_1"]) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'joint_value_target_1' not found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }
  jointValueTarget1 = config["joint_value_target_1"].as<std::vector<double>>();

  if (jointValueTarget1.size() != 6) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'joint_value_target_1' should contain 6 values");
    rclcpp::shutdown();
    return -1; 
  }

  std::stringstream ss; ss << "Loaded joint value target 1: ";
  for (const auto& value : jointValueTarget1) {
    ss << value << " ";
  }
  RCLCPP_INFO_STREAM(LOADER_LOGGER, ss.str());

  // joint value target 2
  if (!config["joint_value_target_2"]) {
    if (movement_mode == MovementMode::JOINT_SPACE) {
      RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'joint_value_target_2' not found in config.yaml");
      rclcpp::shutdown();
      return -1;
    } else {
      RCLCPP_WARN(LOADER_LOGGER, "No joint value target 2 found in config.yaml.");
      return 0;
    }
  }

  jointValueTarget2 = config["joint_value_target_2"].as<std::vector<double>>();
  
  if (jointValueTarget2.size() != 6) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'joint_value_target_2' should contain 6 values");
    rclcpp::shutdown();
    return -1; 
  }

  std::stringstream ss2; ss2 << "Loaded joint value target 2: ";
  for (const auto& value : jointValueTarget2) {
    ss2 << value << " ";
  }
  RCLCPP_INFO_STREAM(LOADER_LOGGER, ss2.str());

  // Load the waypoints from the config file
  if (!config["waypoints"]) {
    
    if (movement_mode == MovementMode::CARTESIAN_SPACE) {
      RCLCPP_ERROR(LOADER_LOGGER,"Error: Key 'waypoints' not found in config.yaml");
      rclcpp::shutdown();
      return -1; 
    } else {
      RCLCPP_WARN(LOADER_LOGGER, "No waypoints found in config.yaml.");
      return 0;
    }
  }
    // Access the "waypoints" node
  const YAML::Node& waypoint_list = config["waypoints"];

  // Loop through each waypoint in the list
  int iter = 0;
  std::string wp_prefix = "wp";
  for (const auto& waypoint_node : waypoint_list) {
    // Create a vector to store the current waypoint's coordinates
    Waypoint current_waypoint;

    // Access the list of coordinates for the current waypoint
    const YAML::Node& coordinates_node = waypoint_node;
    // Check if the current waypoint contains exactly 3 coordinates
    if (coordinates_node.size() != 3) {
      RCLCPP_ERROR(LOADER_LOGGER,"Error: Each waypoint should contain 3 coordinates");
      rclcpp::shutdown();
      return -1; 
    }

    // Loop through each coordinate and add it to the current_waypoint vector
    for (const auto& coordinate : coordinates_node) {
      current_waypoint.coordinates.push_back(coordinate.as<double>());
    }

    current_waypoint.name = wp_prefix + std::to_string(iter++);

    // Add the current waypoint (inner vector) to the waypoints (outer vector)
    waypoints.push_back(current_waypoint);
  }

  if (waypoint_list.size() == 0) {
    RCLCPP_ERROR(LOADER_LOGGER,"Error: No waypoints found in config.yaml");
    rclcpp::shutdown();
    return -1; 
  }

  // Print the loaded waypoints (optional)
  RCLCPP_INFO(LOADER_LOGGER, "Loaded waypoints:");
  for (size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(LOADER_LOGGER, "%s: ", waypoints[i].name.c_str());
    for (size_t j = 0; j < waypoints[i].coordinates.size(); ++j) {
      RCLCPP_INFO(LOADER_LOGGER, "  %.2f", waypoints[i].coordinates[j]);
    }
  }

  return 0;
  }


