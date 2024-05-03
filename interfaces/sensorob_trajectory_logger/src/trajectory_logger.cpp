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

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
    robot_state->setToDefaultValues();

    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setMaxVelocityScalingFactor(1.0);


    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world",
                                                        "ik_valid_points",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    process_launch_args(
        move_group_node, 
        LOGGER,
        desired_frequency,
        planner_id,
        planning_pipeline_id,
        mode
    );

    move_group.setPlannerId(planner_id);
    move_group.setPlanningPipelineId(planning_pipeline_id);

    while (!joint_state_listener.isFrequencySet()) {
        RCLCPP_INFO(rclcpp::get_logger("joint_state_listener"), "Waiting for frequency to be set...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    joint_state_listener.setDesiredFrequency(desired_frequency);
    

    // logging
    std::string home_dir_path;
    clog("Creating subdirectory in 'src/SensoRob/sensorob_logs'", LOGGER);
    std::string current_dir_name(get_current_dir_name());
    const std::string main_dir_name = file_logger::create_new_dir("src/SensoRob/sensorob_logs", current_dir_name, LOGGER);
    home_dir_path = file_logger::create_new_dir("trajectory_log_"+file_logger::get_current_time(), main_dir_name, LOGGER);
    joint_state_listener.openFile(home_dir_path, "executed_joint_states.txt");

    // Start the demo
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to start");


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////


    if (mode == 1) { // Joint space planning
      // Set the joint target
      moveRobotToStartState(move_group, jointValueTargetHome);
      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(jointValueTargetB);

      // Plan the motion
      success = move_group.plan(my_plan);

    } else // Cartesian space planning
    {
      moveRobotToStartState(move_group, jointValueTargetA);
      robot_state->setJointGroupPositions("sensorob_group", jointValueTargetA);
      move_group.setStartState(*robot_state);
      const Eigen::Affine3d &end_effector_state = robot_state->getGlobalLinkTransform(move_group.getEndEffectorLink());
      Eigen::Quaterniond quaternion(end_effector_state.rotation());

      geometry_msgs::msg::Pose pose1;
      pose1.position.x =  end_effector_state.translation().x();
      pose1.position.y =  end_effector_state.translation().y();
      pose1.position.z =  end_effector_state.translation().z();
      pose1.orientation.x = quaternion.x();
      pose1.orientation.y = quaternion.y();
      pose1.orientation.z = quaternion.z();
      pose1.orientation.w = quaternion.w();

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(pose1);

      pose1.position.x += 0.3;
      waypoints.push_back(pose1);  // when looking on robot to the left, the robot moves to the right

      pose1.position.z += 0.2;
      waypoints.push_back(pose1);  // the robot moves up
      
      const double jump_threshold = 0.0;
      const double eef_step = 0.001;
      bool avoid_collisions = false;
      double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, avoid_collisions);
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
        if (mode == 1) {
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

    rclcpp::shutdown();
    return 0;
}
