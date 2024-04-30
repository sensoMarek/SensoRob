//
// Created by jakub on 08.01.2024.
//
#include "sensorob_trajectory_logger/trajectory_logger.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("trajectory_logger", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(move_group.getRobotModel()));
    robot_state->setToDefaultValues();

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "world",
                                                        "ik_valid_points",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    

    //  constraints
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group.getEndEffectorLink();
    ocm.header.frame_id = move_group.getPoseReferenceFrame();
    ocm.orientation.w =  sqrt(2)/2.0;
    ocm.orientation.x = -sqrt(2)/2.0;
    ocm.orientation.y = 0;
    ocm.orientation.z = 0;
    // ocm.orientation = current_pose.pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.4;
    ocm.absolute_y_axis_tolerance = 0.4;
    ocm.absolute_z_axis_tolerance = 1.5; 
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.emplace_back(ocm);

    // TODO args func
    process_launch_args(
        move_group_node, 
        LOGGER, 
        num_rerun,
        planning_mode, 
        file_logging_mode,
        planner_id,
        planning_pipeline_id
    );
    move_group.setPlannerId(planner_id);
    move_group.setPlanningPipelineId(planning_pipeline_id);

    move_group.setPlanningTime(30); //seconds
    clog("Planning time: " + std::to_string(move_group.getPlanningTime()), LOGGER);

    clog("End effector link: " + move_group.getEndEffectorLink(), LOGGER);

    

    // logging
    std::string home_dir_path;
    if (file_logging_mode != FileLogging::NO_LOGGING) {
        clog("Creating subdirectories in 'src/SensoRob/sensorob_logs'", LOGGER);
        std::string current_dir_name(get_current_dir_name());
        const std::string main_dir_name = file_logger::create_new_dir("src/SensoRob/sensorob_logs", current_dir_name, LOGGER);
        home_dir_path = file_logger::create_new_dir("log_"+file_logger::get_current_time()+"_"+planner_id, main_dir_name, LOGGER);
    } 

    // Start the demo
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to start");
    // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // scene 1

    result = obstacles::create_environment(move_group.getPlanningFrame(), objects);
    if (result) clog("Collision objects not created successfully", LOGGER, WARN);

    // add obstacles
    addObjectsToScene(planning_scene, objects, environment_object_ids);
    sleep(1);
    objects.clear(); 

    result = obstacles::create_scene_1(move_group.getPlanningFrame(), objects);
    if (result) clog("Collision object not created successfully", LOGGER, WARN);
    // add obstacles
    addObjectsToScene(planning_scene, objects, object_ids);

    robot_state->setJointGroupPositions("sensorob_group", jointValueTargetA);
    move_group.setStartState(*robot_state);
    move_group.setJointValueTarget(jointValueTargetB);

    if ((planning_mode == PlanningMode::NC_PLANNING) || (planning_mode==PlanningMode::C_PLANNING)) {
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' to plan");

        plan_cycle(move_group, visual_tools, home_dir_path, "scene_1");
    }

    if (planning_mode==PlanningMode::C_PLANNING) {
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' to plan under constraints"); 

        // add constraints
        move_group.setPathConstraints(test_constraints);

        plan_cycle(move_group, visual_tools, home_dir_path, "scene_1_constrainted");

        // remove constraints
        move_group.clearPathConstraints();
        move_group.clearTrajectoryConstraints();
    }

    // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // scene 3

    visual_tools.prompt("Press 'next' to add all obstacles"); 
    visual_tools.trigger();
    visual_tools.deleteAllMarkers();

    planning_scene.removeCollisionObjects(object_ids);
    object_ids.clear(); objects.clear();

    result = obstacles::create_scene_2(move_group.getPlanningFrame(), objects);
    if (result) clog("Collision object not created successfully", LOGGER, WARN);
    // add obstacles
    addObjectsToScene(planning_scene, objects, object_ids);

    robot_state->setJointGroupPositions("sensorob_group", jointValueTarget1);
    move_group.setStartState(*robot_state);
    move_group.setJointValueTarget(jointValueTarget2);

    if ((planning_mode == PlanningMode::NC_PLANNING) || (planning_mode==PlanningMode::C_PLANNING)) {
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' to plan");

        plan_cycle(move_group, visual_tools, home_dir_path, "scene_2");
    }

    // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // 

    visual_tools.prompt("Press 'next' to remove obstacles"); 
    visual_tools.trigger();
    visual_tools.deleteAllMarkers();

    // remove obstacles
    planning_scene.removeCollisionObjects(object_ids);
    sleep(1);
    planning_scene.removeCollisionObjects(environment_object_ids);
    clog(std::to_string(object_ids.size()+environment_object_ids.size()) + " objects were removed", LOGGER);
    object_ids.clear(); objects.clear();

    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    rclcpp::shutdown();
    return 0;
}


void addObjectsToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene, std::vector<moveit_msgs::msg::CollisionObject>& objects, std::vector<std::string>& object_ids) {

    clog("Adding " + std::to_string(objects.size()) + " objects to planning scene:", LOGGER);

    std::string my_str;
    
    for (auto object : objects) {
        std::vector<moveit_msgs::msg::CollisionObject> vector_col;
        vector_col.push_back(object);
        planning_scene.addCollisionObjects(vector_col);

        my_str += object.id + " ";
        object_ids.push_back(object.id);
        sleep(1);       
    }
    
    clog("(" + my_str + ")", LOGGER);
}


void plan_cycle(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    moveit_visual_tools::MoveItVisualTools visual_tools, 
    const std::string home_dir_path, 
    const std::string dir_name) 
{

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    std::vector<file_logger::trajectory_attributes> traj_attributes_vector;
    std::string scene_dir_path;
    if (file_logging_mode != FileLogging::NO_LOGGING) {
        scene_dir_path = file_logger::create_new_dir(dir_name, home_dir_path, LOGGER);
    }

    for (uint iter=0; iter<num_rerun; iter++) {
        clog("Planning iteration: " + std::to_string(iter+1), LOGGER);
        moveit::core::MoveItErrorCode success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success == moveit::core::MoveItErrorCode::SUCCESS) {
            // clog("Planning successful", LOGGER);
            file_logger::trajectory_attributes traj_attributes = file_logger::logAll(move_group, PLANNING_GROUP, my_plan, scene_dir_path, std::to_string(iter+1), file_logging_mode==FileLogging::FULL_LOGGING, LOGGER); 
            
            traj_attributes_vector.push_back(traj_attributes);

            if (file_logging_mode==FileLogging::NO_LOGGING) clog(file_logger::trajectory_attributes_to_string(traj_attributes), LOGGER);  // if file logging is not allowed, print to console
            visual_tools.publishTrajectoryLine(my_plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP));
            visual_tools.trigger();

        } 
        // else 
        // {
        //     clog("Planning not successful", LOGGER, WARN);
        // }
        
    }

    file_logger::log_struct(traj_attributes_vector, num_rerun, file_logging_mode!=FileLogging::NO_LOGGING, LOGGER);
    file_logger::log_vectors(traj_attributes_vector, num_rerun, file_logging_mode!=FileLogging::NO_LOGGING, LOGGER);

    clog("Planning statistics:\n  - success: " + std::to_string(traj_attributes_vector.size()) + "\n  - failure: " + std::to_string(num_rerun - traj_attributes_vector.size()), LOGGER);
}
