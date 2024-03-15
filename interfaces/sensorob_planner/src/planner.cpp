//
// Created by jakub on 08.01.2024.
//
#include "sensorob_planner/planner.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planner");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("planner", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    static const std::string PLANNING_GROUP = "sensorob_group";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link",
                                                        "ik_valid_points",
                                                        move_group.getRobotModel());
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Start the demo
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to add obstacles");

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    std::vector<std::string> object_ids;
    std::vector<std::string> environment_object_ids;
    int result;

    // // // // // // // // // // // // scene 1
    result = create_environment(move_group.getPlanningFrame(), objects);
    if (result) clog("Collision objects not created successfully", LOGGER, WARN);
    // add obstacles
    addObjectsToScene(planning_scene, objects, environment_object_ids);
    sleep(1);
    objects.clear(); 
    result = create_scene_1(move_group.getPlanningFrame(), objects);
    if (result) clog("Collision objects not created successfully", LOGGER, WARN);
    // add obstacles
    addObjectsToScene(planning_scene, objects, object_ids);


    // // // // // // // // // // // // scene 2
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to change obstacles"); 

    // remove obstacles
    planning_scene.removeCollisionObjects(object_ids);
    sleep(1);
    clog(std::to_string(object_ids.size()) + " objects were removed", LOGGER);

    object_ids.clear(); objects.clear(); 
    result = create_scene_2(move_group.getPlanningFrame(), objects);
    if (result) clog("Collision object not created successfully", LOGGER, WARN);
    // add obstacles
    addObjectsToScene(planning_scene, objects, object_ids);

    // // // // // // // // // // // //
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to remove obstacles"); 

    // remove obstacles
    planning_scene.removeCollisionObjects(object_ids);
    sleep(1);
    planning_scene.removeCollisionObjects(environment_object_ids);
    clog(std::to_string(object_ids.size()+environment_object_ids.size()) + " objects were removed", LOGGER);
    object_ids.clear(); objects.clear();

    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish ");

    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

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
