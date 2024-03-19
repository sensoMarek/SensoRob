/*Jakub Ivan BP 2022*/

#ifndef SENSOROB_PLANNER_GLOBAL_DIMENSIONS_H
#define SENSOROB_PLANNER_GLOBAL_DIMENSIONS_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

namespace obstacles {

int create_environment(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects);
int create_scene_1(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects);
int create_scene_2(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects);


//table
static const double pedestal_size[3] = {1.4, 1.2, 0.05};
static const double pedestal_position[3] = {
	0,
	0.35,
	0
};

//obstacle_1
static const double obstacle1_size[3] = {0.4, 0.4, 0.8};
static const double obstacle1_position[3] = {
	obstacle1_size[0] + 0.05, 
	obstacle1_size[1] + 0.30, 
	obstacle1_size[2]/2.0
};

//obstacle_2
static const double obstacle2_size[3] = {0.4, 0.4, 0.8};
static const double obstacle2_position[3] = {
	- obstacle2_size[0] - 0.05, 
	obstacle2_size[1] + 0.30, 
	obstacle2_size[2]/2
};


//obstacle_3
static const double obstacle3_size[3] = {1.40, 0.20, 0.05};
static const double obstacle3_position[3] = {
	0, 
	0.80, 
	obstacle3_size[2]/2 + 0.40
};

}
#endif //SENSOROB_PLANNER_GLOBAL_DIMENSIONS_H
