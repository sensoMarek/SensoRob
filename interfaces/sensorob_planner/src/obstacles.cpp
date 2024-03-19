#include "sensorob_planner/obstacles.h"

namespace obstacles {

int create_environment(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& objects) {
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////// pedestal
	moveit_msgs::msg::CollisionObject pedestal;
	pedestal.header.frame_id = planning_frame;
	pedestal.id = "pedestal";
	shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = pedestal_size[0];
	primitive.dimensions[1] = pedestal_size[1];
	primitive.dimensions[2] = pedestal_size[2];
	geometry_msgs::msg::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = pedestal_position[0];
	box_pose.position.y = pedestal_position[1];
	box_pose.position.z = pedestal_position[2] - (primitive.dimensions[2])/2;
	pedestal.primitives.push_back(primitive);
	pedestal.primitive_poses.push_back(box_pose);
	pedestal.operation = pedestal.ADD;

	objects.push_back(pedestal);
	return 0;
}

int create_scene_1(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& scene_1) {
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// obstacle 1
	moveit_msgs::msg::CollisionObject obstacle_1;
	obstacle_1.header.frame_id = planning_frame;
	obstacle_1.id = "obstacle_1";
	shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = obstacle1_size[0];
	primitive.dimensions[1] = obstacle1_size[1];
	primitive.dimensions[2] = obstacle1_size[2];
	geometry_msgs::msg::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = obstacle1_position[0];
	box_pose.position.y = obstacle1_position[1];
	box_pose.position.z = obstacle1_position[2];
	obstacle_1.primitives.push_back(primitive);
	obstacle_1.primitive_poses.push_back(box_pose);
	obstacle_1.operation = obstacle_1.ADD;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// obstacle 2
	moveit_msgs::msg::CollisionObject obstacle_2;
	obstacle_2.header.frame_id = planning_frame;
	obstacle_2.id = "obstacle_2";
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = obstacle2_size[0];
	primitive.dimensions[1] = obstacle2_size[1];
	primitive.dimensions[2] = obstacle2_size[2];
	box_pose.orientation.w = 1.0;
	box_pose.position.x = obstacle2_position[0];
	box_pose.position.y = obstacle2_position[1];
	box_pose.position.z = obstacle2_position[2];
	obstacle_2.primitives.push_back(primitive);
	obstacle_2.primitive_poses.push_back(box_pose);
	obstacle_2.operation = obstacle_2.ADD;

	scene_1.push_back(obstacle_1); 
	scene_1.push_back(obstacle_2);
	return 0;

}

int create_scene_2(const std::string& planning_frame, std::vector<moveit_msgs::msg::CollisionObject>& scene_2) {
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// obstacle 2
	moveit_msgs::msg::CollisionObject obstacle_3;
	obstacle_3.header.frame_id = planning_frame;
	obstacle_3.id = "obstacle_3";
	shape_msgs::msg::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = obstacle3_size[0];
	primitive.dimensions[1] = obstacle3_size[1];
	primitive.dimensions[2] = obstacle3_size[2];
	geometry_msgs::msg::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = obstacle3_position[0];
	box_pose.position.y = obstacle3_position[1];
	box_pose.position.z = obstacle3_position[2];
	obstacle_3.primitives.push_back(primitive);
	obstacle_3.primitive_poses.push_back(box_pose);
	obstacle_3.operation = obstacle_3.ADD;

	scene_2.push_back(obstacle_3); 
	return 0;
}

}
