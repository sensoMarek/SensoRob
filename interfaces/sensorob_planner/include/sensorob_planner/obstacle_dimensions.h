/*Jakub Ivan BP 2022*/

#ifndef SENSOROB_PLANNER_GLOBAL_DIMENSIONS_H
#define SENSOROB_PLANNER_GLOBAL_DIMENSIONS_H


//table
static const double table_size[3] = {1.4, 1.2, 0.05};
static const double table_position[3] = {
    0,
    0.35,
    0};

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
    0.70, 
    obstacle3_size[2]/2 + 0.40
};



#endif //SENSOROB_PLANNER_GLOBAL_DIMENSIONS_H
