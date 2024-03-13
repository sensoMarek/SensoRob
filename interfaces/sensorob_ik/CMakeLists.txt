cmake_minimum_required(VERSION 3.8)
project(sensorob_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
        include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

set( DEPENDENCIES
        "rclcpp"
        "moveit_core"
        "moveit_ros_planning_interface"
        "moveit_ros_planning"
        "moveit_msgs"
        "moveit_visual_tools"
        "rviz_visual_tools"
        "geometry_msgs"
        "diagnostic_msgs"
        "tf2_ros"
        "tf2_geometry_msgs"
)


# ik_interface
set (EXECUTABLE_1_NAME "ik_interface")
add_executable(${EXECUTABLE_1_NAME}
        src/ik_interface/ik_interface.cpp
        src/ik_interface/fk.cpp
        src/ik.cpp
        src/viz.cpp
        src/logger.cpp)
ament_target_dependencies(${EXECUTABLE_1_NAME} ${DEPENDENCIES})
install(TARGETS ${EXECUTABLE_1_NAME}
        DESTINATION lib/${PROJECT_NAME})
target_include_directories(${EXECUTABLE_1_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(${EXECUTABLE_1_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


# ik_interface_random
set (EXECUTABLE_2_NAME "ik_interface_random")
add_executable(${EXECUTABLE_2_NAME}
        src/ik_interface_random/ik_interface.cpp
        src/ik_interface_random/fk.cpp
        src/ik.cpp
        src/viz.cpp
        src/logger.cpp)
ament_target_dependencies(${EXECUTABLE_2_NAME} ${DEPENDENCIES})
install(TARGETS ${EXECUTABLE_2_NAME}
        DESTINATION lib/${PROJECT_NAME})
target_include_directories(${EXECUTABLE_2_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(${EXECUTABLE_2_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_package()
