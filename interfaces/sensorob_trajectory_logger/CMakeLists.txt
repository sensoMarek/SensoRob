cmake_minimum_required(VERSION 3.8)
project(sensorob_trajectory_logger)

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
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  /opt/ros/humble/include/moveit_msgs
)

# trajectory_logger
add_executable(trajectory_logger
  src/trajectory_logger.cpp
  src/obstacles.cpp
  src/file_logger.cpp
)

target_include_directories(trajectory_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS trajectory_logger
  DESTINATION lib/${PROJECT_NAME})

target_compile_features(trajectory_logger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  "trajectory_logger"
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
  "yaml-cpp"
  "ament_index_cpp"
)



install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
