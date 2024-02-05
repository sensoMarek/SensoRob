search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=sensorob.srdf
robot_name_in_srdf=sensorob
moveit_config_pkg=sensorob_moveit_config
robot_name=sensorob
planning_group_name=sensorob_group
ikfast_plugin_pkg=sensorob_sensorob_group_ikfast_plugin
base_link_name=base_link
eef_link_name=link_6
ikfast_output_path=/home/jakub/ros2_ws/src/SensoRob/sensorob_sensorob_group_ikfast_plugin/src/sensorob_sensorob_group_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
