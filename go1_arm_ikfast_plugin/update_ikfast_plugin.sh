search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=go1.srdf
robot_name_in_srdf=go1
moveit_config_pkg=go1_moveit_config
robot_name=go1
planning_group_name=arm
ikfast_plugin_pkg=go1_arm_ikfast_plugin
base_link_name=link1
eef_link_name=end_effector_link
ikfast_output_path=/mnt/d/Ezeuz/Workbench/Thesis/go1_noetic_robotis/src/go1_arm_ikfast_plugin/src/go1_arm_ikfast_solver.cpp

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
