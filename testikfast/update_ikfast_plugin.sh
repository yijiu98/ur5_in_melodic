search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ur5.srdf
robot_name_in_srdf=ur5
moveit_config_pkg=ur5_moveit_config
robot_name=ur5
planning_group_name=manipulator
ikfast_plugin_pkg=testikfast
base_link_name=base_link
eef_link_name=ee_link
ikfast_output_path=/home/yijiu98/yijiu98/UR5/catkin_ws/src/testikfast/src/ur5_manipulator_ikfast_solver.cpp

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
