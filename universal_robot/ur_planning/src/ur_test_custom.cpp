// 包含miveit的API头文件
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur_moveit_custom_demo");
  ros::NodeHandle node_handle; 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("manipulator");

  // 设置机器人终端的目标位置
  geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 0.726282;
  // target_pose1.orientation.x= 4.04423e-07;
  // target_pose1.orientation.y = -0.687396;
  // target_pose1.orientation.z = 4.81813e-07;

  // target_pose1.position.x = 0.03;
  // target_pose1.position.y = 0.2;
  // target_pose1.position.z = 0.5;
  /*xyz方向不变*/
  // target_pose1.orientation.w = 1;
  // target_pose1.orientation.x= 0;
  // target_pose1.orientation.y = 0;
  // target_pose1.orientation.z = 0;

  // target_pose1.position.x = 0;
  // target_pose1.position.y = 0.2;
  // target_pose1.position.z = 1;
  /*绕x轴顺时针转90度*/
  target_pose1.orientation.w = std::sqrt(2) / 2;
  target_pose1.orientation.x = -std::sqrt(2) / 2;
  target_pose1.orientation.y = 0;
  target_pose1.orientation.z = 0;

  target_pose1.position.x = 0;
  target_pose1.position.y = 0.5;
  target_pose1.position.z = 0.5;

  group.setPoseTarget(target_pose1);

  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
  if (success == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO("Planning succeeded");
  } else if (success == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED) {
      ROS_WARN("Planning failed, possibly due to collisions or constraints.");
  } else if (success == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN) {
      ROS_WARN("The generated motion plan is invalid.");
  } else if (success == moveit::planning_interface::MoveItErrorCode::INVALID_GROUP_NAME) {
      ROS_WARN("Invalid planning group name.");
  } else if (success == moveit::planning_interface::MoveItErrorCode::START_STATE_IN_COLLISION) {
      ROS_WARN("The start state is in collision.");
  } else if (success == moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION) {
      ROS_WARN("The goal state is in collision.");
  } else {
      ROS_ERROR("Planning failed for an unknown reason.");
  }


  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

  //让机械臂按照规划的轨迹开始运动。
  if(success)
      group.execute(my_plan);

  ros::shutdown(); 
  return 0;
}
