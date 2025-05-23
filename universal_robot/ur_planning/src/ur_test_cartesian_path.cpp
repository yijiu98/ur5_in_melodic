#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ur_cartesian_path_demo", ros::init_options::AnonymousName);

	// 创建一个异步的自旋线程（spinning thread）
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface move_group("manipulator");

	// 设置一个目标位置（路点）
	geometry_msgs::Pose target_pose;
	// target_pose.orientation.w = 0.726282;
	// target_pose.orientation.x = 4.04423e-07;
	// target_pose.orientation.y = -0.687396;
	// target_pose.orientation.z = 4.81813e-07;
	// target_pose.position.x = 0.03;
	// target_pose.position.y = 0.2;
	// target_pose.position.z = 0.5;
	target_pose.orientation.w = 1;
	target_pose.orientation.x = 0;
	target_pose.orientation.y = 0;
	target_pose.orientation.z = 0;
	target_pose.position.x = 0;
	target_pose.position.y = 0.2;
	target_pose.position.z = 0.5;
	std::vector<geometry_msgs::Pose> waypoints;
	waypoints.push_back(target_pose);

	geometry_msgs::Pose target_pose2;
	target_pose2.orientation.w = 1;
	target_pose2.orientation.x = 0;
	target_pose2.orientation.y = 0;
	target_pose2.orientation.z = 0;
	target_pose2.position.x = 0;
	target_pose2.position.y = 0.2;
	target_pose2.position.z = 1;
	waypoints.push_back(target_pose2);
	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

	// 生成机械臂的运动规划数据
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	plan.trajectory_ = trajectory;

	// 执行运动
	move_group.execute(plan);

	ros::shutdown(); 
	return 0;
}
