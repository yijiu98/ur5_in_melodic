
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <string>
#include <iostream>
#include <tf/transform_datatypes.h>

using namespace std;


class MoveIt_Control
{
public:
	
	MoveIt_Control(const ros::NodeHandle &nh,moveit::planning_interface::MoveGroupInterface &arm,const string &PLANNING_GROUP) {
		
		this->arm_ = &arm;
		this->nh_ =nh;
		


		arm_->setGoalPositionTolerance(0.001);
		arm_->setGoalOrientationTolerance(0.01);
		arm_->setGoalJointTolerance(0.001);

		arm_->setMaxAccelerationScalingFactor(0.5);
		arm_->setMaxVelocityScalingFactor(0.5);

		const moveit::core::JointModelGroup* joint_model_group =
			arm_->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

		this->end_effector_link = arm_->getEndEffectorLink();

		
		this->reference_frame = "base";
		arm_->setPoseReferenceFrame(reference_frame);
		
		arm_->allowReplanning(true);
		arm_->setPlanningTime(5.0);
		arm_->setPlannerId("TRRT");

		
		go_home();

		
		create_table();

	}	
	void go_home() {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		arm_->setNamedTarget("up");
		arm_->move();
		sleep(0.5);
	}

	bool move_j(const vector<double> &joint_group_positions) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		arm_->setJointValueTarget(joint_group_positions);
		arm_->move();
		sleep(0.5);
		return true;
	}

	bool move_p(const vector<double> &pose) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		// const std::string reference_frame = "base";
		// arm.setPoseReferenceFrame(reference_frame);

		
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();

		
		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);

		ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

		
		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_p_with_constrains(const vector<double>& pose) {

		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		// const std::string reference_frame = "base";
		// arm.setPoseReferenceFrame(reference_frame);
		// arm.setPlannerId("TRRT");
		arm_->setMaxAccelerationScalingFactor(0.5);
		arm_->setMaxVelocityScalingFactor(0.5);

		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();

		// geometry_msgs::PoseStamped current_pose_modified = arm.getCurrentPose(this->end_effector_link);
		// current_pose_modified.header.frame_id = "base";
		// current_pose_modified.pose.position.x = -current_pose_modified.pose.position.x ;
		// current_pose_modified.pose.position.y = -current_pose_modified.pose.position.y ;
		// current_pose_modified.pose.orientation.x = myQuaternion.getX();
		// current_pose_modified.pose.orientation.y = myQuaternion.getY();
		// current_pose_modified.pose.orientation.z = myQuaternion.getZ();
		// current_pose_modified.pose.orientation.w = myQuaternion.getW();
		// arm.setPoseTarget(current_pose_modified.pose);arm.move();
		

		//set constraint 
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = "base";
		ocm.header.frame_id = "ee_link";
		ocm.orientation.x = myQuaternion.getX();
		ocm.orientation.y = myQuaternion.getY();
		ocm.orientation.z = myQuaternion.getZ();
		ocm.orientation.w = myQuaternion.getW();
		ocm.absolute_x_axis_tolerance = 0.1;
		ocm.absolute_y_axis_tolerance = 0.1;
		ocm.absolute_z_axis_tolerance = 0.1;
		ocm.weight = 1.0;

		// Now, set it as the path constraint for the group.
		moveit_msgs::Constraints test_constraints;
		test_constraints.orientation_constraints.push_back(ocm);
		arm_->setPathConstraints(test_constraints);

		/*moveit::core::RobotState start_state(*move_group_interface.getCurrentState());
		geometry_msgs::Pose start_pose2;
		start_pose2.orientation.w = 1.0;
		start_pose2.position.x = 0.55;
		start_pose2.position.y = -0.05;
		start_pose2.position.z = 0.8;
		start_state.setFromIK(joint_model_group, start_pose2);
		move_group_interface.setStartState(start_state);*/

		// Now we will plan to the earlier pose target from the new
		// start state that we have just created.
		arm_->setStartStateToCurrentState();
		arm_->setPoseTarget(target_pose);

		// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
		// Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
		arm_->setPlanningTime(10.0);

		moveit::planning_interface::MoveGroupInterface::Plan plan;
		moveit::planning_interface::MoveItErrorCode success = arm_->plan(plan);

		ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

		arm_->setPlanningTime(5.0);
		arm_->clearPathConstraints();
		if (success) {
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		return false;
	}

	bool move_l(const vector<double>& pose) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");

		vector<geometry_msgs::Pose> waypoints;
		geometry_msgs::Pose target_pose;
		target_pose.position.x = pose[0];
		target_pose.position.y = pose[1];
		target_pose.position.z = pose[2];

		
		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(pose[3], pose[4], pose[5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();
		waypoints.push_back(target_pose);

		
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;

			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}
	
	bool move_l(const vector<vector<double>>& posees) {
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");
		vector<geometry_msgs::Pose> waypoints;
		for (int i = 0; i < posees.size(); i++) {
            geometry_msgs::Pose target_pose;
			target_pose.position.x = posees[i][0];
			target_pose.position.y = posees[i][1];
			target_pose.position.z = posees[i][2];

			
			tf2::Quaternion myQuaternion;
			myQuaternion.setRPY(posees[i][3], posees[i][4], posees[i][5]);
			target_pose.orientation.x = myQuaternion.getX();
			target_pose.orientation.y = myQuaternion.getY();
			target_pose.orientation.z = myQuaternion.getZ();
			target_pose.orientation.w = myQuaternion.getW();
			waypoints.push_back(target_pose);
		}

		
		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 100;   
		int attempts = 0;     

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");

			
			moveit::planning_interface::MoveGroupInterface::Plan plan;
			plan.trajectory_ = trajectory;

			
			arm_->execute(plan);
			sleep(1);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}


	void create_table() {
		
		// Now let's define a collision object ROS message for the robot to avoid.

		ros::Publisher planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    	ros::WallDuration sleep_t(0.5);
    	while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    	{
     	 sleep_t.sleep();
    	}
		moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
		moveit_msgs::PlanningScene planning_scene;
		moveit_msgs::CollisionObject collision_object;
		collision_object.header.frame_id = arm_->getPlanningFrame();

		// The id of the object is used to identify it.
		collision_object.id = "table";

		// Define a box to add to the world.
		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = 2;
		primitive.dimensions[primitive.BOX_Y] = 2;
		primitive.dimensions[primitive.BOX_Z] = 0.01;

		// Define a pose for the box (specified relative to frame_id)
		geometry_msgs::Pose box_pose;
		box_pose.orientation.w = 1.0;
		box_pose.position.x = 0.0;
		box_pose.position.y = 0.0;
		box_pose.position.z = -0.01/2 -0.02;

		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		planning_scene.world.collision_objects.push_back(collision_object);
    	planning_scene.is_diff = true;
    	planning_scene_diff_publisher.publish(planning_scene);

		ROS_INFO("Added an table into the world");
	}
    
	void some_functions_maybe_useful(){
		// moveit::planning_interface::MoveGroupInterface arm("manipulator");

		geometry_msgs::PoseStamped current_pose = this->arm_->getCurrentPose(this->end_effector_link);
		ROS_INFO("current pose:x:%f,y:%f,z:%f,Quaternion:[%f,%f,%f,%f]",current_pose.pose.position.x,current_pose.pose.position.y,
		current_pose.pose.position.z,current_pose.pose.orientation.x,current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,current_pose.pose.orientation.w);

		std::vector<double> current_joint_values = this->arm_->getCurrentJointValues();
		ROS_INFO("current joint values:%f,%f,%f,%f,%f,%f",current_joint_values[0],current_joint_values[1],current_joint_values[2],
		current_joint_values[3],current_joint_values[4],current_joint_values[5]);

		std::vector<double> rpy = this->arm_->getCurrentRPY(this->end_effector_link);
		ROS_INFO("current rpy:%f,%f,%f",rpy[0],rpy[1],rpy[2]);

		string planner = this->arm_->getPlannerId();
		ROS_INFO("current planner:%s",planner.c_str());
		std::cout<<"current planner:"<<planner<<endl;

	}

	
	~MoveIt_Control() {
		
		ros::shutdown();
	}


public:
	
	string reference_frame;
	string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "moveit_control_server_cpp");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	spinner.start();
	static const std::string PLANNING_GROUP = "manipulator";
	moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	

	MoveIt_Control moveit_server(nh,arm,PLANNING_GROUP);

	// Test 

	// test for move_j
	cout<<"-----------------------test for move_j----------------------"<<endl;
	vector<double> joints ={0,0,-1.57,0,0,0};
	moveit_server.move_j(joints);

	cout << "------------------ test for transform ------------------" << endl;
	// 获取当前末端位姿
	geometry_msgs::PoseStamped current_pose = arm.getCurrentPose(moveit_server.end_effector_link);

	// 从 Pose 转换为 4x4 齐次变换矩阵
	tf::Quaternion q(
		current_pose.pose.orientation.x,
		current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,
		current_pose.pose.orientation.w
	);
	tf::Matrix3x3 rot_matrix(q);

	// 打印变换矩阵
	double matrix[4][4] = {0};
	for (int i = 0; i < 3; ++i) {
		tf::Vector3 row = rot_matrix.getRow(i);
		matrix[i][0] = row.getX();
		matrix[i][1] = row.getY();
		matrix[i][2] = row.getZ();
	}
	matrix[0][3] = current_pose.pose.position.x;
	matrix[1][3] = current_pose.pose.position.y;
	matrix[2][3] = current_pose.pose.position.z;
	matrix[3][3] = 1.0;

	cout << "Transform matrix (end effector in base frame):" << endl;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			printf("%8.4f ", matrix[i][j]);
		}
		cout << endl;
	}

	// 逆解 IK
	robot_state::RobotStatePtr kinematic_state = arm.getCurrentState();
	const robot_state::JointModelGroup* joint_model_group =
		arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	std::string end_effector_link = arm.getEndEffectorLink();

	bool found_ik = kinematic_state->setFromIK(
		joint_model_group, current_pose.pose, end_effector_link, 0.1);

	if (found_ik)
	{
		std::vector<double> joint_values;
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		std::cout << "Inverse Kinematics solution found:" << std::endl;
		for (size_t i = 0; i < joint_values.size(); ++i)
		{
			std::cout << "  Joint " << i << ": " << joint_values[i] << std::endl;
		}

		// 正解验证：用 FK 求出末端位姿
		kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
		kinematic_state->updateLinkTransforms();

		const Eigen::Isometry3d& fk_pose =
			kinematic_state->getGlobalLinkTransform(end_effector_link);

		// 计算位置误差
		double dx = fk_pose.translation().x() - current_pose.pose.position.x;
		double dy = fk_pose.translation().y() - current_pose.pose.position.y;
		double dz = fk_pose.translation().z() - current_pose.pose.position.z;
		double pos_error = std::sqrt(dx * dx + dy * dy + dz * dz);

		// 计算方向误差
		Eigen::Quaterniond q_fk(fk_pose.rotation());
		Eigen::Quaterniond q_target(
			current_pose.pose.orientation.w,
			current_pose.pose.orientation.x,
			current_pose.pose.orientation.y,
			current_pose.pose.orientation.z
		);
		double ori_error = q_fk.angularDistance(q_target); // 单位：弧度

		std::cout << "🧭 Position error: " << pos_error << " m" << std::endl;
		std::cout << "🧭 Orientation error: " << ori_error << " rad (" << ori_error * 180.0 / M_PI << " deg)" << std::endl;

		// 判断容差
		const double POS_TOL = 1e-3;     // 1 mm
		const double ORI_TOL = 1e-2;     // ~0.57 deg

		if (pos_error < POS_TOL && ori_error < ORI_TOL) {
			std::cout << "✅ IK solution verified within tolerance." << std::endl;
		} else {
			std::cout << "❌ IK solution error exceeds tolerance!" << std::endl;
		}
	}
	else
	{
		std::cout << "No IK solution found for the current pose." << std::endl;
	}

    ros::shutdown();


	return 0;
}

