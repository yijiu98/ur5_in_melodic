#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <algorithm> // for std::find

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_add_collision_objct");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(1.0);  // 等待一段时间确保场景初始化

    // ---- 添加球体到末端执行器 ----

    // 创建球体 CollisionObject
    moveit_msgs::CollisionObject ball;
    ball.id = "ball";
    ball.header.frame_id = "ee_link";  // 将球体附加到末端执行器

    shape_msgs::SolidPrimitive ball_primitive;
    ball_primitive.type = ball_primitive.SPHERE;
    ball_primitive.dimensions.resize(1);
    ball_primitive.dimensions[0] = 0.05;  // 设置球体半径为 0.05 米

    // 设置球体位置
    geometry_msgs::Pose ball_pose;
    ball_pose.orientation.w = 1.0;  // 设置为单位四元数（不旋转）
    ball_pose.position.x = 0;  // 球体相对末端执行器的 x 位置
    ball_pose.position.y = 0;  // 球体相对末端执行器的 y 位置
    ball_pose.position.z = 0.1;  // 球体相对末端执行器的 z 位置

    // 将球体添加到 CollisionObject 中
    ball.primitives.push_back(ball_primitive);
    ball.primitive_poses.push_back(ball_pose);
    ball.operation = ball.ADD;

    // 将球体从 CollisionObject 转换为 AttachedCollisionObject
    moveit_msgs::AttachedCollisionObject attached_ball;
    attached_ball.object = ball;
    attached_ball.link_name = "ee_link";  // 将球体附加到末端执行器

    // 创建一个附加物体的列表，并加入球体
    std::vector<moveit_msgs::AttachedCollisionObject> attached_collision_objects;
    attached_collision_objects.push_back(attached_ball);

    // 将球体附加到末端执行器
    current_scene.applyAttachedCollisionObjects(attached_collision_objects);

    ROS_INFO("Ball attached to the end effector.");

    // 等待几秒钟，确保物体已附加
    sleep(5.0);

    // ---- 删除球体 ----
    
    // 获取当前已附加的物体
    auto attached_objects = current_scene.getAttachedObjects();
    std::vector<moveit_msgs::AttachedCollisionObject> attach_objects;
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    for (auto &it : attached_objects)
    {
        // 如果物体的ID是"ball"或它属于指定的工具名称列表
        if (it.first == "ball")  // 或者替换为类似的检查（如检查tool_names）
        {
            ROS_INFO("Detach tool (%s) object_id(%s) link_name(%s)", it.first.c_str(), it.second.object.id.c_str(), it.second.link_name.c_str());
            
            // 标记为 REMOVE 操作
            attach_objects.push_back(it.second);
            attach_objects.back().object.operation = moveit_msgs::CollisionObject::REMOVE;
            collision_objects.push_back(attach_objects.back().object);
        }
    }

    // 从场景中移除这些附加物体
    if (!current_scene.applyAttachedCollisionObjects(attach_objects))
    {
        ROS_ERROR("删除工具对象失败!");
        return false;
    }

    // 如果需要同步物体的碰撞信息
    if (!current_scene.applyCollisionObjects(collision_objects))
    {
        ROS_ERROR("删除工具对象失败!");
        return false;
    }

    ROS_INFO("Ball removed from the scene.");

    // 关闭 ROS 节点
    ros::shutdown();

    return 0;
}
