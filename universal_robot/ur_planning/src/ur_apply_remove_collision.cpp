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
