#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Point.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <ros/ros.h>

void disableCollisionForAttachedObject(const std::string& object_id)
{
    // 创建 PlanningScene 消息
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;

    // 设置允许的碰撞矩阵
    moveit_msgs::AllowedCollisionMatrix& acm = planning_scene.allowed_collision_matrix;
    acm.entry_names.push_back(object_id);
    acm.entry_values.resize(1);
    acm.entry_values[0].enabled.resize(1, true);

    // 发布 PlanningScene 消息
    ros::NodeHandle nh; // 添加 NodeHandle
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Duration(1.0).sleep(); // 等待发布器初始化

    planning_scene_diff_publisher.publish(planning_scene);
    ROS_INFO("Disabled collision for object: %s", object_id.c_str());
}



// 加载 STL 文件的自定义函数
shape_msgs::Mesh loadStlFile(const std::string &stl_file_path)
{
    shape_msgs::Mesh mesh;
    std::ifstream stl_file(stl_file_path, std::ios::in | std::ios::binary);
    
    if (!stl_file.is_open())
    {
        ROS_ERROR("Failed to open STL file");
        return mesh;
    }

    // 读取二进制STL文件头
    char header[80];
    stl_file.read(header, 80);

    // 读取三角形数据
    uint32_t num_triangles;
    stl_file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));

    // 逐个读取三角形的顶点数据
    for (uint32_t i = 0; i < num_triangles; ++i)
    {
        float normal[3], v1[3], v2[3], v3[3];
        uint16_t attribute_byte_count;
        
        stl_file.read(reinterpret_cast<char*>(&normal), sizeof(float) * 3);
        stl_file.read(reinterpret_cast<char*>(&v1), sizeof(float) * 3);
        stl_file.read(reinterpret_cast<char*>(&v2), sizeof(float) * 3);
        stl_file.read(reinterpret_cast<char*>(&v3), sizeof(float) * 3);
        stl_file.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(uint16_t));

        // 将三角形的顶点加入到 Mesh 中
        geometry_msgs::Point p1, p2, p3;
        p1.x = v1[0];
        p1.y = v1[1];
        p1.z = v1[2];
        p2.x = v2[0];
        p2.y = v2[1];
        p2.z = v2[2];
        p3.x = v3[0];
        p3.y = v3[1];
        p3.z = v3[2];

        mesh.vertices.push_back(p1);
        mesh.vertices.push_back(p2);
        mesh.vertices.push_back(p3);

        // 创建一个三角形对象并添加到 mesh.triangles
        shape_msgs::MeshTriangle triangle;
        triangle.vertex_indices = {i * 3, i * 3 + 1, i * 3 + 2};  // 每个三角形的三个顶点索引
        mesh.triangles.push_back(triangle);
    }

    stl_file.close();
    return mesh;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_add_collision_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);
    spin.start();

    // 创建运动规划的情景，等待创建完成
    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);

    // 获取 MoveGroup 对象和末端执行器链接
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");  // 这里的 "manipulator" 是你在 MoveIt 配置时设置的组名称
    std::string end_effector_link = move_group.getEndEffectorLink();

    // 获取末端执行器当前的位姿
    geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();

    // 设置物体的位置偏移（根据需要调整偏移量）
    geometry_msgs::Pose pose;
    pose.orientation = current_pose.pose.orientation;  // 保持末端执行器的方向不变

    // 在末端执行器当前位置基础上做一个平移（例如，物体需要在末端执行器下方）
    pose.position.x = current_pose.pose.position.x;   // X轴的偏移
    pose.position.y = current_pose.pose.position.y ;   // Y轴的偏移
    pose.position.z = current_pose.pose.position.z ;  // Z轴的偏移

    // 创建附件物体（工具）
    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = end_effector_link;  // 将物体附加到末端执行器上
    attached_object.object.id = "tool_stl_object";

    // 加载 STL 文件
    std::string stl_file_path = "/home/bingda/catkin_ws/src/universal_robot/ur_planning/mesh/j_clamp.stl";
    shape_msgs::Mesh mesh = loadStlFile(stl_file_path);

    // 将网格物体的属性、位置加入到附件物体中
    attached_object.object.meshes.push_back(mesh);
    attached_object.object.mesh_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;

    // 将附件物体加入到当前场景
    current_scene.applyAttachedCollisionObject(attached_object);

    // 稍等一会儿，确保物体被成功附加
    ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}
