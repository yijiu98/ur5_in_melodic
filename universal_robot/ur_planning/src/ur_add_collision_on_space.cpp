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
#include <Eigen/Geometry>

// 加载 STL 文件的自定义函数
// shape_msgs::Mesh loadStlFile(const std::string &stl_file_path)
// {
//     shape_msgs::Mesh mesh;
//     std::ifstream stl_file(stl_file_path, std::ios::in | std::ios::binary);
    
//     if (!stl_file.is_open())
//     {
//         ROS_ERROR("Failed to open STL file");
//         return mesh;
//     }

//     // 读取二进制STL文件头
//     char header[80];
//     stl_file.read(header, 80);

//     // 读取三角形数据
//     uint32_t num_triangles;
//     stl_file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));

//     // 逐个读取三角形的顶点数据
//     for (uint32_t i = 0; i < num_triangles; ++i)
//     {
//         float normal[3], v1[3], v2[3], v3[3];
//         uint16_t attribute_byte_count;
        
//         stl_file.read(reinterpret_cast<char*>(&normal), sizeof(float) * 3);
//         stl_file.read(reinterpret_cast<char*>(&v1), sizeof(float) * 3);
//         stl_file.read(reinterpret_cast<char*>(&v2), sizeof(float) * 3);
//         stl_file.read(reinterpret_cast<char*>(&v3), sizeof(float) * 3);
//         stl_file.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(uint16_t));

//         // 将三角形的顶点加入到 Mesh 中
//         geometry_msgs::Point p1, p2, p3;
//         p1.x = v1[0];
//         p1.y = v1[1];
//         p1.z = v1[2];
//         p2.x = v2[0];
//         p2.y = v2[1];
//         p2.z = v2[2];
//         p3.x = v3[0];
//         p3.y = v3[1];
//         p3.z = v3[2];

//         mesh.vertices.push_back(p1);
//         mesh.vertices.push_back(p2);
//         mesh.vertices.push_back(p3);

//         // 创建一个三角形对象并添加到 mesh.triangles
//         shape_msgs::MeshTriangle triangle;
//         triangle.vertex_indices = {i * 3, i * 3 + 1, i * 3 + 2};  // 每个三角形的三个顶点索引
//         mesh.triangles.push_back(triangle);
//     }

//     stl_file.close();
//     return mesh;
// }
shape_msgs::Mesh loadStlFile(const std::string &stl_file_path)
{
    shape_msgs::Mesh mesh;
    std::ifstream stl_file(stl_file_path);
    
    if (!stl_file.is_open())
    {
        ROS_ERROR("Failed to open STL file");
        return mesh;
    }

    std::string first_line;
    std::getline(stl_file, first_line);

    if (first_line.find("solid") == 0)  // 检查是否是 ASCII STL
    {
        ROS_INFO("Detected ASCII STL file, parsing...");
        std::string line;
        std::vector<geometry_msgs::Point> vertices;

        while (std::getline(stl_file, line))
        {
            std::istringstream iss(line);
            std::string keyword;
            iss >> keyword;
            
            if (keyword == "vertex")
            {
                geometry_msgs::Point p;
                iss >> p.x >> p.y >> p.z;
                vertices.push_back(p);

                if (vertices.size() == 3)  // 每 3 个点形成一个三角形
                {
                    mesh.vertices.push_back(vertices[0]);
                    mesh.vertices.push_back(vertices[1]);
                    mesh.vertices.push_back(vertices[2]);

                    shape_msgs::MeshTriangle triangle;
                    triangle.vertex_indices[0] = mesh.vertices.size() - 3;
                    triangle.vertex_indices[1] = mesh.vertices.size() - 2;
                    triangle.vertex_indices[2] = mesh.vertices.size() - 1;
                    mesh.triangles.push_back(triangle);
                    
                    vertices.clear();  // 清空以读取下一个三角形
                }
            }
        }
    }
    else  // 处理二进制 STL
    {
        ROS_INFO("Detected binary STL file, parsing...");
        stl_file.close();
        stl_file.open(stl_file_path, std::ios::in | std::ios::binary);
        stl_file.seekg(80, std::ios::beg); // 跳过 STL 头部

        uint32_t num_triangles;
        stl_file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));

        for (uint32_t i = 0; i < num_triangles; ++i)
        {
            float normal[3], v1[3], v2[3], v3[3];
            uint16_t attribute_byte_count;

            stl_file.read(reinterpret_cast<char*>(&normal), sizeof(float) * 3);
            stl_file.read(reinterpret_cast<char*>(&v1), sizeof(float) * 3);
            stl_file.read(reinterpret_cast<char*>(&v2), sizeof(float) * 3);
            stl_file.read(reinterpret_cast<char*>(&v3), sizeof(float) * 3);
            stl_file.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(uint16_t));

            geometry_msgs::Point p1, p2, p3;
            p1.x = v1[0]; p1.y = v1[1]; p1.z = v1[2];
            p2.x = v2[0]; p2.y = v2[1]; p2.z = v2[2];
            p3.x = v3[0]; p3.y = v3[1]; p3.z = v3[2];

            mesh.vertices.push_back(p1);
            mesh.vertices.push_back(p2);
            mesh.vertices.push_back(p3);

            shape_msgs::MeshTriangle triangle;
            triangle.vertex_indices[0] = mesh.vertices.size() - 3;
            triangle.vertex_indices[1] = mesh.vertices.size() - 2;
            triangle.vertex_indices[2] = mesh.vertices.size() - 1;
            mesh.triangles.push_back(triangle);
        }
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

    // ---- 设置障碍物的位置 ----
    geometry_msgs::Pose pose;
    // pose.orientation.w = 0;  // 设置为单位四元数（不旋转）
    // pose.position.x = 0.0;     // 障碍物的 x 坐标位置
    // pose.position.y = 0;   // 障碍物的 y 坐标位置
    // pose.position.z = 1;     // 障碍物的 z 坐标位置
    Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    // ---- 加载 STL 文件 ----
    std::string stl_file_path = "/home/bingda/catkin_ws/src/universal_robot/ur_planning/mesh/output_mesh.stl";
    shape_msgs::Mesh mesh = loadStlFile(stl_file_path);

    // ---- 创建 CollisionObject ----
    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "stl_obstacle";  // 障碍物的 ID
    collision_object.header.frame_id = "world";  // 设置在 "world" 坐标系下
    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(pose);  // 设置障碍物的位姿
    collision_object.operation = collision_object.ADD;

    // ---- 将障碍物加入到场景 ----
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    current_scene.addCollisionObjects(collision_objects);

    ROS_INFO("STL obstacle added to the scene at the position (0.0, -0.93, 0.5).");

    // 等待几秒钟，确保障碍物已添加到场景
    sleep(5.0);

    ros::shutdown();
    return 0;
}
