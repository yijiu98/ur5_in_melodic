#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

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
        triangle.vertex_indices = {i * 3, i * 3 + 1, i * 3 + 2};
        mesh.triangles.push_back(triangle);
    }

    stl_file.close();
    return mesh;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_collision_object");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    
    sleep(2.0);

    moveit_msgs::CollisionObject collision_object;
    collision_object.id = "tool_stl_object";
    collision_object.header.frame_id = move_group.getEndEffectorLink();

    // collision_object.header.frame_id = move_group.getPlanningFrame();

    std::string stl_file_path = "/home/bingda/catkin_ws/src/universal_robot/ur_planning/mesh/output_mesh.stl";
    shape_msgs::Mesh mesh = loadStlFile(stl_file_path);

    geometry_msgs::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;
    mesh_pose.position.x = 0;
    mesh_pose.position.y = 0;
    mesh_pose.position.z = 0;

    collision_object.meshes.push_back(mesh);
    collision_object.mesh_poses.push_back(mesh_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Added collision object to the planning scene.");
    ros::Duration(1.0).sleep();

    ros::shutdown();
    return 0;
}
