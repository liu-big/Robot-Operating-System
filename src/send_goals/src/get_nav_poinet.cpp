// 文件: get_nav_poinet.cpp
// 作者: Guo
// 日期: 2023-10-26
// 功能: 该节点是一个 ROS C++ 节点，用于获取机器人当前在 "map" 坐标系下的位置和姿态信息。
//       它通过 TF 变换监听器获取 "base_link" 到 "map" 的变换，并打印出位置（x, y, z）
//       以及姿态的四元数和欧拉角（roll, pitch, yaw）。

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Quaternion.h" // 包含四元数相关的头文件
#include "tf2/LinearMath/Matrix3x3.h" // 包含矩阵相关的头文件

int main(int argc, char *argv[])
{
    // 设置本地化环境，支持中文输出
    setlocale(LC_ALL,"");

    // 初始化 ROS 节点，节点名为 "get_nav_point"
    ros::init(argc,argv,"get_nav_point");

    // 创建 ROS 节点句柄
    ros::NodeHandle nh;

    // 创建 TF2 缓冲区和监听器，用于获取坐标系之间的变换
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 设置循环频率为 1 Hz
    ros::Rate r(1);

    // ROS 主循环
    while(ros::ok())
    {
        // 定义一个 PointStamped 消息，表示机器人自身坐标系 (base_link) 下的原点
        geometry_msgs::PointStamped point_base_link;

        point_base_link.header.frame_id = "base_link";
        // 设置时间戳为最新可用时间，ros::Time() 表示使用当前时间
        point_base_link.header.stamp = ros::Time();

        point_base_link.point.x = 0;
        point_base_link.point.y = 0;
        point_base_link.point.z = 0;

        try 
        {
            // 尝试将 base_link 坐标系下的点转换到 map 坐标系下
            geometry_msgs::PointStamped point_world;
            point_world = buffer.transform(point_base_link, "map");

            // 打印转换后的坐标点信息（位置）
            ROS_INFO("坐标点相对于 map 的坐标为:(x=%.6f, y=%.6f, z=%.6f)", point_world.point.x, point_world.point.y, point_world.point.z);

            // 获取从 "map" 到 "base_link" 的 TF 变换
            geometry_msgs::TransformStamped transformStamped = buffer.lookupTransform("map", "base_link", ros::Time(0));
            
            // 将变换中的旋转部分（四元数）转换为 tf2::Quaternion 类型
            tf2::Quaternion quat;
            tf2::fromMsg(transformStamped.transform.rotation, quat);

            // 将四元数转换为欧拉角 (roll, pitch, yaw)
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // 打印四元数和欧拉角信息
            ROS_INFO("四元数: (x=%.6f, y=%.6f, z=%.6f, w=%.6f)", quat.x(), quat.y(), quat.z(), quat.w());
            ROS_INFO("欧拉角: (roll=%.6f, pitch=%.6f, yaw=%.6f)", roll, pitch, yaw);
        }
        catch (const tf2::TransformException &ex) // 捕获 TF 转换异常
        {
            ROS_WARN("TF 转换异常: %s", ex.what());
        }
        catch (const std::exception &ex) // 捕获其他标准异常
        {
            ROS_ERROR("程序发生未知异常: %s", ex.what());
        }

        // 按照设定的频率休眠
        r.sleep();
        // 处理 ROS 回调函数
        ros::spinOnce();
    }

    return 0;
}
