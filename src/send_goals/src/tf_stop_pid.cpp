// 文件: tf_stop_pid.cpp
// 作者: Guo
// 日期: 2023-10-26
// 功能: 该节点用于接收一个目标坐标系名称作为参数，计算机器人当前位置与该目标坐标系之间的 TF 变换，
//       并根据距离和角度计算线速度和角速度，发布到 /cmd_vel 话题，实现机器人向目标点移动并停止。

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

// 声明全局变量，用于存储目标坐标系的名称
std::string stop_pose_name;

int main(int argc, char *argv[])
{
    // 设置本地化环境，支持中文输出
    setlocale(LC_ALL,"");

    // 初始化 ROS 节点，节点名为 "stop"
    ros::init(argc,argv,"stop");

    // 检查命令行参数数量，确保传入了目标坐标系名称
    if (argc != 2)
    {
        ROS_ERROR("错误：请传入正确的参数！用法：rosrun send_goals tf_stop_pid <目标坐标系名称>");
        return 1; // 参数错误，退出程序
    } else {
        // 获取命令行参数作为目标坐标系名称
        stop_pose_name = argv[1];
    }

    // 创建 ROS 节点句柄
    ros::NodeHandle nh; 

    // 创建一个 Publisher，用于向 "/cmd_vel" 话题发布速度指令
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    // 创建 TF2 缓冲区和监听器，用于获取坐标系之间的变换
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    // 设置循环频率为 1 Hz
    ros::Rate r(1);

    // ROS 主循环
    while (ros::ok())
    {
        // 定义一个 PointStamped 消息，表示机器人自身坐标系 (base_link) 下的原点
        geometry_msgs::PointStamped point_stop;
        
        point_stop.header.frame_id = "base_link";
        // 设置时间戳为最新可用时间，ros::Time() 表示使用当前时间，但这里注释说明是 ROS 的一个 bug，通常用 ros::Time(0) 或 ros::Time::now()
        point_stop.header.stamp = ros::Time(); 

        point_stop.point.x = 0;
        point_stop.point.y = 0;
        point_stop.point.z = 0;

        try 
        {
            // 尝试将 base_link 坐标系下的点转换到指定的目标坐标系下
            geometry_msgs::PointStamped point_world;
            point_world = buffer.transform(point_stop,stop_pose_name);

            // 打印转换后的坐标点信息
            ROS_INFO("目标坐标系 \"%s\" 相对于 base_link 的坐标为:(%.2f,%.2f,%.2f)", stop_pose_name.c_str(), point_world.point.x,point_world.point.y,point_world.point.z);

            // 创建 Twist 消息，用于发布速度指令
            geometry_msgs::Twist speed;

            // 计算线速度：与目标点的距离成正比，使用 PID 思想简化为 P 控制
            // 距离 = sqrt(x^2 + y^2)
            speed.linear.x = 1 * sqrt(pow(point_world.point.x,2) + pow(point_world.point.y,2));
            // 计算角速度：与目标点相对于机器人 x 轴的角度成正比，使用 PID 思想简化为 P 控制
            // 角度 = atan2(y, x)
            speed.angular.z = 5 * atan2(point_world.point.y,point_world.point.x);

            // 限制线速度的最大值，防止速度过快
            if(speed.linear.x > 0.5)
            {
                speed.linear.x = 0.5;
            }
            // 此处缺少对负向线速度的限制，或者可以根据实际需求调整
            // if(speed.linear.x < -0.5)
            // {
            //     speed.linear.x = -0.5;
            // }

            // 发布计算出的速度指令
            pub.publish(speed);

        }
        catch(const tf2::TransformException& e) // 捕获 TF 转换异常
        {
            ROS_WARN("TF 转换异常: %s",e.what());
        }
        catch(const std::exception& e) // 捕获其他标准异常
        {
            ROS_ERROR("程序发生未知异常:%s",e.what());
        }
        // 按照设定的频率休眠
        r.sleep();
        // 处理 ROS 回调函数
        ros::spinOnce();
    }
    return 0;
}




