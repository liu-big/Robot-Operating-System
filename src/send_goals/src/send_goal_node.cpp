// 文件: send_goal_node.cpp
// 作者: Guo
// 日期: 2023-10-26
// 功能: 该节点是一个 ROS C++ 节点，用于向 move_base 动作服务器发送导航目标点。
//       它定义了两个预设的目标点，并依次发送给机器人，等待机器人到达目标点。

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> // 包含 move_base 动作消息定义
#include <actionlib/client/simple_action_client.h> // 包含 SimpleActionClient 类，用于与动作服务器交互
#include <iostream> // 包含标准输入输出流
using namespace std; // 使用标准命名空间
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// 定义 MoveBaseClient 类型，简化代码
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char *argv[])
{
    // 初始化 ROS 节点，节点名为 "send_goals_node"
    ros::init(argc, argv, "send_goals_node");

    // 创建一个 SimpleActionClient，用于与 "move_base" 动作服务器进行通信
    // true 表示在单独的线程中处理回调
    MoveBaseClient ac("move_base", true);
 
    // 等待 move_base 动作服务器启动
    ROS_INFO("等待 move_base 动作服务器启动...");
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("move_base 动作服务器未启动，继续等待...");
    }
    ROS_INFO("move_base 动作服务器已连接！");

    // 定义第一个目标点
    move_base_msgs::MoveBaseGoal goal1;
    // 设置目标点在 map 坐标系下的位置和姿态
    goal1.target_pose.pose.position.x = 0.5;
    goal1.target_pose.pose.position.y = -2.14;
    goal1.target_pose.pose.orientation.z = 0;
    goal1.target_pose.pose.orientation.w = 0.951839761956;
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();

    // 发送第一个目标点给 move_base
    ROS_INFO("发送第一个目标点...");
    ac.sendGoal(goal1);

    // 等待第一个目标点完成
    // 如果目标规划失败，会一直警告
    ROS_INFO("等待第一个目标点完成...");
    ac.waitForResult(); // 阻塞直到目标完成

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("第一个目标点成功抵达！");
    } else {
        ROS_WARN("第一个目标点规划失败或未能抵达！当前状态: %s", ac.getState().toString().c_str());
    }

    // 定义第二个目标点
    move_base_msgs::MoveBaseGoal goal2;
    // 设置目标点在 map 坐标系下的位置和姿态
    goal2.target_pose.pose.position.x = -0.13;
    goal2.target_pose.pose.position.y = -4.96;
    goal2.target_pose.pose.orientation.z = 0;
    goal2.target_pose.pose.orientation.w = 0.951839761956;
    goal2.target_pose.header.frame_id = "map";
    goal2.target_pose.header.stamp = ros::Time::now();

    // 发送第二个目标点给 move_base
    ROS_INFO("发送第二个目标点...");
    ac.sendGoal(goal2);

    // 等待第二个目标点完成
    ROS_INFO("等待第二个目标点完成...");
    ac.waitForResult(); // 阻塞直到目标完成

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("第二个目标点成功抵达！");
    } else {
        ROS_WARN("第二个目标点规划失败或未能抵达！当前状态: %s", ac.getState().toString().c_str());
    }

    ROS_INFO("所有目标点发送完毕，程序结束。");
    return 0;
}