#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "auto_docking/auto_docking.hpp"  // 你的自定义类

int main(int argc, char **argv)
{
    // 初始化 ROS2 节点
    rclcpp::init(argc, argv);

    // 创建节点句柄
    //auto node = rclcpp::Node::make_shared("auto_docking");

    // TF2 buffer 和 listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // 创建你的 AutoDockingAction 对象
    //auto auto_docking_action = std::make_shared<AutoDockingAction>("auto_docking_server", tf_buffer, node);
    auto auto_docking_action = std::make_shared<AutoDockingAction>("auto_docking_server", tf_buffer);

    // ROS2 循环
    //rclcpp::spin(node);
    rclcpp::spin(auto_docking_action);  // spin 这个节点
    rclcpp::shutdown();
    return 0;
}

