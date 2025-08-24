/**
 * @file        auto_docking.hpp
 * @brief       自动对接 action_server (ROS2)
 * @details     自动对接 action 服务端 ROS2 版本
 * @author      guoxy
 * @date        2025-08-15
 * @version     ros 3.0.0
 */
#ifndef SRC_AUTO_DOCKING_HPP
#define SRC_AUTO_DOCKING_HPP

#include <vector>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "auto_docking/dockingmachine.hpp"
#include "auto_docking/laser_obstacle_detect.h"
#include "auto_docking/yaml_helper.h"
#include "robot_msg/msg/base_info.hpp"
#include "robot_msg/srv/set_device.hpp"
#include "robot_msg/action/auto_dock.hpp"

class AutoDockingAction : public rclcpp::Node
{
public:
    using AutoDockAction = robot_msg::action::AutoDock;
    using GoalHandleAutoDock = rclcpp_action::ServerGoalHandle<AutoDockAction>;

    AutoDockingAction(std::string name, std::shared_ptr<tf2_ros::Buffer> tf);
    ~AutoDockingAction();

    void Init(bool dockCtrl, bool touch);

private:
    // ROS2 Action Server
    rclcpp_action::Server<AutoDockAction>::SharedPtr action_server_;
    //void executeCB(const std::shared_ptr<GoalHandleAutoDock> goal_handle);

    void execute(const std::shared_ptr<GoalHandleAutoDock> goal_handle);
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const robot_msg::action::AutoDock::Goal> goal);

    // 取消回调
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleAutoDock> goal_handle);

    // 执行回调
    void handle_accepted(const std::shared_ptr<GoalHandleAutoDock> goal_handle);



    // ROS2 Publisher / Subscriber
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<robot_msg::msg::BaseInfo>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr docker_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr collision_bar_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> sonars_sub_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> line_laser_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_collision_bar_;

    // ROS2 Service
    rclcpp::Service<robot_msg::srv::SetDevice>::SharedPtr config_server_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr camera_control_client_up_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr camera_control_client_down_;

    // Parameters
    bool use_imu_angle_;
    bool use_bms_state_;
    bool enable_sonar_;
    bool enable_line_laser_;
    bool enable_virtual_bumper_;
    bool enable_real_bumper_;
    bool docking_success_op_camera_up_;
    bool docking_success_op_camera_down_;
    bool docking_forward_;
    uint8_t baseChargingState_;
    double seperation_dis_;
    double safety_distance_;
    double max_docking_distance_;
    double forward_safety_dis_;
    double docking_point_offset_;
    double distance_movebase_arrived_tolerance_;
    double scan_docker_distance_tolerance_;

    // Robot state
    geometry_msgs::msg::Pose2D robot_pose_;
    geometry_msgs::msg::Twist robot_vel_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    robot_msg::action::AutoDock::Feedback feedback_;
    std::string action_name_;

    // Obstacle detection
    LaserObstacleDetect obs_detector_;
    bool virtual_bump_trigger_;
    bool real_bump_trigger_;
    bool bump_trigger_;

    // Docking state machine
    std::unique_ptr<DOCKINGMACHINE> dockingmachine_;

    // Functions
    void LoadParam();
    void RobotState_CallBack(const robot_msg::msg::BaseInfo::SharedPtr msg);
    void Odom_CallBack(const nav_msgs::msg::Odometry::SharedPtr msg);
    void Imu_CallBack(const sensor_msgs::msg::Imu::SharedPtr msg);
    void BatteryCallBack(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void SonarCallBacks(const sensor_msgs::msg::Range::SharedPtr msg);
    void LineLaserCallBacks(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void collisionBarCB(const std_msgs::msg::UInt8::SharedPtr msg);
    void dockerCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    bool configServer(const std::shared_ptr<robot_msg::srv::SetDevice::Request> request,
                      std::shared_ptr<robot_msg::srv::SetDevice::Response> response);

    //void SafetyControl(geometry_msgs::msg::Twist &target_vel);
    void SafetyControl(geometry_msgs::msg::Twist &target_vel, const std::shared_ptr<GoalHandleAutoDock> goal_handle);
    void handleCameraAfterDocking(bool swt);
};

#endif // SRC_AUTO_DOCKING_HPP
