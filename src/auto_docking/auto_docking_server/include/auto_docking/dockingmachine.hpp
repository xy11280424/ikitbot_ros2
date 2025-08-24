#ifndef SRC_DOCKINGMACHINE_HPP
#define SRC_DOCKINGMACHINE_HPP

#include <iostream>
#include <cmath>
#include <list>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "angles/angles.h"

#include "robot_msg/action/auto_dock.hpp"
//#include "robot_msg/msg/auto_dock_feedback.hpp"
#include "robot_msg/action/auto_dock.hpp"
#include "robot_msg/srv/set_device.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2/exceptions.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

/**
 * the event enum
 */
enum Event
{
    NONE = 0,
    RECV_CHG_CMD,
    RECV_SEPARATE_CMD,
    REACHED_FIRST_POINT,
    REACHED_SECOND_POINT,
    REACHED_THIRD_POINT,
    REACHED_RETRY_POINT,
    DOCK_POSE_AVAILABLE,
    DOCK_POSE_UNAVAILABLE,
    ERROR_DOCK_POSE,
    DOCK_IN,
    DOCK_STABILIZING,
    DOCKING_DONE,
    SEPARATE_DONE,
    DISTANCE_OUT,
    DOCKER_NO_POWER,
    TIMEOUT,
    RETRY_FAIL,
    OBSTACLE,
    SEPARATE_STUCK,
    REACHED_SEPARATE_RETRY_POINT
};

class DOCKINGMACHINE;  // 前向声明

/**
 * @brief 状态接口
 */
class State
{
public:
    State() { std::cout << "state construct" << std::endl; }
    virtual ~State() = default;

    virtual void Handle(DOCKINGMACHINE* machine,
                        const geometry_msgs::msg::Pose2D& robot_pose,
                        geometry_msgs::msg::Twist& cmd_vel) = 0;

    virtual void getCurrentState(DOCKINGMACHINE* machine,
                                 uint8_t& state,
                                 std::string& stateStr) = 0;
};

/**
 * @brief 状态机类
 */
class DOCKINGMACHINE
{
public:
    DOCKINGMACHINE(std::unique_ptr<State> init_state, std::shared_ptr<tf2_ros::Buffer> tf, rclcpp::Node::SharedPtr node)
        : state_(std::move(init_state)),
          tf_buffer_(tf),
          node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "Docking state machine construct");

        event_now_ = Event::NONE;
        touch_ctrl_ = true;
        got_dock_pose_ = false;
        chargingState_ = false;
        retry_num_ = 0;
        separated_dis_ = 0.0;
        last_oscillation_time_reset_ = node_->now();

        // 创建服务客户端
        motor_client_ = node_->create_client<robot_msg::srv::SetDevice>("device_operation");
    }

    // ---------------- 状态机核心接口 ----------------
    void ExeCycle(const geometry_msgs::msg::Pose2D& robot_pose,
                  geometry_msgs::msg::Twist& cmd_vel)
    {
        computePoseUpdate(robot_pose);
        if (state_) {
            state_->Handle(this, robot_pose, cmd_vel);
        }
    }

    void chargeState(std::unique_ptr<State> pState);

    void getState(uint8_t& state_now, std::string& stateStr_now)
    {
        if (state_) {
            state_->getCurrentState(this, state_now, stateStr_now);
        }
    }

    void setEvent(Event event) { event_now_ = event; }
    Event getEvent() const { return event_now_; }

    void resetOscillationTime() { last_oscillation_time_reset_ = node_->now(); }

    // ---------------- 运动相关工具 ----------------
    void computePoseUpdate(const geometry_msgs::msg::Pose2D& posenow)
    {
        double dx = posenow.x - pose_last_.x;
        double dy = posenow.y - pose_last_.y;

        poseUpdate_.x = std::sqrt(dx * dx + dy * dy);
        poseUpdate_.theta = angles::normalize_angle(posenow.theta - pose_last_.theta);

        pose_last_ = posenow;
    }

    bool valid_sport_check(double timeout)
    {
        if (timeout > 0.0) {
            rclcpp::Duration elapsed = rclcpp::Duration::from_seconds(timeout);
            if (last_oscillation_time_reset_ + elapsed < node_->now()) {
                return false;
            }
        }
        return true;
    }

    // ---------------- 重试逻辑 ----------------
    void setRetryList(const std::list<geometry_msgs::msg::Pose2D>& poselist)
    {
        retryPoseList_.clear();
        retryPoseList_.assign(poselist.begin(), poselist.end());
    }

    bool getRetryPose()
    {
        if (retryPoseList_.empty())
            return false;
        target_pose_ = retryPoseList_.back();
        retryPoseList_.pop_back();
        return true;
    }

    // ---------------- 电机服务调用 ----------------
    void motor_ctrl(bool state)
    {
        auto request = std::make_shared<robot_msg::srv::SetDevice::Request>();
        request->device_id = "motor";
        request->cmd = "release_motor";
        request->parameter1 = state ? 0 : 1;

        RCLCPP_INFO(node_->get_logger(), "Docking, %s motor", state ? "enable" : "disable");

        if (!motor_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(), "Service /device_operation not available");
            return;
        }

        motor_client_->async_send_request(
            request,
            [this, state](rclcpp::Client<robot_msg::srv::SetDevice>::SharedFuture future) {
                auto response = future.get();
                if (response->result) {
                    RCLCPP_INFO(node_->get_logger(), "Motor %s success", state ? "enable" : "disable");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Motor %s failed", state ? "enable" : "disable");
                }
            });
    }

    // ---------------- 数据成员 ----------------
    bool docking_forward_;
    bool touch_ctrl_;
    bool got_dock_pose_;
    bool chargingState_;
    bool motor_error_;
    int retry_num_;
    bool obstacle_;
    bool line_laser_obstacle_;
    bool sonar_obstacle_;
    bool obstacle_stuck_;
    double max_back_dis_;
    double forward_safety_dis_;
    double separated_dis_;

    geometry_msgs::msg::Pose2D dock_pose_;
    geometry_msgs::msg::Pose2D target_pose_;
    geometry_msgs::msg::Pose2D poseUpdate_;
    geometry_msgs::msg::Pose2D pose_last_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Node::SharedPtr node_;   //持有外部传进来的 Node

private:
    std::unique_ptr<State> state_;
    Event event_now_;
    rclcpp::Time last_oscillation_time_reset_;
    std::list<geometry_msgs::msg::Pose2D> retryPoseList_;

    rclcpp::Client<robot_msg::srv::SetDevice>::SharedPtr motor_client_;
};

/**
 * @brief 初始状态
 */
class InitState : public State
{
public:
    InitState() { std::cout << "init state" << std::endl; }
    ~InitState() override = default;

    void Handle(DOCKINGMACHINE *machine,
        const geometry_msgs::msg::Pose2D &robot_pose,
        geometry_msgs::msg::Twist &cmd_vel);

    void getCurrentState(DOCKINGMACHINE*,
                         uint8_t& state,
                         std::string& stateStr) override
    {
        state = robot_msg::action::AutoDock::Feedback::NONE;
        stateStr = "InitState -> init state";
    }
};


/**
 * @brief The SCAN_DOCKER_POSE class 扫描充电桩位置
 */
class SCAN_DOCKER_POSE : public State
{
public:
    SCAN_DOCKER_POSE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("SCAN_DOCKER_POSE"), "scan docker pose state");
        counter = 0;
        rotated = 0.0;
    }
    ~SCAN_DOCKER_POSE() override = default;

    void Handle(DOCKINGMACHINE* machine,
                const geometry_msgs::msg::Pose2D& robot_pose,
                geometry_msgs::msg::Twist& cmd_vel) override;

    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == OBSTACLE){
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "SCAN_DOCKER_POSE -> scan station OBSTACLE ";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::SEARCHING_STATION;
            stateStr = "SCAN_DOCKER_POSE -> scan station";
        }
    }

private:
    int counter;
    double rotated;
    geometry_msgs::msg::TransformStamped transformStamped_[20]; // data buffer
};


/**
 * @brief The SCAN_DOCKER_POINT class 旋转判断是不是有多个充电桩
 */
class RECOGNIZE_DOCKER : public State
{
public:
    RECOGNIZE_DOCKER()
    {
        //RCLCPP_INFO(rclcpp::get_logger("RECOGNIZE_DOCKER"), "recognize docker state");
        recognized_docker = false;
        rotated_2pi = false;
        docker_dist = 10;
        rotated = 0.0;
    }
    ~RECOGNIZE_DOCKER() override = default;

    void Handle(DOCKINGMACHINE* machine,
                const geometry_msgs::msg::Pose2D& robot_pose,
                geometry_msgs::msg::Twist& cmd_vel) override;

    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        state = robot_msg::action::AutoDock::Feedback::SEARCHING_STATION;
        stateStr = "RECOGNIZE_DOCKER -> recognize station";
    }

private:
    double rotated;
    bool rotated_2pi;
    double yaw_err;
    double docker_dist;
    geometry_msgs::msg::TransformStamped transformStamped_;
    geometry_msgs::msg::Pose2D min_dist_docker_pose;
    bool recognized_docker;
};


/**
 * @brief The GOTO_FIRST_POINT_STATE class 去往充电桩正前方0.6m处
 */
class GOTO_FIRST_POINT_STATE : public State
{
public:
    GOTO_FIRST_POINT_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("GOTO_FIRST_POINT_STATE"), "go to first point");
    }
    ~GOTO_FIRST_POINT_STATE() override = default;

    void Handle(DOCKINGMACHINE* machine,
                const geometry_msgs::msg::Pose2D& robot_pose,
                geometry_msgs::msg::Twist& cmd_vel) override;

    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == OBSTACLE){
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "GOTO_FIRST_POINT_STATE -> docking station 1 OBSTACLE";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "GOTO_FIRST_POINT_STATE -> docking station 1";
        }
    }

    /**
     * @brief updateRetryList
     * @param machine
     * @param RefPoint
     */
    void updateRetryList(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& RefPoint)
    {
        geometry_msgs::msg::Pose2D             tempPoint;
        std::list<geometry_msgs::msg::Pose2D> tempPoints;

        tempPoints.clear();

        // mid
        tempPoints.push_front(RefPoint);
        // left
        tempPoint.theta = angles::normalize_angle(RefPoint.theta + M_PI_2);
        tempPoint.x = RefPoint.x + 0.8 * std::cos(tempPoint.theta);
        tempPoint.y = RefPoint.y + 0.8 * std::sin(tempPoint.theta);
        tempPoints.push_front(tempPoint);
        // right
        tempPoint.theta = angles::normalize_angle(RefPoint.theta - M_PI_2);
        tempPoint.x = RefPoint.x + 0.8 * std::cos(tempPoint.theta);
        tempPoint.y = RefPoint.y + 0.8 * std::sin(tempPoint.theta);
        tempPoints.push_front(tempPoint);
        //mid
        tempPoints.push_front(RefPoint);

        machine->setRetryList(tempPoints);
    }

    bool getSecondPose(DOCKINGMACHINE *machine, std::shared_ptr<tf2_ros::Buffer> tf, geometry_msgs::msg::Pose2D& pose)
    {
        geometry_msgs::msg::PointStamped dockerPoint, basePoint;
        dockerPoint.header.frame_id = "docker";
        dockerPoint.header.stamp = machine->node_->get_clock()->now();
        dockerPoint.point.x = 0.6;
        dockerPoint.point.y = 0.0;
        dockerPoint.point.z = 0.0;

        try
        {
            tf->transform(dockerPoint, basePoint, "odom");
            pose.x = basePoint.point.x;
            pose.y = basePoint.point.y;
            RCLCPP_WARN(machine->node_->get_logger(), "GOTO_FIRST_POINT_STATE -> second pose: x %.3f, y %.3f", pose.x, pose.y);
            return true;
        }
        catch (tf2::TransformException& exception)
        {
            pose.x = 0;
            pose.y = 0;
            RCLCPP_ERROR(machine->node_->get_logger(), "GOTO_FIRST_POINT_STATE -> can't transform: %s", exception.what());
            return false;
        }
    }
};


/**
 * @brief The GOTO_SECOND_POINT_STATE class 去往充电桩正前方0.45米处
 */
class GOTO_SECOND_POINT_STATE:public State
{
public:
    GOTO_SECOND_POINT_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("GOTO_SECOND_POINT_STATE"), "go to second point");
        dis_err = 0;
        angle_err = 0;
        heading = 0;
        reached_xy = false;
    }
    ~GOTO_SECOND_POINT_STATE() override= default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == OBSTACLE){
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "GOTO_SECOND_POINT_STATE -> docking station 2 OBSTACLE";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "GOTO_SECOND_POINT_STATE -> docking station 2";
        }
    }
    /**
     * @brief updateRetryList 更新重试点坐标
     * @param machine
     * @param RefPoint
     */
    void updateRetryList(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& RefPoint)
    {
        geometry_msgs::msg::Pose2D             tempPoint;
        std::list<geometry_msgs::msg::Pose2D> tempPoints;

        tempPoints.clear();

        tempPoint.theta = angles::normalize_angle(RefPoint.theta);
        tempPoint.x = RefPoint.x + 0.2 * std::cos(tempPoint.theta + M_PI);
        tempPoint.y = RefPoint.y + 0.2 * std::sin(tempPoint.theta + M_PI);
        tempPoints.push_front(tempPoint);
        machine->setRetryList(tempPoints);
    }
    /**
     * @brief getThirdPose 获取下一个坐标
     * @param tf
     * @param pose
     * @return
     */
    bool getThirdPose(DOCKINGMACHINE* machine, std::shared_ptr<tf2_ros::Buffer> tf, geometry_msgs::msg::Pose2D& pose)
    {
        geometry_msgs::msg::PointStamped dockerPoint, basePoint;

        dockerPoint.header.frame_id = "docker";
        dockerPoint.header.stamp = machine->node_->get_clock()->now();
        dockerPoint.point.x = 0.45;
        dockerPoint.point.y = 0.0;
        dockerPoint.point.z = 0.0;

        try
        {
            tf->transform(dockerPoint, basePoint, "odom");
            pose.x = basePoint.point.x;
            pose.y = basePoint.point.y;
            RCLCPP_WARN(machine->node_->get_logger(), "GOTO_SECOND_POINT_STATE -> third pose: x %.3f, y %.3f", pose.x, pose.y);
            return true;
        }
        catch (tf2::TransformException& exception)
        {
            pose.x = 0;
            pose.y = 0;
            RCLCPP_ERROR(machine->node_->get_logger(), "GOTO_SECOND_POINT_STATE -> can't transform: %s", exception.what());
            return false;
        }
    }

private:
    double_t dis_err;
    double_t angle_err;
    double_t heading;
    double_t yaw_err;
    bool reached_xy;
};



/**
 * @brief The GOTO_THIRD_POINT_STATE class 获取对准角度
 */
class GOTO_THIRD_POINT_STATE:public State
{
public:
    GOTO_THIRD_POINT_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("GOTO_THIRD_POINT_STATE"), "go to third point");
        reached_xy = false;
        reached_yaw = false;
        dis_err = 0;
        heading = 0;
        angle_err = 0;
        yaw_err = 0;
    }
    ~GOTO_THIRD_POINT_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == OBSTACLE){
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "GOTO_THIRD_POINT_STATE -> docking station 3 OBSTACLE";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "GOTO_THIRD_POINT_STATE -> docking station 3";
        }
    }
    /**
     * @brief updateRetryList 更新重试点坐标
     * @param machine
     * @param RefPoint
     */
    void updateRetryList(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& RefPoint)
    {
        geometry_msgs::msg::Pose2D             tempPoint;
        std::list<geometry_msgs::msg::Pose2D> tempPoints;

        tempPoints.clear();

        tempPoint.theta = angles::normalize_angle(RefPoint.theta + M_PI);
        tempPoint.x = RefPoint.x + 0.5 * std::cos(RefPoint.theta);
        tempPoint.y = RefPoint.y + 0.5 * std::sin(RefPoint.theta);
        tempPoints.push_front(tempPoint);

        machine->setRetryList(tempPoints);
    }
private:
    bool reached_yaw;
    bool reached_xy;

    double_t dis_err;
    double_t heading;
    double_t angle_err;
    double_t yaw_err;
};
/**
 * @brief The GOTO_RETRY_POINT_STATE class 重试状态
 */
class GOTO_RETRY_POINT_STATE:public State
{
public:
    GOTO_RETRY_POINT_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("GOTO_RETRY_POINT_STATE"), "go to retry point");
        reached_yaw = false;
        reached_xy = false;
        dis_err = 0;
        heading = 0;
        angle_err = 0;
        yaw_err = 0;
    }
    ~GOTO_RETRY_POINT_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == OBSTACLE){
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "GOTO_RETRY_POINT_STATE->docking station 4 OBSTACLE";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "GOTO_RETRY_POINT_STATE->docking station 4";
        }
    }

private:
    bool reached_yaw;
    bool reached_xy;

    double_t dis_err;
    double_t heading;
    double_t angle_err;
    double_t yaw_err;
};

/**
 * @brief The SLOW_BACK_STATE class 缓慢后退对接
 */
class SLOW_BACK_STATE:public State
{
public:
    SLOW_BACK_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("SLOW_BACK_STATE"), "slow back");
        distance = 0;
    }
    ~SLOW_BACK_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
        stateStr = "SLOW_BACK_STATE->docking station 5";
    }
    /**
     * @brief updateRetryList
     * @param machine
     * @param RefPoint
     */
    void updateRetryList(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& RefPoint)
    {
        geometry_msgs::msg::Pose2D             tempPoint;
        std::list<geometry_msgs::msg::Pose2D> tempPoints;

        tempPoints.clear();

        tempPoint.theta = angles::normalize_angle(RefPoint.theta + M_PI);
        tempPoint.x = RefPoint.x + 0.7 * std::cos(tempPoint.theta);
        tempPoint.y = RefPoint.y + 0.7 * std::sin(tempPoint.theta);
        tempPoints.push_front(tempPoint);

        machine->setRetryList(tempPoints);
    }
private:
    double distance;
};


/**
 * @brief The SLOW_FORWARD_STATE class 前向对接
 */
class SLOW_FORWARD_STATE:public State
{
public:
    SLOW_FORWARD_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("SLOW_FORWARD_STATE"), "slow forward");
        distance = 0;
    }
    ~SLOW_FORWARD_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {   
        if(machine->getEvent() == OBSTACLE){
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "SLOW_FORWARD_STATE -> docking station 6 OBSTACLE";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "SLOW_FORWARD_STATE -> docking station 6";
        }
    }
    /**
     * @brief updateRetryList
     * @param machine
     * @param RefPoint
     */
    void updateRetryList(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& RefPoint)
    {
        geometry_msgs::msg::Pose2D             tempPoint;
        std::list<geometry_msgs::msg::Pose2D> tempPoints;

        tempPoints.clear();

        tempPoint.theta = angles::normalize_angle(RefPoint.theta + M_PI);
        tempPoint.x = RefPoint.x + 0.7 * std::cos(tempPoint.theta);
        tempPoint.y = RefPoint.y + 0.7 * std::sin(tempPoint.theta);
        tempPoints.push_front(tempPoint);

        machine->setRetryList(tempPoints);
    }
private:
    double distance;
};
/**
 * @brief The CHARGING_STATE class 充电状态检测
 */
class CHARGING_STATE:public State
{
public:
    CHARGING_STATE()
    {
        dock_stabilizer=0;
        //RCLCPP_INFO(rclcpp::get_logger("CHARGING_STATE"), "charging state");
    }
    ~CHARGING_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        switch (machine->getEvent()) {
        case DOCK_IN:
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "CHARGING_STATE -> dock in ";
            break;
        case DOCK_STABILIZING:
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "CHARGING_STATE -> dock stabilizing ";
            break;
        case DOCKING_DONE:
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "CHARGING_STATE -> docking stabilizing";
            break;
        default:
            break;
        }
    }
private:
    uint8_t dock_stabilizer;
};
/**
 * @brief The CHARGING_DONE class 对接完成
 */
class CHARGING_DONE:public State
{
public:
    CHARGING_DONE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("CHARGING_DONE"), "charging done state");
    }
    ~CHARGING_DONE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        state = robot_msg::action::AutoDock::Feedback::DOCKING_SUCCESS;
        stateStr = "CHARGING_DONE -> docking success";
    }
};
/**
 * @brief The FAIL_STATE class 失败状态
 */
class FAIL_STATE:public State
{
public:
    FAIL_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("FAIL_STATE"), "fail state");
    }
    ~FAIL_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        switch (machine->getEvent())
        {
            case REACHED_FIRST_POINT:
            case REACHED_SECOND_POINT:
            case REACHED_THIRD_POINT:
            case REACHED_RETRY_POINT:
            case DOCK_IN:
            case DISTANCE_OUT:
                state_ = robot_msg::action::AutoDock::Feedback::DOCKER_OUT_OF_DISTANCE ;
                stateStr_ = "FAIL_STATE -> docking out of distance";
                break;
            case RETRY_FAIL:
                state_ = robot_msg::action::AutoDock::Feedback::DOCKING_FAILURE ;
                stateStr_ = "FAIL_STATE -> docking retry failure";
                break;
            case OBSTACLE:
                state_ = robot_msg::action::AutoDock::Feedback::OBSTACLE_FAILURE ;
                stateStr_ = "FAIL_STATE -> docking failure, obstacle";
                break;
            case DOCK_POSE_UNAVAILABLE:
                state_ = robot_msg::action::AutoDock::Feedback::SEARCHING_FAILURE ;
                stateStr_ = "FAIL_STATE -> docker not found";
                break;
            case ERROR_DOCK_POSE:
                state_ = robot_msg::action::AutoDock::Feedback::ERROR_DOCKER_POS ;
                stateStr_ = "FAIL_STATE -> error docker pose found";
                break;
            case DOCKER_NO_POWER:
                state_ = robot_msg::action::AutoDock::Feedback::DOCKER_NO_POWER ;
                stateStr_ = "FAIL_STATE -> docker no power";
                break;
            case RECV_SEPARATE_CMD:
                state_ = robot_msg::action::AutoDock::Feedback::SEPERAT_FAILURE ;
                stateStr_ = "FAIL_STATE -> seperate failed";
                break;
            case SEPARATE_STUCK:
                state_ = robot_msg::action::AutoDock::Feedback::SEPERAT_FAILURE ;
                stateStr_ = "FAIL_STATE -> seperate failed";
                break;
            default:
                break;
        }
        state = state_;
        stateStr = stateStr_;
    }

private:
    uint8_t state_;
    std::string stateStr_;
};
/**
 * @brief The OBSTACLE_STATE class 障碍物检测
 */
class OBSTACLE_STATE:public State
{
public:
    OBSTACLE_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("OBSTACLE_STATE"), "obstacle state");
    }
    ~OBSTACLE_STATE() override = default;

    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose, geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        state = robot_msg::action::AutoDock::Feedback::OBSTACLE ;
        stateStr = "OBSTACLE_STATE -> docking failure, obstacle";
    }
private:
};
/**
 * @brief The SEPARATE_STATE class 脱离状态
 */
class SEPARATE_STATE:public State
{
public:
    SEPARATE_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("SEPARATE_STATE"), "separate state");
        distance = 0;
    }
    ~SEPARATE_STATE() override = default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == DISTANCE_OUT)
        {
            state = robot_msg::action::AutoDock::Feedback::DOCKING_STATION;
            stateStr = "SEPARATE_STATE -> docking station 7";
        }
        else if(machine->getEvent() == OBSTACLE)
        {
                
            state = robot_msg::action::AutoDock::Feedback::OBSTACLE;
            stateStr = "SEPARATE_STATE -> separating obstacle";
        }
        else
        {
            state = robot_msg::action::AutoDock::Feedback::SEPERATING;
            stateStr = "SEPARATE_STATE -> separating";
        }

    }
private:
    double distance;

};
/**
 * @brief The SEPARATE_DONE_STATE class 脱离完成
 */
class SEPARATE_DONE_STATE:public State
{
public:
    SEPARATE_DONE_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("SEPARATE_DONE_STATE"), "separate done state");
    }
    ~SEPARATE_DONE_STATE() override = default;

    void Handle(DOCKINGMACHINE *machine, const geometry_msgs::msg::Pose2D &robot_pose, geometry_msgs::msg::Twist &cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t & state, std::string& stateStr) override
    {
        state = robot_msg::action::AutoDock::Feedback::SEPERATING_DONE;
        stateStr = "SEPARATE_DONE_STATE -> separating done";
    }
};


/**
 * @brief The GOTO_SEPARATE_RECOVER_POINT_STATE class 退桩卡住恢复去往充电桩正前方0.45米处
 */
class GOTO_SEPARATE_RECOVER_POINT_STATE:public State
{
public:
    GOTO_SEPARATE_RECOVER_POINT_STATE()
    {
        //RCLCPP_INFO(rclcpp::get_logger("GOTO_SEPARATE_RECOVER_POINT_STATE"), "go to separate recover point");
        distance=0;
    }
    ~GOTO_SEPARATE_RECOVER_POINT_STATE() override= default;
    void Handle(DOCKINGMACHINE* machine, const geometry_msgs::msg::Pose2D& robot_pose,geometry_msgs::msg::Twist& cmd_vel) override;
    void getCurrentState(DOCKINGMACHINE* machine, uint8_t& state, std::string& stateStr) override
    {
        if(machine->getEvent() == SEPARATE_DONE){
            state = robot_msg::action::AutoDock::Feedback::SEPERATING_DONE;
            stateStr = "GOTO_SEPARATE_RECOVER_POINT_STATE -> separating done";
        }
        else {
            state = robot_msg::action::AutoDock::Feedback::SEPERATING;
            stateStr = "GOTO_SEPARATE_RECOVER_POINT_STATE -> separating";
        }
    }
   
private:
    double distance;
};

#endif //SRC_DOCKINGMACHINE_HPP
