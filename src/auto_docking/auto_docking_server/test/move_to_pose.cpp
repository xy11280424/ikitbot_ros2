#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "angles/angles.h"
#include <chrono>
#include <cmath>

class MoveToPose : public rclcpp::Node
{
public:
    MoveToPose(tf2_ros::Buffer &tf)
    : Node("move_to_pose"),
      kp_rho(1.0),
      kp_alpha(5.0),
      kp_beta(0.0),
      tfBuffer_(tf)
    {
        RCLCPP_INFO(this->get_logger(), "move to pose");

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 100, std::bind(&MoveToPose::odomCallback, this, std::placeholders::_1));
    }

    void getTarget()
    {
        geometry_msgs::msg::Pose2D docker_pose;
        geometry_msgs::msg::PointStamped docker_point, base_point;

        // 查询最近的 20 帧 docker 位姿
        for (int counter = 0; counter < 20; counter++)
        {
            transformStamped_[counter] = tfBuffer_.lookupTransform(
                "odom", "docker", tf2::TimePointZero);
        }

        // 时间判断
        auto time_diff = transformStamped_[19].header.stamp.sec - transformStamped_[15].header.stamp.sec;
        if (time_diff < 5.0)
        {
            docker_pose.x = 0.0;
            docker_pose.y = 0.0;
            docker_pose.theta = 0.0;
            for (int i = 15; i <= 19; i++)
            {
                tf2::Quaternion q(
                    transformStamped_[i].transform.rotation.x,
                    transformStamped_[i].transform.rotation.y,
                    transformStamped_[i].transform.rotation.z,
                    transformStamped_[i].transform.rotation.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getEulerYPR(yaw, pitch, roll);

                docker_pose.x += transformStamped_[i].transform.translation.x / 5.0;
                docker_pose.y += transformStamped_[i].transform.translation.y / 5.0;
                docker_pose.theta += yaw / 5.0;
            }
            RCLCPP_INFO(this->get_logger(),
                        "---docker pose update: x %.3f, y %.3f, theta %.3f",
                        docker_pose.x, docker_pose.y, angles::to_degrees(docker_pose.theta));
        }

        docker_point.header.frame_id = "docker";
        docker_point.header.stamp = this->get_clock()->now();
        docker_point.point.x = 0.5;
        docker_point.point.y = 0.0;
        docker_point.point.z = 0.0;

        try
        {
            tfBuffer_.transform(docker_point, base_point, "odom");
            target_pose_.x = base_point.point.x;
            target_pose_.y = base_point.point.y;
            target_pose_.theta = angles::normalize_angle(docker_pose.theta + M_PI);
            RCLCPP_WARN(this->get_logger(),
                        "target pose: x %.3f, y %.3f, theta %.3f",
                        target_pose_.x, target_pose_.y, angles::to_degrees(target_pose_.theta));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Can't transform: %s", ex.what());
        }
    }

    void computeCmd()
    {
        geometry_msgs::msg::Twist cmd_vel;

        double x_diff = target_pose_.x - robot_pose_.x;
        double y_diff = target_pose_.y - robot_pose_.y;

        rho_ = std::hypot(x_diff, y_diff);
        alpha_ = angles::normalize_angle(std::atan2(y_diff, x_diff) - robot_pose_.theta);
        beta_  = angles::normalize_angle(target_pose_.theta - robot_pose_.theta - alpha_);

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "rho, alpha, beta: %.3f, %.3f, %.3f",
                             rho_, angles::to_degrees(alpha_), angles::to_degrees(beta_));

        if (rho_ < 0.03)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
        }
        else
        {
            cmd_vel.linear.x  = kp_rho * rho_;
            cmd_vel.angular.z = kp_alpha * alpha_ + kp_beta * beta_;

            if (alpha_ > M_PI_4 || alpha_ < -M_PI_4)
                cmd_vel.linear.x = 0.0;

            cmd_vel.linear.x  = std::min(cmd_vel.linear.x, 0.05);
            cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -0.3, 0.3);
        }

        vel_pub_->publish(cmd_vel);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose_.x = msg->pose.pose.position.x;
        robot_pose_.y = msg->pose.pose.position.y;

        tf2::Quaternion quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        robot_pose_.theta = yaw;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "odom: x %.3f, y %.3f, theta %.3f",
                             robot_pose_.x, robot_pose_.y, angles::to_degrees(robot_pose_.theta));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Pose2D target_pose_;
    geometry_msgs::msg::Pose2D robot_pose_;

    tf2_ros::Buffer &tfBuffer_;
    geometry_msgs::msg::TransformStamped transformStamped_[20];

    double rho_, kp_rho;
    double alpha_, kp_alpha;
    double beta_, kp_beta;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto tfBuffer = std::make_shared<tf2_ros::Buffer>(rclcpp::Clock(RCL_ROS_TIME));
    auto tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    // 延时等待 TF 启动
    rclcpp::sleep_for(std::chrono::seconds(2));

    auto node = std::make_shared<MoveToPose>(*tfBuffer);
    node->getTarget();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        node->computeCmd();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
