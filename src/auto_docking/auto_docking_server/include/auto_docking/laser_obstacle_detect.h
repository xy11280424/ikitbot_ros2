/**
 * @file        laser_obstacle_detect.h
 * @brief       自动对接action_server
 * @details  	自动对接action服务端
 * @author      weibw
 * @version     V1.0.0
 * @copyright   Copyright (c) 2021-2021  iscas
 */
#ifndef LASER_OBSTACLE_DETECT_H
#define LASER_OBSTACLE_DETECT_H
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "laser_geometry/laser_geometry.hpp"
#include <tf2/exceptions.h>
#include <cmath>

class LaserObstacleDetect : public rclcpp::Node
{
public:
    LaserObstacleDetect();

    bool haveObstacle(const geometry_msgs::msg::Twist &twist);
    void setRobotRadius(float radius);

private:
    void scanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    bool cvtLaserScan2Pointcloud(const sensor_msgs::msg::LaserScan &scan,
                                 sensor_msgs::msg::PointCloud2 &cloud);
    bool obstacleDetect(const sensor_msgs::msg::PointCloud2 &cloud,
                        const geometry_msgs::msg::Twist &twist);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_sub;
    sensor_msgs::msg::LaserScan m_scan;

    rclcpp::Time m_obstacle_time;
    rclcpp::Time m_free_time;
    float m_robot_radius_sq;

    laser_geometry::LaserProjection m_projector;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
};

#endif // LASER_OBSTACLE_DETECT_H
