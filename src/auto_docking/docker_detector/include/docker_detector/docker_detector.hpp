/**
 * @file        docker_detector.hpp
 * @brief       充电桩检测
 * @details  	根据激光线段特征识别充电桩位置，并发布tf关系
 * @author      glh
 * @date        2020-08-04
 * @version     V1.0.0
 * @copyright   Copyright (c) 2021-2021  iscas
 */
#ifndef SRC_DOCKER_DETECTOR_HPP
#define SRC_DOCKER_DETECTOR_HPP

#include <iostream>
#include <vector>
#include <list>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "angles/angles.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "docker_detector/line_split.hpp"

using namespace std;
using namespace visualization_msgs;

class DOCKER : public rclcpp::Node
{
public:
    DOCKER();
    ~DOCKER();

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr docker_info_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 参数
    int32_t min_point_num;
    int32_t min_index_num;
    double max_gap_dis;
    double in_line_threshold;
    double docker_line_len;
    double docker_angle_diff;
    double docker_distance_threshold;
    double match_len_tolerance;
    double match_angle_tolerance;
    double laser_yaw_calib_;
    double intensity_min, intensity_max;

    std::string laser_frame, docker_frame;
    bool pub_visual_marker;
    bool lidar_reverse_;

    geometry_msgs::msg::PoseStamped docker_pose_;

    std::mutex param_locker;
    LineSplit* lineSplit_;

    void laser_CallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void docker_match(const std::vector<Line>& lines);
    void calcDockerPose(const std::vector<Line>& line);
    void displayLine(const std::vector<Line>& line);
    void publishTF();
    void reconfigParam();
};

#endif //SRC_DOCKER_DETECTOR_HPP
