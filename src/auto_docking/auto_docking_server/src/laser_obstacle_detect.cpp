#include "auto_docking/laser_obstacle_detect.h"
LaserObstacleDetect::LaserObstacleDetect()
    : Node("laser_obstacle_detect"),
      tfBuffer_(this->get_clock()),
      tfListener_(tfBuffer_)
{
    m_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 50, std::bind(&LaserObstacleDetect::scanCallBack, this, std::placeholders::_1));
    m_obstacle_time = this->now() - rclcpp::Duration::from_seconds(1.5);
    m_robot_radius_sq = 0.23 * 0.23;
}

void LaserObstacleDetect::scanCallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    m_scan = *msg;
}

bool LaserObstacleDetect::cvtLaserScan2Pointcloud(const sensor_msgs::msg::LaserScan &scan,
                                                  sensor_msgs::msg::PointCloud2 &cloud)
{
    float epsilon = 0.0001;
    sensor_msgs::msg::LaserScan message = scan;

    // 修正无效激光数据
    for (size_t i = 0; i < message.ranges.size(); i++) {
        float range = message.ranges[i];
        if (!std::isfinite(range) || range < epsilon) {
            message.ranges[i] = message.range_max - epsilon;
        }
    }

    // 投影到 PointCloud2
    try {
        m_projector.transformLaserScanToPointCloud("base_link", message, cloud, tfBuffer_);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "transformLaserScanToPointCloud exception: %s", ex.what());
        return false;
    }
    return true;
}

bool LaserObstacleDetect::obstacleDetect(const sensor_msgs::msg::PointCloud2 &cloud2,
                                         const geometry_msgs::msg::Twist &twist)
{
    
    //sensor_msgs::msg::PointCloud cloud;  //ros2不再支持PointCloud，只能通過迭代器遍歷點雲
    float sim_time = 0.1;
    float robot_x = 0.0, robot_y = 0.0, robot_yaw = 0.0;

    for (size_t i = 0; i < 20; ++i) {
        robot_x += twist.linear.x * cos(robot_yaw) * sim_time;
        robot_y += twist.linear.y * sin(robot_yaw) * sim_time;
        robot_yaw += twist.angular.z * sim_time;

        int discontinuities_tolerance = 3;
        int close_point = 0, discontinuities = 0;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud2, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud2, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            float sq_dist = pow(*iter_x - robot_x, 2) + pow(*iter_y - robot_y, 2);

            if (sq_dist <= m_robot_radius_sq) {
                ++close_point;
            } else if (close_point < 3) {
                close_point = 0;
            } else if (discontinuities < discontinuities_tolerance) {
                ++close_point;
                ++discontinuities;
            } else {
                close_point = 0;
                discontinuities = 0;
            }

            if (close_point > 10) {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected at step %zu", i);
                return true;
            }
        }
    }
    return false;
}

bool LaserObstacleDetect::haveObstacle(const geometry_msgs::msg::Twist &twist)
{
    sensor_msgs::msg::PointCloud2 cloud;
    if (!cvtLaserScan2Pointcloud(m_scan, cloud)) {
        m_obstacle_time = this->get_clock()->now();
        return true;
    }

    if (obstacleDetect(cloud, twist)) {
        m_obstacle_time = this->get_clock()->now();
        return true;
    } else {
        m_free_time = this->get_clock()->now();
        return (m_free_time - m_obstacle_time).seconds() <= 1.0;
    }
}

void LaserObstacleDetect::setRobotRadius(float radius)
{
    m_robot_radius_sq = radius * radius;
}

