#include "docker_detector/docker_detector.hpp"
#include <visualization_msgs/msg/marker.hpp>


DOCKER::DOCKER() : Node("docker_detector_node")
{
    // 声明参数
    this->declare_parameter("pub_visual_marker", true);
    this->declare_parameter("lidar_reverse", false);

    this->declare_parameter<int>("min_point_num", 5);
    this->declare_parameter<int>("min_index_num", 5);
    this->declare_parameter<double>("max_gap_dist", 0.1);
    this->declare_parameter<double>("in_line_threshold", 0.05);
    this->declare_parameter<double>("docker_line_len", 0.13);
    this->declare_parameter<double>("docker_angle_diff", 120.0);
    this->declare_parameter<double>("docker_distance_threshold", 0.5);
    this->declare_parameter<double>("match_len_tolerance", 0.03);
    this->declare_parameter<double>("match_angle_tolerance", 3.0);
    this->declare_parameter<double>("laser_yaw_calibration", 0.0);
    this->declare_parameter<std::string>("laser_frame", "laser");
    this->declare_parameter<std::string>("docker_frame", "docker");
    this->declare_parameter<double>("intensity_min", 150.0);
    this->declare_parameter<double>("intensity_max", 290.0);

    // 获取参数
    pub_visual_marker = this->get_parameter("pub_visual_marker").as_bool();
    lidar_reverse_ = this->get_parameter("lidar_reverse").as_bool();
    min_point_num = this->get_parameter("min_point_num").as_int();
    min_index_num = this->get_parameter("min_index_num").as_int();
    max_gap_dis = this->get_parameter("max_gap_dist").as_double();
    in_line_threshold = this->get_parameter("in_line_threshold").as_double();
    docker_line_len = this->get_parameter("docker_line_len").as_double();
    docker_angle_diff = this->get_parameter("docker_angle_diff").as_double();
    docker_distance_threshold = this->get_parameter("docker_distance_threshold").as_double();
    match_len_tolerance = this->get_parameter("match_len_tolerance").as_double();
    match_angle_tolerance = this->get_parameter("match_angle_tolerance").as_double();
    laser_yaw_calib_ = this->get_parameter("laser_yaw_calibration").as_double();
    laser_frame = this->get_parameter("laser_frame").as_string();
    docker_frame = this->get_parameter("docker_frame").as_string();
    intensity_min = this->get_parameter("intensity_min").as_double();
    intensity_max = this->get_parameter("intensity_max").as_double();

    // 创建 LineSplit
    lineSplit_ = new LineSplit(min_point_num, max_gap_dis);

    // ROS2 pub/sub
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&DOCKER::laser_CallBack, this, std::placeholders::_1));

    line_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("docker_vis", 10);
    docker_info_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("docker_pose", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "docker_line_len: %.3f", docker_line_len);
    RCLCPP_INFO(this->get_logger(), "docker_angle_diff: %.3f", docker_angle_diff);
}

DOCKER::~DOCKER()
{
    if (lineSplit_ != nullptr)
        delete lineSplit_;
    lineSplit_ = nullptr;
}

void DOCKER::laser_CallBack(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    vector<Point> point_cloud;
    vector<Point> point_cloud_temp;

    laser_frame = msg->header.frame_id;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        if (msg->intensities[i] < intensity_min || msg->intensities[i] > intensity_max)
            continue;
        if (msg->ranges[i] < msg->range_min || msg->ranges[i] > (msg->range_max - 0.1))
            continue;

        double theta = msg->angle_min + msg->angle_increment * i;
        Point p;
        p.x = msg->ranges[i] * cos(theta);
        p.y = msg->ranges[i] * sin(theta);
        point_cloud.push_back(p);
    }

    std::reverse(point_cloud.begin(), point_cloud.end());
    if (point_cloud.size() > 10)
        point_cloud_temp.assign(point_cloud.begin() + 2, point_cloud.end() - 2);
    else
        point_cloud_temp = point_cloud;

    vector<Line> lines = lineSplit_->LineSplitWithRDP(point_cloud_temp, in_line_threshold);
    if (lines.size() < 2)
        return;

    docker_match(lines);
}

void DOCKER::displayLine(const vector<Line> &line)
{
    if (!pub_visual_marker)
        return;

    //visualization_msgs::msg::Marker marker;
    //marker.action = Marker::ADD;
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.color.a = 1.0f;
    marker.color.r = 1.0f;
    marker.header.frame_id = laser_frame;
    marker.header.stamp = this->now();
    marker.id = 1;
    marker.ns = "marker";
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    //marker.type = Marker::LINE_LIST;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;

    for (auto &l : line)
    {
        geometry_msgs::msg::Point p;
        p.x = l.start_point.x;
        p.y = l.start_point.y;
        marker.points.push_back(p);

        p.x = l.end_point.x;
        p.y = l.end_point.y;
        marker.points.push_back(p);
    }
    line_marker_->publish(marker);
}

void DOCKER::docker_match(const vector<Line> &lines)
{
    std::vector<geometry_msgs::msg::PoseStamped> docker_vector_;
    docker_vector_.clear();

    for (size_t i = 0; i < lines.size() - 1; i++)
    {
        if (fabs(lines[i].len - docker_line_len) < match_len_tolerance &&
            fabs(lines[i + 1].len - docker_line_len) < match_len_tolerance)
        {
            double corner_dist = hypot(lines[i].start_point.x - lines[i + 1].end_point.x,
                                       lines[i].start_point.y - lines[i + 1].end_point.y);
            if (corner_dist < 0.05)
            {
                double angle_diff;
                if (lidar_reverse_)
                    angle_diff = angles::normalize_angle(M_PI - lines[i].theta + lines[i + 1].theta);
                else
                    angle_diff = -angles::normalize_angle(M_PI - lines[i].theta + lines[i + 1].theta);

                if (fabs(angles::to_degrees(angle_diff) - docker_angle_diff) < match_angle_tolerance)
                {
                    vector<Line> dock_marker = {lines[i], lines[i + 1]};
                    displayLine(dock_marker);
                    calcDockerPose(dock_marker);
                    docker_vector_.push_back(docker_pose_);
                    i++;
                }
            }
        }
    }

    if (docker_vector_.size() == 1)
    {
        docker_pose_ = docker_vector_.front();
        docker_info_->publish(docker_pose_);
        publishTF();
    }
    else if (docker_vector_.size() > 1)
    {
        docker_pose_ = docker_vector_.front();
        for (auto &pose : docker_vector_)
        {
            if (pose.pose.position.z < docker_pose_.pose.position.z)
                docker_pose_ = pose;
        }
        docker_info_->publish(docker_pose_);
        publishTF();
    }
}

void DOCKER::calcDockerPose(const vector<Line> &line)
{
    docker_pose_.header.frame_id = docker_frame;
    docker_pose_.header.stamp = this->now();

    docker_pose_.pose.position.x = (line[1].start_point.x + line[0].end_point.x) / 2;
    docker_pose_.pose.position.y = (line[1].start_point.y + line[0].end_point.y) / 2;
    docker_pose_.pose.position.z = hypot(docker_pose_.pose.position.x, docker_pose_.pose.position.y);

    double x_diff = line[1].start_point.x - line[0].end_point.x;
    double y_diff = line[1].start_point.y - line[0].end_point.y;
    double theta;
    if (lidar_reverse_)
        theta = angles::normalize_angle(atan2(y_diff, x_diff) - M_PI_2);
    else
        theta = angles::normalize_angle(atan2(y_diff, x_diff) + M_PI_2);

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    docker_pose_.pose.orientation.x = q.x();
    docker_pose_.pose.orientation.y = q.y();
    docker_pose_.pose.orientation.z = q.z();
    docker_pose_.pose.orientation.w = q.w();
}

void DOCKER::publishTF()
{
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = laser_frame;
    transformStamped.child_frame_id = docker_frame;

    transformStamped.transform.translation.x = docker_pose_.pose.position.x;
    transformStamped.transform.translation.y = docker_pose_.pose.position.y;
    transformStamped.transform.translation.z = 0;

    transformStamped.transform.rotation = docker_pose_.pose.orientation;
    tf_broadcaster_->sendTransform(transformStamped);
}

void DOCKER::reconfigParam()
{
    // 可添加动态参数更新逻辑
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DOCKER>());
    rclcpp::shutdown();
    return 0;
}
