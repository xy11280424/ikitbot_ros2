#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DockerListener : public rclcpp::Node
{
public:
    DockerListener()
        : Node("docker_listener"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Hello world! DockerListener started.");
        // 定时器，10Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DockerListener::timerCallback, this));
    }

private:
    void timerCallback()
    {
        std::string err_msg;
        geometry_msgs::msg::TransformStamped in, out;

        in.header.stamp = this->now();
        in.header.frame_id = "docker";

        if (!tf_buffer_.canTransform("odom", in.header.frame_id, in.header.stamp,
                                     rclcpp::Duration::from_seconds(0.1), &err_msg))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "can't transform from odom to docker at %.3f",
                                 in.header.stamp.seconds());
            RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, err_msg);
        }
        else
        {
            try
            {
                out = tf_buffer_.transform(in, "odom");
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "got transform x %.3f, y %.3f",
                                     out.transform.translation.x,
                                     out.transform.translation.y);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failed: " << ex.what());
            }
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DockerListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
