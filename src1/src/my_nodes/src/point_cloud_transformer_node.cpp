#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp" // Updated include

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer()
    : Node("point_cloud_transformer"),
      tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
      tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10, std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_points", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            // Using std::chrono::seconds for duration
            transform_stamped = tf_buffer_.lookupTransform("base_link", "camera_link_optical", msg->header.stamp, std::chrono::seconds(1));
            sensor_msgs::msg::PointCloud2 transformed_msg;
            tf2::doTransform(*msg, transformed_msg, transform_stamped);
            publisher_->publish(transformed_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}
