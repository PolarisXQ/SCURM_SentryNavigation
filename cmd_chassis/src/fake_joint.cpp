#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
    : Node("transform_publisher_node")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "state_estimation", 200,
            std::bind(&TransformPublisherNode::listener_callback, this, std::placeholders::_1));
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto transform = std::make_shared<geometry_msgs::msg::TransformStamped>();

        transform->header.stamp = this->now();
        transform->header.frame_id = "odom";
        transform->child_frame_id = "chassis_link";
        transform->transform.translation.x = msg->pose.pose.position.x;
        transform->transform.translation.y = msg->pose.pose.position.y;
        transform->transform.translation.z = msg->pose.pose.position.z;
        transform->transform.rotation.x = 0.0;
        transform->transform.rotation.y = 0.0;
        transform->transform.rotation.z = 0.0;
        transform->transform.rotation.w = 1.0;

        // Publish the transform
        broadcaster_->sendTransform(*transform);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransformPublisherNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}