#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
        : Node("transform_publisher_node")
    {
        this->declare_parameter<std::string>("odom_frame_id","odom");
        this->declare_parameter<std::string>("map_frame_id","map");

        this->get_parameter_or<std::string>("odom_frame_id", odom_frame_id, "odom");
        this->get_parameter_or<std::string>("map_frame_id", map_frame_id, "map");
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", 10, std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = map_frame_id;
        transform.child_frame_id = odom_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    std::string odom_frame_id, map_frame_id;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
    rclcpp::shutdown();
    return 0;
}