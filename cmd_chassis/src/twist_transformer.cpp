#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class TwistTransformer : public rclcpp::Node
{
public:
    TwistTransformer() : Node("twist_transformer"), tfBuffer(get_clock()), tfListener(tfBuffer)
    {
        this->declare_parameter<std::string>("chassis_frame", "chassis_link");
        this->get_parameter<std::string>("chassis_frame", chassis_frame_);
        cmd_in_yaw_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_in_yaw", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 20, std::bind(&TwistTransformer::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform(chassis_frame_, "yaw_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        auto twist_in_yaw = std::make_shared<geometry_msgs::msg::Twist>();
        twist_in_yaw->linear.x = msg->linear.x * cos(yaw) + msg->linear.y * sin(yaw);
        twist_in_yaw->linear.y = msg->linear.y * cos(yaw) - msg->linear.x * sin(yaw);
        twist_in_yaw->angular.x = msg->angular.x;
        twist_in_yaw->angular.y = msg->angular.y;
        twist_in_yaw->angular.z = msg->angular.z;

        cmd_in_yaw_publisher_->publish(*twist_in_yaw);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_in_yaw_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::string chassis_frame_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistTransformer>());
    rclcpp::shutdown();
    return 0;
}
