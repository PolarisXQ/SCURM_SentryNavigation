#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

class IMURotateNode : public rclcpp::Node
{
public:
    IMURotateNode() : Node("imu_rotate_node", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "livox/imu", 10, std::bind(&IMURotateNode::listener_callback, this, std::placeholders::_1));
    }

private:
    void listener_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        // Create a quaternion for the rotation
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(3.14159, 0, 0);  // 180 degrees in radians

        // Rotate the orientation of the IMU data
        msg->orientation.x = 0.0;
        msg->orientation.y = 0.0;
        msg->orientation.z = 0.0;
        msg->orientation.w = 1.0;

        msg->angular_velocity.z = -msg->angular_velocity.z;

        msg->linear_acceleration.y = -msg->linear_acceleration.y;
        msg->linear_acceleration.z = -msg->linear_acceleration.z;

        // Publish the rotated IMU data
        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMURotateNode>());
    rclcpp::shutdown();
    return 0;
}
