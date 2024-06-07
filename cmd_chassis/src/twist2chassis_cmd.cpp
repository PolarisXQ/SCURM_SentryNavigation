#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rm_interfaces/msg/chassis_cmd.hpp>
#include <std_msgs/msg/int8.hpp>

class CmdChassisNode : public rclcpp::Node
{
public:
    CmdChassisNode()
    : Node("twist_to_chassis_cmd")
    {
        this->declare_parameter<std::string>("twist_topic", "cmd_vel_in_yaw");
        this->declare_parameter<std::string>("chassis_topic", "chassis_cmd");
        this->declare_parameter<std::string>("chassis_type_topic", "chassis_type");

        twist_topic_ = this->get_parameter("twist_topic").as_string();
        chassis_topic_ = this->get_parameter("chassis_topic").as_string();
        chassis_type_topic_ = this->get_parameter("chassis_type_topic").as_string();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_topic_, 10, std::bind(&CmdChassisNode::twist_cbk, this, std::placeholders::_1));
        chassis_type_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            chassis_type_topic_, 10, std::bind(&CmdChassisNode::chassis_type_cbk, this, std::placeholders::_1));

        chassis_pub_ = this->create_publisher<rm_interfaces::msg::ChassisCmd>(chassis_topic_, 10);

        chassis_msg_.type = 2;
    }

private:
    void twist_cbk(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        chassis_msg_.twist = *msg;
        chassis_pub_->publish(chassis_msg_);
    }

    void chassis_type_cbk(const std_msgs::msg::Int8::SharedPtr msg)
    {
        chassis_msg_.type = msg->data;
        chassis_pub_->publish(chassis_msg_);
    }

    std::string twist_topic_;
    std::string chassis_topic_;
    std::string chassis_type_topic_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr chassis_type_sub_;

    rclcpp::Publisher<rm_interfaces::msg::ChassisCmd>::SharedPtr chassis_pub_;

    rm_interfaces::msg::ChassisCmd chassis_msg_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdChassisNode>());
    rclcpp::shutdown();
    return 0;
}