#include "rm_decision_cpp/behaviors/anti_autoaim.hpp"

using namespace BT;
namespace rm_decision
{
  AntiAutoAim::AntiAutoAim(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : StatefulActionNode(name, config), node_(node)
  {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  }

  NodeStatus AntiAutoAim::onStart()
  {
    // rclcpp::spin_some(node_);
    if (!getInput("msec", msec_))
    {
      msec_ = 4;
    }

    if (!getInput("speed", speed_))
    {
      speed_ = 1.0;
    }

    // create a timestamp
    start_time_ = node_->now();

    return NodeStatus::RUNNING;
  }

  NodeStatus AntiAutoAim::onRunning()
  {
    // rclcpp::spin_some(node_);
    auto now = node_->now();
    auto elapsed_time = (now - start_time_).seconds();
    float now_speed = speed_;
    geometry_msgs::msg::Twist cmd_vel_msg;
    RCLCPP_DEBUG(node_->get_logger(),"elapsed_time: %f", elapsed_time);
    double angle = 30.0/180.0*3.14159;
    if (elapsed_time < msec_/4)
    {
      if(elapsed_time < msec_/16 || elapsed_time > 3*msec_/16)
        now_speed*=0.5;
      cmd_vel_msg.linear.y = now_speed;
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::RUNNING;
    }
    else if(elapsed_time < msec_/2)
    {
      if(elapsed_time < 5*msec_/16 || elapsed_time > 7*msec_/16)
        now_speed*=0.5;
      cmd_vel_msg.linear.y = -now_speed*cos(angle)/cos(angle);
      cmd_vel_msg.linear.x = -now_speed*sin(angle)/cos(angle); 
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::RUNNING;
    }
    else if(elapsed_time < 3*msec_/4)
    {
      if(elapsed_time < 9*msec_/16 || elapsed_time > 11*msec_/16)
        now_speed*=0.5;
      cmd_vel_msg.linear.y = now_speed;
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::RUNNING;
    }
    else if(elapsed_time < msec_)
    {
      if(elapsed_time < 13*msec_/16 || elapsed_time > 15*msec_/16)
        now_speed*=0.5;
      cmd_vel_msg.linear.y = -now_speed*cos(angle)/cos(angle);
      cmd_vel_msg.linear.x = now_speed*sin(angle)/cos(angle);
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::RUNNING;
    }
    else
    {
      cmd_vel_pub_->publish(cmd_vel_msg);
      return NodeStatus::SUCCESS;
    }

  }
  

  void AntiAutoAim::onHalted()
  {
    // stop the robot
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  PortsList AntiAutoAim::providedPorts()
  {
    return {InputPort<float>("msec"),
            InputPort<float>("speed")
            };
  }
} // end namespace rm_decision