#ifndef RM_DECISION_ANTIAUTOAIM_HPP_
#define RM_DECISION_ANTIAUTOAIM_HPP_
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace BT;
namespace rm_decision
{
  class AntiAutoAim : public StatefulActionNode
  {
  public:
    AntiAutoAim(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

    // this function is invoked once at the beginning.
    NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

    static PortsList providedPorts();

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Node::SharedPtr node_;
    float msec_;
    float speed_;    
    TimerQueue<> timer_;
    uint64_t timer_id_;
    rclcpp::Time start_time_;


    std::atomic_bool timer_waiting_;
    std::mutex delay_mutex_;
  };
} // end namespace rm_decision

#endif