#include <gtest/gtest.h>
#include <chrono>

#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>

#include "rm_decision_cpp/behaviors/attack.hpp"
#include "rm_decision_cpp/behaviors/spin.hpp"
#include "rm_decision_cpp/behaviors/nav2pose.hpp"
#include "rm_decision_cpp/behaviors/navthroughposes.hpp"
#include "rm_decision_cpp/behaviors/calculate_pose.hpp"
#include "rm_decision_cpp/behaviors/topics2blackboard.hpp"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"

namespace rm_decision
{
  class TreeExecNode : public rclcpp::Node
  {
  public:
    TreeExecNode(const rclcpp::NodeOptions &options)
        : Node("tree_exec", options)
    {
      RCLCPP_INFO(this->get_logger(), "Initializing TreeExecNode");
      BehaviorTreeFactory factory;

      factory.registerNodeType<rm_decision::Nav2Pose>("Nav2Pose");
      factory.registerNodeType<rm_decision::NavThroughPoses>("NavThroughPoses");
      factory.registerNodeType<rm_decision::Spin>("Spin");
      factory.registerNodeType<rm_decision::Attack>("Attack");
      factory.registerNodeType<rm_decision::CalculatePose>("CalculatePose");
      factory.registerNodeType<rm_decision::Topics2Blackboard>("Topics2Blackboard");
      // Registering a SimpleActionNode using a function pointer.
      // You can use C++11 lambdas or std::bind
      //   factory.registerSimpleCondition("CheckBattery", [&](TreeNode&) { return CheckBattery(); });

      // You can also create SimpleActionNodes using methods of a class
      //   GripperInterface gripper;
      //   factory.registerSimpleAction("OpenGripper", [&](TreeNode&){ retzurn gripper.open(); } );
      //   factory.registerSimpleAction("CloseGripper", [&](TreeNode&){ return gripper.close(); } );
      //
      // Trees are created at deployment-time (i.e. at run-time, but only
      // once at the beginning).

      // IMPORTANT: when the object "tree" goes out of scope, all the
      // TreeNodes are destroyed
      std::string tree_xml_file = this->declare_parameter<std::string>("tree_xml_file", "");
      tree_xml_file = this->get_parameter("tree_xml_file").as_string();
      tree = factory.createTreeFromFile("/home/polaris/rm_ws/src/rm_decision_cpp/behavior_tree/my_tree.xml");

      int tick_period_milliseconds = this->declare_parameter("tick_period_milliseconds", 3000);
      tick_period_milliseconds = this->get_parameter("tick_period_milliseconds").as_int();
      timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_period_milliseconds), std::bind(&TreeExecNode::executeTick, this));
    }

    void executeTick()
    {
      RCLCPP_INFO(this->get_logger(), "executeTick %d", tick_count_++);
      tree.tickWhileRunning();
    }

  private:
    BT::Tree tree;
    std::int32_t tick_count_;
    rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace rm_decision

TEST(rm_decision_cpp, a_first_test)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rm_decision::TreeExecNode>(options);
  node.reset();
}

int main(int argc, char** argv)
{
    // testing::InitGoogle Test(&argc, argv);
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return RUN_ALL_TESTS();
}