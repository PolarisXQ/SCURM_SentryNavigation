#include "rm_decision_cpp/behaviors/attack.hpp"
#include "rm_decision_cpp/behaviors/spin.hpp"
#include "rm_decision_cpp/behaviors/nav2pose.hpp"
#include "rm_decision_cpp/behaviors/topics2blackboard.hpp"
#include "rm_decision_cpp/behaviors/anti_autoaim.hpp"
#include "rm_decision_cpp/behaviors/sentry_cmd.hpp"
#include "rm_decision_cpp/behaviors/control_gimbal.hpp"
#include "rm_decision_cpp/behaviors/align_chassis.hpp"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <chrono>
#include "rm_decision_cpp/custume_types.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tree_exec");
  // tf
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  //declare parameters
  node->declare_parameter<std::string>("tree_xml_file", "");
  node->declare_parameter("tick_period_milliseconds", 3000);
  node->declare_parameter("groot_port", 5556);
  node->declare_parameter("tree_node_model_export_path","");
  // topics to blackboard node's param
  node->declare_parameter<double>("tracking_timeout_s", 1.0);
  node->declare_parameter("game_state_topic_name", "game_state");
  node->declare_parameter("target_topic_name", "/tracker/target");
  // nav2pose node's param
  node->declare_parameter("send_goal_timeout_ms",1000);
  // attack node's param
  node->declare_parameter<double>("attack_distance", 3.0);
  node->declare_parameter<double>("vehicle_dim", 1.0);
  node->declare_parameter<std::string>("global_frame", "map");
  node->declare_parameter<std::string>("base_frame", "base_link");
  node->declare_parameter<std::string>("obs_pcl_topic", "/FAR_obs_debug");
  node->declare_parameter<int>("min_obs_num", 2);
  node->declare_parameter<double>("obs_intensity_threshold", 0.2);
  node->declare_parameter<bool>("use_costmap", true);
  node->declare_parameter<int>("occ_threshold", 50);
  node->declare_parameter<double>("cut_radius", 0.3);
  // spin
  node->declare_parameter<double>("stop_spin_slope_degree_thre", 14.0);
  //get parameters
  std::string tree_xml_file;
  node->get_parameter("tree_xml_file",tree_xml_file);
  int tick_period_milliseconds;
  node->get_parameter("tick_period_milliseconds",tick_period_milliseconds);
  std::chrono::system_clock::duration timeout;
  timeout = std::chrono::milliseconds(tick_period_milliseconds);
  unsigned int groot_port = 5556;
  node->get_parameter("groot_port",groot_port);
  std::string tree_node_model_export_path;
  node->get_parameter("tree_node_model_export_path",tree_node_model_export_path);

  BT::BehaviorTreeFactory factory;

  // Register the custom nodes
  factory.registerNodeType<rm_decision::Nav2Pose>("Nav2Pose", node);
  factory.registerNodeType<rm_decision::AntiAutoAim>("AntiAutoAim",node);
  factory.registerNodeType<rm_decision::Spin>("Spin",node);
  factory.registerNodeType<rm_decision::Attack>("Attack",node,tf_buffer,tf_listener);
  factory.registerNodeType<rm_decision::Topics2Blackboard>("Topics2Blackboard",node,tf_buffer,tf_listener);
  factory.registerNodeType<rm_decision::ControlGimbal>("ControlGimbal",node,tf_buffer,tf_listener);
  factory.registerNodeType<rm_decision::SentryCmd>("SentryCmd",node);
  factory.registerNodeType<rm_decision::AlignChassis>("AlignChassis",node,tf_buffer,tf_listener);
  RCLCPP_INFO(node->get_logger(), "Loaded all custom nodes");

  // Visualize custom types in the Blackboard
  BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>(PoseStampedToJson);
  BT::RegisterJsonDefinition<geometry_msgs::msg::PointStamped>(PointStampedToJson);


  // generate xml file
  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  // save to file
  std::ofstream file(tree_node_model_export_path);
  file << xml_models;
  file.close();
  RCLCPP_INFO(node->get_logger(), "Generated XML file");
  
  auto tree = factory.createTreeFromFile(tree_xml_file.c_str());
  std::shared_ptr<BT::Groot2Publisher> groot2publisher_ptr_;
  groot2publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree, groot_port);

  unsigned long int tick_count = 0;
  while(rclcpp::ok()){
    tick_count++;
    RCLCPP_INFO(node->get_logger(), "----------Tick %lu---------", tick_count);
    rclcpp::spin_some(node);
    tree.tickOnce();
    tree.sleep(timeout);
  }

  rclcpp::shutdown();
  return 0;
}