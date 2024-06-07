#ifndef RM_DECISION_ATTACK_HPP_
#define RM_DECISION_ATTACK_HPP_

#define DEBUG

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/qos.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rm_decision_cpp/custume_types.hpp"

#include <memory>
#include <optional>
#include <time.h>

#include <nav2_util/robot_utils.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using namespace BT;
namespace rm_decision
{
  class Attack : public SyncActionNode
  {
  public:
    Attack(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener);

    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obs_pcl_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimation_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr attack_pose_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr final_attack_pose_vis_pub_;
#ifdef DEBUG
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_point_vis_pub_local_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occ_point_vis_pub_global_;
#endif
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr croped_pcl_pub_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
    // tf
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    
    
    bool target_tracking_,obs_pcl_received_,vehicle_pose_received_;
    double distance_to_target_,vehicle_dim_;
    double vehicle_X_,vehicle_Y_;
    int pose_candidate_num,min_obs_num_;
    double obs_intensity_threshold_;
    std::string global_frame_, base_frame_;
    std::string obs_pcl_topic_,target_topic_;
    auto_aim_interfaces::msg::Target target_info_;
    geometry_msgs::msg::Pose target_point_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr obs_pcl_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surrouding_obs_;
    bool use_costmap_;
    int occ_threshold_;
    double cut_radius_;
    nav_msgs::msg::OccupancyGrid costmap, global_costmap;

    void obs_pcl_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void state_estimation_callback_(const nav_msgs::msg::Odometry::SharedPtr msg);
    void local_costmap_callback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void global_costmap_callback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  };
} // end namespace rm_decision
#endif