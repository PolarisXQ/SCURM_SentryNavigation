#include "rm_decision_cpp/behaviors/attack.hpp"
#include <cmath>
#include <string>
#include <vector>

using namespace BT;
namespace rm_decision
{
  Attack::Attack(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener)
      : SyncActionNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener),
        target_tracking_(false), obs_pcl_received_(false), vehicle_pose_received_(false)
  {
    node_->get_parameter_or<double>("attack_distance", distance_to_target_, 3.0);
    node_->get_parameter_or<double>("vehicle_dim", vehicle_dim_, 1.0);
    node_->get_parameter_or<std::string>("global_frame", global_frame_, "map");
    node_->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
    node_->get_parameter_or<std::string>("obs_pcl_topic", obs_pcl_topic_, "/terrain_map");
    node_->get_parameter_or<int>("min_obs_num", min_obs_num_, 2);
    node_->get_parameter_or<double>("obs_intensity_threshold", obs_intensity_threshold_, 0.2);
    node_->get_parameter_or<bool>("use_costmap", use_costmap_, true);
    node_->get_parameter_or<int>("occ_threshold", occ_threshold_, 50);
    node_->get_parameter_or<double>("cut_radius", cut_radius_, 0.3);

    if(use_costmap_)
    {
      local_costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", rclcpp::SensorDataQoS(), std::bind(&Attack::local_costmap_callback_, this, std::placeholders::_1));
      global_costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/global_costmap/costmap", rclcpp::SensorDataQoS(), std::bind(&Attack::global_costmap_callback_, this, std::placeholders::_1));
    }
    else{
      obs_pcl_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(obs_pcl_topic_, rclcpp::SensorDataQoS(), std::bind(&Attack::obs_pcl_callback_, this, std::placeholders::_1));
      state_estimation_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 100, std::bind(&Attack::state_estimation_callback_, this, std::placeholders::_1));
      croped_pcl_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/croped_pcl", 10);
    }

    attack_pose_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/attack_pose_vis", 1);
#ifdef DEBUG
    occ_point_vis_pub_local_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/occ_point_vis_local_", 1);
    occ_point_vis_pub_global_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/occ_point_vis_global_", 1);
#endif
    final_attack_pose_vis_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/final_attack_pose_vis", 1);
    
    double circle = 2 * 3.14159 * distance_to_target_;
    pose_candidate_num = ceil(circle / vehicle_dim_);

    surrouding_obs_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    obs_pcl_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    surrouding_obs_->clear();
    obs_pcl_->clear();
  }

  NodeStatus Attack::tick()
  {
    // get target position
    auto target_position = getInput<geometry_msgs::msg::PoseStamped>("target_position");
    if (!target_position)
    {
      throw RuntimeError("error reading port [target_position]:", target_position.error());
    }
    target_point_ = target_position.value().pose;

    // get robot pose
    geometry_msgs::msg::PoseStamped pose_in_map;
    double transform_tolerance_ = 0.1;
    if (!nav2_util::getCurrentPose(
            pose_in_map, *tf_buffer_, "map", base_frame_,
            transform_tolerance_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Initial robot pose is not available.");
      return NodeStatus::FAILURE;
    }

    if (use_costmap_)
    {
      // check msg
      if (!costmap.info.width || !global_costmap.info.width)
      {
        RCLCPP_WARN(rclcpp::get_logger("Attack"), "Waiting for costmap");
        return NodeStatus::FAILURE;
      }

      vehicle_X_ = pose_in_map.pose.position.x;
      vehicle_Y_ = pose_in_map.pose.position.y;
    }
    else
    {
      // check msg
      if (!obs_pcl_received_ || !vehicle_pose_received_)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Waiting for target, obs or vehicle pose");
        return NodeStatus::FAILURE;
      }

      // crop surrounding obstacles
      surrouding_obs_->clear();
      for (long unsigned int i = 0; i < obs_pcl_->points.size(); i++)
      {
        float pointX = obs_pcl_->points[i].x;
        float pointY = obs_pcl_->points[i].y;
        // float pointZ = obs_pcl_->points[i].z;
        float dis = sqrt(pow(pointX - target_point_.position.x, 2) + pow(pointY - target_point_.position.y, 2));
        if (dis < distance_to_target_)
        {
          surrouding_obs_->points.push_back(obs_pcl_->points[i]);
        }
      }
      sensor_msgs::msg::PointCloud2 croped_pcl;
      pcl::toROSMsg(*surrouding_obs_, croped_pcl);
      croped_pcl.header.frame_id = global_frame_;
      croped_pcl.header.stamp = node_->now();
      croped_pcl_pub_->publish(croped_pcl);
    }

    // generate pose_candidate_num points around target point as candidate attack poses
    RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Generating %d candidate attack poses", pose_candidate_num);
    std::vector<geometry_msgs::msg::Pose> attack_poses;
    visualization_msgs::msg::MarkerArray attack_poses_vis;
    double alpha = atan2(vehicle_Y_ - target_point_.position.y, vehicle_X_ - target_point_.position.x);
    for (int i = 0; i < pose_candidate_num; i++)
    {
      geometry_msgs::msg::Pose attack_pose;
      attack_pose.position.x = distance_to_target_ * cos(2 * 3.14159 / pose_candidate_num * i + alpha ) + target_point_.position.x;
      attack_pose.position.y = distance_to_target_ * sin(2 * 3.14159 / pose_candidate_num * i + alpha ) + target_point_.position.y;
      attack_poses.push_back(attack_pose);
      visualization_msgs::msg::Marker attack_pose_vis;
      attack_pose_vis.header.frame_id = global_frame_;
      attack_pose_vis.header.stamp = node_->now();
      attack_pose_vis.ns = "attack_pose_vis";
      attack_pose_vis.id = i;
      attack_pose_vis.type = visualization_msgs::msg::Marker::SPHERE;
      attack_pose_vis.action = visualization_msgs::msg::Marker::ADD;
      attack_pose_vis.pose.position.x = attack_pose.position.x;
      attack_pose_vis.pose.position.y = attack_pose.position.y;
      attack_pose_vis.pose.position.z = 0;
      attack_pose_vis.pose.orientation.x = 0;
      attack_pose_vis.pose.orientation.y = 0;
      attack_pose_vis.pose.orientation.z = 0;
      attack_pose_vis.pose.orientation.w = 1;
      attack_pose_vis.scale.x = 0.1;
      attack_pose_vis.scale.y = 0.1;
      attack_pose_vis.scale.z = 0.1;
      attack_pose_vis.color.a = 1.0;
      // the closer to the vehicle, the redder the marker
      attack_pose_vis.color.r = 0.3 + 0.7 * fabs(pose_candidate_num / 2 - i) / (pose_candidate_num / 2);
      attack_pose_vis.color.g = 0.0;
      attack_pose_vis.color.b = 0.0;
      attack_poses_vis.markers.push_back(attack_pose_vis);
    }
    RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "%ld candidate attack poses generated", attack_poses.size());

#ifdef DEBUG
    // visualization
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "occ_space_global";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = global_costmap.info.resolution;
    marker.scale.y = global_costmap.info.resolution;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
#endif
    std::vector<int> obs_num(pose_candidate_num, 0);
    if (use_costmap_)
    {
      for (unsigned int i = 0; i < global_costmap.info.width; i++)
      {
        for (unsigned int j = 0; j < global_costmap.info.height; j++)
        {
          auto global_costmap_index = i + j * global_costmap.info.width;
          auto x = i * global_costmap.info.resolution + global_costmap.info.origin.position.x;
          auto y = j * global_costmap.info.resolution + global_costmap.info.origin.position.y;
          auto distance_to_center = std::hypot(x - target_point_.position.x, y - target_point_.position.y);
          if (distance_to_center <= distance_to_target_ && distance_to_center > cut_radius_)
          {
            double theta = atan2(y - target_point_.position.y, x - target_point_.position.x);
            double delta = theta - alpha;
            if (delta < 0)
            {
              delta += 2 * 3.14159;
            }
            if (delta > 2 * 3.14159)
            {
              delta -= 2 * 3.14159;
            }
            double shim = 3.14159 / pose_candidate_num; //=2*3.14159/pose_candidate_num/2;
            int index = round((delta + shim) / (2 * 3.14159 / pose_candidate_num));
            if (index >= pose_candidate_num)
            {
              index = 0;
            }
            obs_num[index] += global_costmap.data[global_costmap_index]/occ_threshold_;
            if(global_costmap.data[global_costmap_index]/occ_threshold_){
              // the more points in the direction, the bigger the marker
              attack_poses_vis.markers[index].scale.x += 0.01;
              if (attack_poses_vis.markers[index].scale.x >= vehicle_dim_)
                attack_poses_vis.markers[index].scale.x = vehicle_dim_;
              attack_poses_vis.markers[index].scale.y += 0.01;
              if (attack_poses_vis.markers[index].scale.y >= vehicle_dim_)
                attack_poses_vis.markers[index].scale.y = vehicle_dim_;
              attack_poses_vis.markers[index].scale.z += 0.01;
              if (attack_poses_vis.markers[index].scale.z >= vehicle_dim_)
                attack_poses_vis.markers[index].scale.z = vehicle_dim_;
#ifdef DEBUG
              // visual
              marker.points.push_back(geometry_msgs::msg::Point());
              marker.points.back().x = x;
              marker.points.back().y = y;
              marker.points.back().z = 0;
#endif
            }
          }
        }
      }
      // attack_pose_vis_pub_->publish(attack_poses_vis);
#ifdef DEBUG
      markers.markers.push_back(marker);
      markers.markers[0].header.stamp = node_->now();
      markers.markers[0].header.frame_id = global_frame_;
      markers.markers[0].id = 0;
      markers.markers[0].points.resize(marker.points.size());
      markers.markers[0].colors.resize(marker.points.size());
      for(size_t i = 0; i < marker.points.size(); i++){
        markers.markers[0].points[i] = marker.points[i];
        markers.markers[0].colors[i].r = 1.0;
        markers.markers[0].colors[i].g = 0.0;
        markers.markers[0].colors[i].b = 0.0;
        markers.markers[0].colors[i].a = 1.0;
      }
      occ_point_vis_pub_global_->publish(markers);


      // visualization
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = node_->now();
      marker.ns = "occ_space_local";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = costmap.info.resolution;
      marker.scale.y = costmap.info.resolution;
      marker.color.r = 1.0;
      marker.color.a = 1.0;
#endif
      for (unsigned int i = 0; i < costmap.info.width; i++)
      {
        for (unsigned int j = 0; j < costmap.info.height; j++)
        {
          auto costmap_index = i + j * costmap.info.width;
          auto x = i * costmap.info.resolution + costmap.info.origin.position.x;
          auto y = j * costmap.info.resolution + costmap.info.origin.position.y;
          auto distance_to_center = std::hypot(x - target_point_.position.x, y - target_point_.position.y);
          if (distance_to_center <= distance_to_target_ && distance_to_center > cut_radius_)
          {
            double theta = atan2(y - target_point_.position.y, x - target_point_.position.x);
            double delta = theta - alpha;
            if (delta < 0)
            {
              delta += 2 * 3.14159;
            }
            if (delta > 2 * 3.14159)
            {
              delta -= 2 * 3.14159;
            }
            int index = round(delta / (2 * 3.14159 / pose_candidate_num));
            if (index == pose_candidate_num)
            {
              index = 0;
            }
            obs_num[index] += costmap.data[costmap_index]/occ_threshold_;
            if(costmap.data[costmap_index]/occ_threshold_){
              // the more points in the direction, the bigger the marker
              attack_poses_vis.markers[index].scale.x += 0.01;
              if (attack_poses_vis.markers[index].scale.x >= vehicle_dim_)
                attack_poses_vis.markers[index].scale.x = vehicle_dim_;
              attack_poses_vis.markers[index].scale.y += 0.01;
              if (attack_poses_vis.markers[index].scale.y >= vehicle_dim_)
                attack_poses_vis.markers[index].scale.y = vehicle_dim_;
              attack_poses_vis.markers[index].scale.z += 0.01;
              if (attack_poses_vis.markers[index].scale.z >= vehicle_dim_)
                attack_poses_vis.markers[index].scale.z = vehicle_dim_;
#ifdef DEBUG
              // visual
              marker.points.push_back(geometry_msgs::msg::Point());
              marker.points.back().x = x;
              marker.points.back().y = y;
              marker.points.back().z = 0;
#endif
            }
          }
        }
      }
      attack_pose_vis_pub_->publish(attack_poses_vis);
#ifdef DEBUG
      markers.markers.push_back(marker);
      markers.markers[0].header.stamp = node_->now();
      markers.markers[0].header.frame_id = global_frame_;
      markers.markers[0].id = 0;
      markers.markers[0].points.resize(marker.points.size());
      markers.markers[0].colors.resize(marker.points.size());
      for(size_t i = 0; i < marker.points.size(); i++){
        markers.markers[0].points[i] = marker.points[i];
        markers.markers[0].colors[i].r = 1.0;
        markers.markers[0].colors[i].g = 0.0;
        markers.markers[0].colors[i].b = 0.0;
        markers.markers[0].colors[i].a = 1.0;
      }
      occ_point_vis_pub_local_->publish(markers);
#endif
    }
    else{
      for (long unsigned int i = 0; i < surrouding_obs_->points.size(); i++)
      {
        double theta = atan2(surrouding_obs_->points[i].y - target_point_.position.y, surrouding_obs_->points[i].x - target_point_.position.x);
        double delta = theta - alpha;
        if (delta < 0)
        {
          delta += 2 * 3.14159;
        }
        if (delta > 2 * 3.14159)
        {
          delta -= 2 * 3.14159;
        }
        int index = round(delta / (2 * 3.14159 / pose_candidate_num));
        if (index == pose_candidate_num)
        {
          index = 0;
        }
        obs_num[index]++;

        // the more points in the direction, the bigger the marker
        attack_poses_vis.markers[index].scale.x += 0.01;
        if (attack_poses_vis.markers[index].scale.x >= vehicle_dim_)
          attack_poses_vis.markers[index].scale.x = vehicle_dim_;
        attack_poses_vis.markers[index].scale.y += 0.01;
        if (attack_poses_vis.markers[index].scale.y >= vehicle_dim_)
          attack_poses_vis.markers[index].scale.y = vehicle_dim_;
        attack_poses_vis.markers[index].scale.z += 0.01;
        if (attack_poses_vis.markers[index].scale.z >= vehicle_dim_)
          attack_poses_vis.markers[index].scale.z = vehicle_dim_;
      }
      attack_pose_vis_pub_->publish(attack_poses_vis);
    }

    visualization_msgs::msg::Marker final_attack_pose_vis;
    final_attack_pose_vis.header.frame_id = global_frame_;
    final_attack_pose_vis.header.stamp = node_->now();
    final_attack_pose_vis.ns = "final_attack_pose_vis";
    final_attack_pose_vis.id = 0;
    final_attack_pose_vis.type = visualization_msgs::msg::Marker::SPHERE;
    final_attack_pose_vis.action = visualization_msgs::msg::Marker::ADD;
    final_attack_pose_vis.pose.orientation.x = 0;
    final_attack_pose_vis.pose.orientation.y = 0;
    final_attack_pose_vis.pose.orientation.z = 0;
    final_attack_pose_vis.pose.orientation.w = 1;
    final_attack_pose_vis.scale.x = 0.15;
    final_attack_pose_vis.scale.y = 0.15;
    final_attack_pose_vis.scale.z = 0.15;
    final_attack_pose_vis.color.a = 1.0;
    final_attack_pose_vis.color.r = 0.0;
    final_attack_pose_vis.color.g = 1.0;
    final_attack_pose_vis.color.b = 0.0;

    geometry_msgs::msg::PoseStamped attack_pose;
    attack_pose.header.frame_id = global_frame_;
    for (long unsigned int i = 0; i <= attack_poses.size() / 4; i++) // start from the nearest pose and refuse point on the opposite side
    {
      if (obs_num[i] < min_obs_num_ )
      {
        RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Attack pose found at %ld", i);
        attack_pose.header.stamp = node_->now();
        attack_pose.pose = attack_poses[i];
        setOutput<geometry_msgs::msg::PoseStamped>("attack_pose", attack_pose);
        final_attack_pose_vis.pose.position.x = attack_pose.pose.position.x;
        final_attack_pose_vis.pose.position.y = attack_pose.pose.position.y;
        final_attack_pose_vis_pub_->publish(final_attack_pose_vis);
        return NodeStatus::SUCCESS;
      }
      if (obs_num[attack_poses.size() - i - 1] < min_obs_num_)
      {
        RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Attack pose found at %ld", attack_poses.size() - i - 1);
        attack_pose.header.stamp = node_->now();
        attack_pose.pose = attack_poses[attack_poses.size() - i - 1];
        setOutput<geometry_msgs::msg::PoseStamped>("attack_pose", attack_pose);
        final_attack_pose_vis.pose.position.x = attack_pose.pose.position.x;
        final_attack_pose_vis.pose.position.y = attack_pose.pose.position.y;
        final_attack_pose_vis_pub_->publish(final_attack_pose_vis);
        return NodeStatus::SUCCESS;
      }
    }
    RCLCPP_WARN(rclcpp::get_logger("Attack"), "No valid attack pose found, stay in place");
    attack_pose.header.stamp = node_->now();
    attack_pose.pose = pose_in_map.pose;
    setOutput<geometry_msgs::msg::PoseStamped>("attack_pose", attack_pose);
    final_attack_pose_vis.pose.position.x = attack_pose.pose.position.x;
    final_attack_pose_vis.pose.position.y = attack_pose.pose.position.y;
    final_attack_pose_vis_pub_->publish(final_attack_pose_vis);
    return NodeStatus::SUCCESS;
  }

  void Attack::obs_pcl_callback_(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI> msg_pcl;
    pcl::fromROSMsg(*msg, msg_pcl);
    // filter out points with intensity < obs_intensity_threshold_
    obs_pcl_->clear();
    for (long unsigned int i = 0; i < msg_pcl.points.size(); i++)
    {
      if (msg_pcl.points[i].intensity > obs_intensity_threshold_)
      {
        obs_pcl_->points.push_back(msg_pcl.points[i]);
      }
    }
    obs_pcl_received_ = true;
    RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Received %ld obs points", obs_pcl_->points.size());
  }

  void Attack::state_estimation_callback_(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if(use_costmap_)
      return;
    vehicle_X_ = msg->pose.pose.position.x;
    vehicle_Y_ = msg->pose.pose.position.y;
    vehicle_pose_received_ = true;
    RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Received vehicle pose");
  }

  void Attack::local_costmap_callback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    costmap = *msg;
    RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Received local costmap");
  }

  void Attack::global_costmap_callback_(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    global_costmap = *msg;
    RCLCPP_DEBUG(rclcpp::get_logger("Attack"), "Received global costmap");
  }

  PortsList Attack::providedPorts()
  {
    return {
        InputPort<geometry_msgs::msg::PoseStamped>("target_position"),
        OutputPort<geometry_msgs::msg::PoseStamped>("attack_pose"),
    };
  }
} // end namespace rm_decision