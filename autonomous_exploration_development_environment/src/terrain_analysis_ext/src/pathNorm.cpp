#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <iostream>
#include <vector>
#include <mutex>
// pcl::visualization::PCLVisualizer viewer("PCL Viewer");

double PI = 3.14159;

class PointCloudProcessingNode : public rclcpp::Node
{
public:
    PointCloudProcessingNode()
        : Node("point_cloud_processing_node"), stateestimation_received(false),
          normal_analysis_done(false), pc_init(false), read_map_from_file(false)
    {
        this->declare_parameter<std::string>("map_file_path", "");
        this->declare_parameter<double>("vehicle_dim_Y", 0.6);
        this->declare_parameter<double>("vehicle_dim_x", 0.6);
        this->declare_parameter<double>("Z_offset", 0.6);
        this->declare_parameter<double>("remove_thre", 0.01);
        // this->declare_parameter<double>("min_stair_height", 0.1);
        this->declare_parameter<bool>("use_ror", true);
        this->declare_parameter<double>("ror_radius", 0.141);
        this->declare_parameter<bool>("remove_ceiling", true);
        this->declare_parameter<double>("ceiling_height", 2.0);
        // this->declare_parameter<double>("min_stair_degree", 30.0);
        this->declare_parameter<double>("speed_up_slope_degree_thre", 16.0);
        this->declare_parameter<double>("max_speed_up_slope_degree", 45.0);
        this->declare_parameter<double>("speed_up_max_rate", 1.5);
        this->declare_parameter<double>("speedup_duration", 0.5);
        this->declare_parameter<std::string>("base_frame", "chassis_link");
        this->declare_parameter<std::string>("global_frame", "map");
        this->declare_parameter<double>("ground_height", 0.0);

        this->get_parameter_or<std::string>("map_file_path", map_file_path, "");
        this->get_parameter_or<double>("vehicle_dim_Y", vehicle_dim_Y, 0.6);
        this->get_parameter_or<double>("vehicle_dim_x", vehicle_dim_X, 0.6);
        this->get_parameter_or<double>("Z_offset", Z_offset, 0.6);
        this->get_parameter_or<double>("remove_thre", remove_thre, 0.01);
        // this->get_parameter_or<double>("min_stair_height", min_stair_height, 0.1);
        this->get_parameter_or<bool>("use_ror", use_ror, true);
        this->get_parameter_or<double>("ror_radius", ror_radius, 0.141);
        this->get_parameter_or<bool>("remove_ceiling", remove_ceiling, true);
        this->get_parameter_or<double>("ceiling_height", ceiling_height, 2.0);
        // this->get_parameter_or<double>("min_stair_degree", min_stair_degree, 30.0);
        this->get_parameter_or<double>("speed_up_slope_degree_thre", speed_up_slope_degree_thre, 18.0);
        this->get_parameter_or<double>("max_speed_up_slope_degree", max_speed_up_slope_degree, 45.0);
        this->get_parameter_or<double>("speed_up_max_rate", speed_up_max_rate, 2.0);
        this->get_parameter_or<double>("speedup_duration", speedup_duration, 0.5);
        this->get_parameter_or<std::string>("base_frame", base_frame, "chassis_link");
        this->get_parameter_or<std::string>("global_frame", global_frame, "map");
        this->get_parameter_or<double>("ground_height", ground_height, 0.0);

        pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("path_pointcloud", 10);
        all_normal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("all_norm", 10);
        avg_normal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("avg_norm", 10);
        // Stair_normal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("stair_norm", 10);
        angDiff_publisher_ = this->create_publisher<std_msgs::msg::Float32>("angle_diff", 10);
        slope_degree_publisher_ = this->create_publisher<std_msgs::msg::Float32>("slope_degree", 10);
        speed_up_rate_publisher_ = this->create_publisher<std_msgs::msg::Float32>("speed_up", 10);
        subscription_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PointCloudProcessingNode::plan_callback, this, std::placeholders::_1));
        subscription_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/terrain_map_ext", 10, std::bind(&PointCloudProcessingNode::pointcloud_callback, this, std::placeholders::_1));
        subscription_stateestimation_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/state_estimation", 10, std::bind(&PointCloudProcessingNode::stateestimation_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize the bounding box
        boundingbox_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        surface_hull = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        objects = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_path, *cloud) == -1) // Replace with your file name
        {
            RCLCPP_WARN(this->get_logger(), "Failed to load PCD from file %s.\n", map_file_path.c_str());
            RCLCPP_WARN(this->get_logger(), "Listening to terrain_map_ext or the remapped topic for map pointcloud input.\n");
            read_map_from_file = false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Loaded PCD from file %s.\n", map_file_path.c_str());
            read_map_from_file = true;
            bb_filter.setDim(2);
            bb_filter.setInputCloud(cloud);
            pc_init = true;
        }
        hull.setDimension(2); // slove 2D convex hull
        bb_filter.setDim(2);
        normalEstimation.setRadiusSearch(0.3);

        path = std::make_shared<nav_msgs::msg::Path>();
    }

private:
    // bool stair_detection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &objects, double step_avg_normal_x, double step_avg_normal_y, double step_avg_normal_z)
    // {
    //     // ----------------------- Stair Detection -----------------------
    //     // if there are more than 2 layers of points, then it is a stair
    //     unsigned int init_point_num = objects->points.size();
    //     std::vector<double> height_list;
    //     for (long unsigned int i = 0; i < objects->points.size(); i++)
    //     {
    //         height_list.push_back(objects->points[i].z);
    //     }
    //     std::sort(height_list.begin(), height_list.end());

    //     // remove point that is remove_thre higher than the lowest point and remove_thre lower than the highest point
    //     double lowest_point = height_list[0];
    //     double highest_point = height_list[height_list.size() - 1];
    //     if (highest_point - lowest_point < min_stair_height)
    //     {
    //         return false;
    //     }
    //     /*  SLOPE
    //                                                     ..  ----
    //                                                 .....     |     <- remove_thre
    //                                             ......      ----
    //                                         ------
    //                                     ------
    //                                 ------
    //                             ------
    //                         ------
    //                     ------
    //                 .......                                 ----
    //             ......                                        |     <- remove_thre
    //         .....                                           ----
    //     */
    //     /*   STAIR
    //                              ..........................  ----
    //                             ..                             |     <- remove_thre
    //                            ..                            ----
    //                            --
    //                            --
    //                            --
    //                            --
    //                          ---
    //                          --
    //                          ..                              ----
    //                         ...                                |     <- remove_thre
    //          ..................                              ----
    //     */
    //     for (long unsigned int i = 0; i < height_list.size(); i++)
    //     {
    //         if (height_list[i] - lowest_point > remove_thre && highest_point - height_list[i] > remove_thre)
    //         {
    //             height_list.erase(height_list.begin() + i);
    //             i--;
    //         }
    //     }

    //     if (height_list.size() > init_point_num / 2)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Stair detected.(MAYBE)\n");
    //         return true;
    //     }
    //     else
    //     {
    //         return false;
    //     }

    //     // ----------------------- Stair Detection -----------------------
    //     // check if there are lots of normal vectors that are pointing horizontally
    //     // so this method is only applied to a situation that the map is pre-built,
    //     // caz those points with horizontal normal is not scanned by the lidar

    //     // unsigned int init_point_num = objects->points.size();
    //     // unsigned int horizontal_normal_num = 0;
    //     // for (long unsigned int i = 0; i < normals->points.size(); i++)
    //     // {
    //     //     if (normals->points[i].normal_z < 0.1) // in angle, 0.1 is about 84.3 degree
    //     //     {
    //     //         horizontal_normal_num++;
    //     //     }
    //     // }
    // }
    void stateestimation_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!normal_analysis_done)
        {
            std_msgs::msg::Float32 slope_degree_msg;
            slope_degree_msg.data = 0.0;
            slope_degree_publisher_->publish(slope_degree_msg);
            return;
        }

        // get robot pose in global frame
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(base_frame, global_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        mtx.lock();
        tf2::Matrix3x3 m(tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w));
        double roll = 0, pitch = 0, yaw = 0;
        m.getRPY(roll, pitch, yaw);

        // find robot is in which step
        // x should be between x1 and x2
        // so does y
        // step_id is between 0 and path.size()-2
        double poseX = t.transform.translation.x;
        double poseY = t.transform.translation.y;
        double poseZ = t.transform.translation.z;

        unsigned int step_id = 0;
        RCLCPP_DEBUG(this->get_logger(), "path size: %ld.\n", path->poses.size());
        RCLCPP_DEBUG(this->get_logger(), "avg_normal_x size: %ld.\n", avg_normal_x.size());
        RCLCPP_DEBUG(this->get_logger(), "avg_normal_y size: %ld.\n", avg_normal_y.size());
        RCLCPP_DEBUG(this->get_logger(), "avg_normal_z size: %ld.\n", avg_normal_z.size());
        RCLCPP_DEBUG(this->get_logger(), "avg_height size: %ld.\n", avg_height.size());
        // RCLCPP_DEBUG(this->get_logger(), "is_stair size: %ld.\n", is_stair.size());
        for (long unsigned int i = 0; i < path->poses.size() - 1; i++)
        {
            double x1 = path->poses[i].pose.position.x;
            double y1 = path->poses[i].pose.position.y;
            double x2 = path->poses[i + 1].pose.position.x;
            double y2 = path->poses[i + 1].pose.position.y;
            RCLCPP_DEBUG(this->get_logger(), "x1: %f, y1: %f, x2: %f, y2: %f.\n", x1, y1, x2, y2);
            RCLCPP_DEBUG(this->get_logger(), "poseX: %f, poseY: %f.\n", poseX, poseY);
            if ((poseX - x1) * (poseX - x2) <= 0 && (poseY - y1) * (poseY - y2) <= 0)
            {
                step_id = i;
                break;
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "step_id: %d.\n", step_id);

        if (step_id >= avg_normal_x.size() - 2)
        {
            RCLCPP_INFO(this->get_logger(), "Robot is at the end of the path.\n");
            std_msgs::msg::Float32 slope_degree_msg;
            slope_degree_msg.data = 0.0;
            slope_degree_publisher_->publish(slope_degree_msg);
            normal_analysis_done = false;
            mtx.unlock();
            return;
        }

        double next_step_slope_degree = atan2(sqrt(pow(avg_normal_x[step_id + 1], 2) + pow(avg_normal_y[step_id + 1], 2)), avg_normal_z[step_id + 1]) * 180 / PI;
        double current_step_slope_degree = atan2(sqrt(pow(avg_normal_x[step_id], 2) + pow(avg_normal_y[step_id], 2)), avg_normal_z[step_id]) * 180 / PI;
        if (next_step_slope_degree > 90.0) // TODO:
            next_step_slope_degree = 180.0 - next_step_slope_degree;

        if (current_step_slope_degree > 90.0)
            current_step_slope_degree = 180.0 - current_step_slope_degree;

        double next_step_height = avg_height[step_id + 1];

        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Next step height: %f.\n", next_step_height);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Pose height: %f.\n", poseZ);

        if (poseZ - Z_offset > next_step_height)
        {
            next_step_slope_degree *= -1.0;
            current_step_slope_degree *= -1.0; // can not trigger the align condition
            RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Robot is going down.\n");

            // if (is_stair[step_id + 1])
            // {
            //     RCLCPP_INFO(this->get_logger(), "Next step is a stair.\n");
            //     std_msgs::msg::Float32 speed_up_rate_msg;
            //     speed_up_rate_msg.data = speed_up_max_rate;
            //     // speed_up_rate_publisher_->publish(speed_up_rate_msg);
            //     mtx.unlock();
            //     return;
            // }
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Next step slope degree: %f.\n", next_step_slope_degree);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "Current step slope degree: %f.\n", current_step_slope_degree);
        std_msgs::msg::Float32 slope_degree_msg;
        next_step_slope_degree = std::max(next_step_slope_degree, current_step_slope_degree);
        slope_degree_msg.data = next_step_slope_degree;
        slope_degree_publisher_->publish(slope_degree_msg); // publish the slope degree of the next step or current step, unit is degree

        double angle;
        try
        {
            angle = atan2(avg_normal_y[step_id + 1], avg_normal_x[step_id + 1]);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "%s", e.what());
            angle = atan2(avg_normal_y[step_id], avg_normal_x[step_id]);
        }

        RCLCPP_INFO(this->get_logger(), "path angle relative to map frame is %f\n", angle / PI * 180);
        RCLCPP_INFO(this->get_logger(), "yaw angle of the vehicle is %f\n", yaw / PI * 180);
        RCLCPP_INFO(this->get_logger(), "angle difference is %f\n", (angle - yaw) / PI * 180);

        double angDiff = angle - yaw;

        while (angDiff > PI / 4)
        {
            angDiff -= PI / 2;
        }
        while (angDiff < -PI / 4)
        {
            angDiff += PI / 2;
        }

        RCLCPP_INFO(this->get_logger(), "min angle difference is %f\n", angDiff / PI * 180);

        if (next_step_slope_degree > speed_up_slope_degree_thre && next_step_slope_degree < max_speed_up_slope_degree)
        {
            std_msgs::msg::Float32 speed_up_rate_msg;
            speed_up_rate_msg.data = (next_step_slope_degree / max_speed_up_slope_degree + 1) * speed_up_max_rate;
            speed_up_rate_publisher_->publish(speed_up_rate_msg);

            if(poseZ > ground_height)
            {
                angDiff = 0;
                RCLCPP_INFO(this->get_logger(), "Robot is on the slope. Disable alignment.\n");
            }
            std_msgs::msg::Float32 min_angDiff_msg;
            min_angDiff_msg.data = angDiff;
            angDiff_publisher_->publish(min_angDiff_msg); // publish the angle difference between the vehicle and the slope, unit is rad
        }
        else
        {
            std_msgs::msg::Float32 speed_up_rate_msg;
            speed_up_rate_msg.data = 1.0;
            speed_up_rate_publisher_->publish(speed_up_rate_msg);
        }
        mtx.unlock();
    }

    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // path.size = path length / vehicle_dim_X
        // avg_normal_xyz.size()/avg_height.size = path.size - 1

        if (!pc_init)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for pointcloud input.\n");
            return;
        }

        mtx.lock();
        path->poses.clear();
        path->poses.insert(path->poses.end(), msg->poses.begin(), msg->poses.end());

        if (!read_map_from_file)
        {
            bb_filter.setInputCloud(cloud); // set input cloud to be filtered
        }

        // drop some pose so that each pose is of distance vehicle_dim_X
        auto it = path->poses.begin();
        auto it2 = path->poses.begin();
        it2++;
        while (it2 != path->poses.end())
        {
            double distance = sqrt(pow(it->pose.position.x - it2->pose.position.x, 2) + pow(it->pose.position.y - it2->pose.position.y, 2));
            if (distance < vehicle_dim_X)
            {
                path->poses.erase(it2);
                continue;
            }
            else
            {
                it++;
                it2++;
            }
        }

        if (path->poses.size() <= 2)
        {
            RCLCPP_WARN(this->get_logger(), "Path is too short.\n");
            mtx.unlock();
            return;
        }

        // ----------------------- Compute the average normal vector of each step -----------------------
        // segment the path into steps, each step is of length vehicle_dim_X
        // reset to zero
        avg_normal_x.clear();
        avg_normal_y.clear();
        avg_normal_z.clear();
        avg_height.clear();
        // is_stair.clear();

        pcl::PointCloud<pcl::Normal>::Ptr all_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZ> path_cloud;
        // std::vector<double> cos_theta_list;

        it = path->poses.begin();
        it2 = path->poses.begin();
        it2++;
        while (it2 != path->poses.end())
        {
            double cos_theta = (it->pose.position.x - it2->pose.position.x) / sqrt(pow(it->pose.position.x - it2->pose.position.x, 2) + pow(it->pose.position.y - it2->pose.position.y, 2));
            double sin_theta = (it->pose.position.y - it2->pose.position.y) / sqrt(pow(it->pose.position.x - it2->pose.position.x, 2) + pow(it->pose.position.y - it2->pose.position.y, 2));
            double x1 = it->pose.position.x + vehicle_dim_Y / 2 * sin_theta;
            double y1 = it->pose.position.y - vehicle_dim_Y / 2 * cos_theta;
            double x2 = it->pose.position.x - vehicle_dim_Y / 2 * sin_theta;
            double y2 = it->pose.position.y + vehicle_dim_Y / 2 * cos_theta;
            double x3 = it2->pose.position.x - vehicle_dim_Y / 2 * sin_theta;
            double y3 = it2->pose.position.y + vehicle_dim_Y / 2 * cos_theta;
            double x4 = it2->pose.position.x + vehicle_dim_Y / 2 * sin_theta;
            double y4 = it2->pose.position.y - vehicle_dim_Y / 2 * cos_theta;

            boundingbox_ptr->clear();
            polygons.clear();
            surface_hull->clear();
            objects->clear();

            boundingbox_ptr->push_back(pcl::PointXYZ(x1, y1, 0.2));
            boundingbox_ptr->push_back(pcl::PointXYZ(x2, y2, 0.2));
            boundingbox_ptr->push_back(pcl::PointXYZ(x3, y3, 0.2));
            boundingbox_ptr->push_back(pcl::PointXYZ(x4, y4, 0.2));
            boundingbox_ptr->push_back(pcl::PointXYZ(x1, y2, 0.2));

            hull.setInputCloud(boundingbox_ptr);       // set input
            hull.reconstruct(*surface_hull, polygons); // compute 2D convex hull

            bb_filter.setHullIndices(polygons);   // set hull vertices
            bb_filter.setHullCloud(surface_hull); // set hull cloud
            bb_filter.filter(*objects);           // exec CropHull filter, store the result in objects

            if (remove_ceiling)
            {
                for (unsigned int i = 0; i < objects->points.size(); i++)
                {
                    if (objects->points[i].z > ceiling_height)
                    {
                        objects->points.erase(objects->points.begin() + i);
                        i--;
                    }
                }
            }
            // ----------------------- Radius Outlier Removal -----------------------
            if (use_ror)
            {
                ror.setInputCloud(objects);
                ror.setRadiusSearch(ror_radius); // since the voxel size of terrain_analysis is 0.1
                ror.setMinNeighborsInRadius(5);  // assume that the pointcloud is of 1 layer, so the number of points in half of the circle is 5
                // ror.setNegative(true);	        // set to false to remove the points outside the circle, otherwise remove the points inside the circle
                ror.filter(*objects); // apply filter
            }
            path_cloud.points.insert(path_cloud.points.end(), objects->points.begin(), objects->points.end());

            // ----------------------- Compute the normals of the segment-out points -----------------------
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            normalEstimation.setInputCloud(objects);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
            normalEstimation.setSearchMethod(kdtree);
            normalEstimation.compute(*normals);

            all_normals->insert(all_normals->end(), normals->begin(), normals->end());
            // cos_theta_list.push_back(cos_theta);

            //  ----------------------- Compute the average normal vector of this segment -----------------------
            double step_avg_normal_x = 0.0;
            double step_avg_normal_y = 0.0;
            double step_avg_normal_z = 0.0;
            double step_avg_height = 0.0;
            for (long unsigned int i = 0; i < normals->points.size(); i++)
            {
                step_avg_normal_x += normals->points[i].normal_x;
                step_avg_normal_y += normals->points[i].normal_y;
                step_avg_normal_z += normals->points[i].normal_z;
                step_avg_height += objects->points[i].z;
            }
            if (normals->points.size() == 0)
            {
                RCLCPP_WARN(this->get_logger(), "No points in this step.\n");
                step_avg_normal_x = 0.0;
                step_avg_normal_y = 0.0;
                step_avg_normal_z = 1.0;
                step_avg_height = 0.0;
            }
            else
            {
                step_avg_normal_x /= normals->points.size();
                step_avg_normal_y /= normals->points.size();
                step_avg_normal_z /= normals->points.size();
                step_avg_height /= normals->points.size();
            }

            // remove nan points
            if (isnan(step_avg_normal_x) || isnan(step_avg_normal_y) || isnan(step_avg_normal_z))
            {
                step_avg_normal_x = 0.0;
                step_avg_normal_y = 0.0;
                step_avg_normal_z = 1.0;
            }

            avg_normal_x.push_back(step_avg_normal_x);
            avg_normal_y.push_back(step_avg_normal_y);
            avg_normal_z.push_back(step_avg_normal_z);
            avg_height.push_back(step_avg_height);

            // bool is_stair_step = stair_detection(objects, step_avg_normal_x, step_avg_normal_y, step_avg_normal_z);
            // is_stair.push_back(is_stair_step);

            it++;
            it2++;
        }

        // Publish the pointcloud of the path
        sensor_msgs::msg::PointCloud2 pc_msg;
        pcl::toROSMsg(path_cloud, pc_msg);
        pc_msg.header.frame_id = global_frame;
        pc_publisher_->publish(pc_msg);

        // Publish the normal vector of each point
        geometry_msgs::msg::PoseArray all_normal_msg;
        all_normal_msg.header.frame_id = global_frame;
        for (long unsigned int i = 0; i < all_normals->points.size(); i++)
        {
            geometry_msgs::msg::Pose normal;
            normal.position.x = path_cloud.points[i].x;
            normal.position.y = path_cloud.points[i].y;
            normal.position.z = path_cloud.points[i].z;

            double roll = atan2(all_normals->points[i].normal_y, all_normals->points[i].normal_z);
            double pitch = atan2(all_normals->points[i].normal_x, all_normals->points[i].normal_z);
            double yaw = atan2(all_normals->points[i].normal_y, all_normals->points[i].normal_x); // - cos_theta_list[i] * PI / 2;
            tf2::Quaternion q;
            q.setRPY(pitch, roll, yaw);
            normal.orientation.w = q.getW();
            normal.orientation.x = q.getX();
            normal.orientation.y = q.getY();
            normal.orientation.z = q.getZ();
            all_normal_msg.poses.push_back(normal);
        }

        all_normal_publisher_->publish(all_normal_msg);

        // Publish the average normal vector of each step
        geometry_msgs::msg::PoseArray avg_normal_msg;
        avg_normal_msg.header.frame_id = global_frame;
        for (unsigned long int i = 0; i < avg_normal_x.size() - 2; i++)
        {
            geometry_msgs::msg::Pose normal;
            normal.position.x = (path->poses[i].pose.position.x + path->poses[i + 1].pose.position.x) / 2; // place the normal vector at the center of the step
            normal.position.y = (path->poses[i].pose.position.y + path->poses[i + 1].pose.position.y) / 2;
            normal.position.z = avg_height[i];

            double roll = atan2(avg_normal_y[i], avg_normal_z[i]);
            double pitch = atan2(avg_normal_x[i], avg_normal_z[i]);
            double yaw = atan2(avg_normal_y[i], avg_normal_x[i]); // - cos_theta_list[i] * PI / 2;
            tf2::Quaternion q;
            q.setRPY(pitch, roll, yaw);
            normal.orientation.w = q.getW();
            normal.orientation.x = q.getX();
            normal.orientation.y = q.getY();
            normal.orientation.z = q.getZ();
            avg_normal_msg.poses.push_back(normal);

            // if (is_stair[i])
            // {
            //     geometry_msgs::msg::PoseStamped normal_stamped;
            //     normal_stamped.header.frame_id = global_frame;
            //     normal_stamped.pose = normal;
            //     Stair_normal_publisher_->publish(normal_stamped);
            // }
        }
        avg_normal_publisher_->publish(avg_normal_msg);

        normal_analysis_done = true;

        mtx.unlock();
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::fromROSMsg(*msg, *cloud);
        pc_init = true;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_plan_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pointcloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_stateestimation_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr all_normal_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr avg_normal_publisher_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Stair_normal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angDiff_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_up_rate_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slope_degree_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double vehicle_dim_X, vehicle_dim_Y, Z_offset, ground_height;
    // double adjustThre;
    double goalX, goalY;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr;
    pcl::ConvexHull<pcl::PointXYZ> hull; // create a ConvexHull object
    std::vector<pcl::Vertices> polygons; // create a vector of pcl::Vertices, used to store the polygonal description of the convex hull
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> surface_hull;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> objects;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    pcl::CropHull<pcl::PointXYZ> bb_filter;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    bool stateestimation_received;
    bool normal_analysis_done;
    bool pc_init;
    bool read_map_from_file;
    int step_num;
    std::string map_file_path;
    std::vector<double> avg_normal_x;
    std::vector<double> avg_normal_y;
    std::vector<double> avg_normal_z;
    std::vector<double> avg_height;
    // std::vector<bool> is_stair;
    std::mutex mtx;
    nav_msgs::msg::Path::SharedPtr path;
    double remove_thre;
    // double min_stair_height;
    bool use_ror, remove_ceiling;
    double ror_radius, ceiling_height;
    // double min_stair_degree;
    double speed_up_slope_degree_thre;
    double speed_up_max_rate;
    double max_speed_up_slope_degree;
    double speedup_duration;
    std::string base_frame, global_frame;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessingNode>());
    rclcpp::shutdown();
    return 0;
}