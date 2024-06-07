#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Geometry>
#include <pcl-1.12/pcl/common/transforms.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <mutex>
class PCDPublisherNode : public rclcpp::Node
{
public:
    PCDPublisherNode()
        : Node("pcd_publisher_node"), timer_(nullptr)
    {
        // Get the parameter value
        std::string pub_full_map_topic;
        std::string map_file_path;
        int64_t period_ms;
        bool rotate_map;

        this->declare_parameter<std::string>("pub_full_map_topic", "full_map");
        this->declare_parameter<std::string>("map_file_path", "");
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<int64_t>("period_ms", 1000);
        this->declare_parameter<bool>("analysis_mode", false);
        this->declare_parameter<bool>("rotate_map", false);
        this->declare_parameter<bool>("merge_terrainCloudLocal", false);
        this->declare_parameter<double>("localTerrainMapRadius", localTerrainMapRadius);

        this->get_parameter_or<std::string>("pub_full_map_topic", pub_full_map_topic, "full_map");
        this->get_parameter_or<std::string>("map_file_path", map_file_path, "");
        this->get_parameter_or<std::string>("frame_id", frame_id, "map");
        this->get_parameter_or<int64_t>("period_ms", period_ms, 1000);
        this->get_parameter_or<bool>("analysis_mode", analysis_mode, false);
        this->get_parameter_or<bool>("rotate_map", rotate_map, false);
        this->get_parameter_or<bool>("merge_terrainCloudLocal", merge_terrainCloudLocal, false);
        this->get_parameter_or<double>("localTerrainMapRadius", localTerrainMapRadius, 10.0);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_full_map_topic, 10);

        if (analysis_mode && merge_terrainCloudLocal)
        {
            RCLCPP_ERROR(this->get_logger(), "analysis_mode & merge_terrainCloudLocal is not allowed to turn on at the same time!");
            return;
        }

        if (analysis_mode)
        {
            fake_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);
        }
        if (merge_terrainCloudLocal)
        {
            subTerrainCloudLocal = this->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map", 2, std::bind(&PCDPublisherNode::terrainCloudLocalHandler, this, std::placeholders::_1));
            subOdometry = this->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 2, std::bind(&PCDPublisherNode::odometryHandler, this, std::placeholders::_1));
            terrainCloudLocal = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        }

        cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file_path, *cloud) == -1) // Replace with your file name
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file\n");
            return;
        }
        map_file_size = cloud->points.size();
        RCLCPP_INFO(this->get_logger(), "Loaded %ld data points from PCD file", map_file_size);
        if (rotate_map)
        {
            // rotate map along x axis
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
            pcl::transformPointCloud(*cloud, *cloud, transform);
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&PCDPublisherNode::publish_cloud, this));
    }

private:
    void publish_cloud()
    {
        if (analysis_mode)
        {
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.frame_id = "map";
            odom_msg.child_frame_id = "sensor";
            odom_msg.header.stamp = this->now();
            odom_msg.pose.pose.position.x = 0;
            odom_msg.pose.pose.position.y = 0;
            odom_msg.pose.pose.position.z = 0;
            // TODO: terrain analysis do not consider orientation
            odom_msg.pose.pose.orientation.x = 0;
            odom_msg.pose.pose.orientation.y = 0;
            odom_msg.pose.pose.orientation.z = 0;
            odom_msg.pose.pose.orientation.w = 1;
            fake_odom_publisher_->publish(odom_msg);
        }

        mtx.lock();
        if (merge_terrainCloudLocal)
        {
            // clean point previously added by terrainCloudLocal
            cloud->points.resize(map_file_size);
            pcl::PointXYZI point;
            int terrainCloudLocalSize = terrainCloudLocal->points.size();
            for (int i = 0; i < terrainCloudLocalSize; i++)
            {
                point = terrainCloudLocal->points[i];
                float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
                if (dis <= localTerrainMapRadius)
                {
                    cloud->points.push_back(point);
                }
            }
            cloud->width = cloud->points.size(); // !IMPORTANT: You must set width to change the size of point cloud, otherwise it will not work in rviz
            cloud->height = 1;
        }


        sensor_msgs::msg::PointCloud2 cloud_msg_;
        pcl::toROSMsg(*cloud, cloud_msg_);
        cloud_msg_.header.frame_id = frame_id; // Replace with your frame id
        cloud_msg_.header.stamp = this->now();
        publisher_->publish(cloud_msg_);
        RCLCPP_DEBUG(this->get_logger(), "Publishing point cloud");
        mtx.unlock();
    }

    void terrainCloudLocalHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloudLocal2)
    {
        terrainCloudLocal->clear();
        pcl::fromROSMsg(*terrainCloudLocal2, *terrainCloudLocal);
    }

    void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
    {
        double roll, pitch, yaw;
        geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
        tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

        vehicleRoll = roll;
        vehiclePitch = pitch;
        vehicleYaw = yaw;
        vehicleX = odom->pose.pose.position.x;
        vehicleY = odom->pose.pose.position.y;
        vehicleZ = odom->pose.pose.position.z;

        sinVehicleYaw = sin(vehicleYaw);
        cosVehicleYaw = cos(vehicleYaw);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fake_odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subTerrainCloudLocal;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::TimerBase::SharedPtr timer_;
    bool analysis_mode, merge_terrainCloudLocal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudLocal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double localTerrainMapRadius;
    std::string frame_id;
    u_int64_t map_file_size;
    std::mutex mtx;

    float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
    float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
    float sinVehicleYaw = 0, cosVehicleYaw = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDPublisherNode>());
    rclcpp::shutdown();
    return 0;
}