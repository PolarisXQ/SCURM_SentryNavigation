#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PCDListenerNode : public rclcpp::Node
{
public:
    PCDListenerNode()
        : Node("pcd_saver_node")
    {
        this->declare_parameter<std::string>("save_path", "./analysis_result.pcd");
        this->declare_parameter<std::string>("topic_name","/terrain_map_ext");

        this->get_parameter_or<std::string>("save_path", save_path, "./analysis_result.pcd");
        this->get_parameter_or<std::string>("topic_name", topic_name,"/terrain_map_ext");

        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name, 10, std::bind(&PCDListenerNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);
        pcl::io::savePCDFileASCII(save_path, cloud); // Replace with your file name
        RCLCPP_INFO(this->get_logger(), "Saved point cloud");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    std::string save_path;
    std::string topic_name;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDListenerNode>());
    rclcpp::shutdown();
    return 0;
}