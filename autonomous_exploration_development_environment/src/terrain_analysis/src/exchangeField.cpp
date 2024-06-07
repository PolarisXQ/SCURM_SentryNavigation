#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class PointCloudProcessor : public rclcpp::Node
{
public:
  PointCloudProcessor()
  : Node("point_cloud_field_exchange")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output_topic", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_topic", 10, std::bind(&PointCloudProcessor::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::PointCloud2Modifier pcd_modifier(*msg);
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*msg, "intensity");

    for (; iter_z != iter_z.end(); ++iter_z, ++iter_intensity) {
      float temp = *iter_z;
      *iter_z = *iter_intensity;
      *iter_intensity = temp;
    }

    publisher_->publish(*msg);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}