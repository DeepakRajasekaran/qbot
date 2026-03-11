#include "qbot_simulator/camera_patcher_node.hpp"
#include <memory>

namespace qbot_simulator
{

CameraFramePatcher::CameraFramePatcher()
: Node("camera_frame_patcher"), optical_frame_id_("camera_optical_link")
{
  // Publishers for RTAB-Map
  rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rtabmap/rgb/image", 10);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/rtabmap/depth/image", 10);
  info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/rtabmap/rgb/camera_info", 10);

  // Subscribers to raw camera topics from Gazebo bridge
  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10, std::bind(&CameraFramePatcher::rgbCallback, this, std::placeholders::_1));
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/depth_image", 10, std::bind(&CameraFramePatcher::depthCallback, this, std::placeholders::_1));
  info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", 10, std::bind(&CameraFramePatcher::infoCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "C++ Patcher started. Republishing images to /rtabmap/* with frame_id '%s'.", optical_frame_id_.c_str());
}

void CameraFramePatcher::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto patched_msg = std::make_unique<sensor_msgs::msg::Image>(*msg);
  patched_msg->header.frame_id = optical_frame_id_;
  rgb_pub_->publish(std::move(patched_msg));
}

void CameraFramePatcher::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  auto patched_msg = std::make_unique<sensor_msgs::msg::Image>(*msg);
  patched_msg->header.frame_id = optical_frame_id_;
  depth_pub_->publish(std::move(patched_msg));
}

void CameraFramePatcher::infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  auto patched_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(*msg);
  patched_msg->header.frame_id = optical_frame_id_;
  info_pub_->publish(std::move(patched_msg));
}

} // namespace qbot_simulator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qbot_simulator::CameraFramePatcher>());
  rclcpp::shutdown();
  return 0;
}