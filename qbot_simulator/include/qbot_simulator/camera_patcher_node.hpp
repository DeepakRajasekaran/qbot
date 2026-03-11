#ifndef QBOT_SIMULATOR__CAMERA_PATCHER_NODE_HPP_
#define QBOT_SIMULATOR__CAMERA_PATCHER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace qbot_simulator
{

class CameraFramePatcher : public rclcpp::Node
{
public:
  CameraFramePatcher();

private:
  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  std::string optical_frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

}  // namespace qbot_simulator

#endif  // QBOT_SIMULATOR__CAMERA_PATCHER_NODE_HPP_