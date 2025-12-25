#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace qbot_simulator
{

class SensorFrameRepublisher : public rclcpp::Node
{
public:
  explicit SensorFrameRepublisher();

private:
  void scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg
  );

  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg
  );

  std::string m_scanFrameId;
  std::string m_imuFrameId;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scanSub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_scanPub;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imuSub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imuPub;
};

}  // namespace qbot_simulator
