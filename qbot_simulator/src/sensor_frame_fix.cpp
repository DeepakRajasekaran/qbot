#include "qbot_simulator/sensor_frame_fix.hpp"

namespace qbot_simulator
{

SensorFrameRepublisher::SensorFrameRepublisher()
: Node("sensor_frame_republisher")
{
  this->declare_parameter<std::string>("scan_frame_id", "lidar");
  m_scanFrameId = this->get_parameter("scan_frame_id").as_string();

  this->declare_parameter<std::string>("imu_frame_id", "imu");
  m_imuFrameId = this->get_parameter("imu_frame_id").as_string();

  m_scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan_gz",
    rclcpp::SensorDataQoS(),
    std::bind(&SensorFrameRepublisher::scanCallback, this, std::placeholders::_1)
  );

  m_scanPub = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan",
    rclcpp::SensorDataQoS()
  );

  m_imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data_gz",
    rclcpp::SensorDataQoS(),
    std::bind(&SensorFrameRepublisher::imuCallback, this, std::placeholders::_1)
  );

  m_imuPub = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data",
    rclcpp::SensorDataQoS()
  );


  RCLCPP_INFO(
    this->get_logger(),
    "Republishing /scan_gz -> /scan with frame_id='%s'",
    m_scanFrameId.c_str()
  );

  RCLCPP_INFO(
    this->get_logger(),
    "Republishing /imu/data_gz -> /imu/data with frame_id='%s'",
    m_imuFrameId.c_str()
  );
}

void SensorFrameRepublisher::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg
)
{
  auto out = *msg;
  out.header.frame_id = m_scanFrameId;
  m_scanPub->publish(out);
}

void SensorFrameRepublisher::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg
)
{
  auto out = *msg;
  out.header.frame_id = m_imuFrameId;
  m_imuPub->publish(out);
}

}  // namespace qbot_simulator


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qbot_simulator::SensorFrameRepublisher>());
  rclcpp::shutdown();
  return 0;
}
