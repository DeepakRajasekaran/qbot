#ifndef QBOT_CONTROLLER__MPU6050_NODE_HPP_
#define QBOT_CONTROLLER__MPU6050_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

namespace qbot_controller
{

class MPU6050Node : public rclcpp::Node
{
public:
  MPU6050Node();
  virtual ~MPU6050Node();

private:
  /**
   * @brief Initializes the I2C connection and configures the MPU6050.
   */
  void initI2C();

  /**
   * @brief Reads raw data from the sensor and publishes an Imu message.
   */
  void readSensor();

  int m_file = -1;
  std::string m_i2cBus;
  int m_deviceAddr;
  std::string m_frameId;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_pubImu;
  rclcpp::TimerBase::SharedPtr m_timer;
};

}  // namespace qbot_controller

#endif  // QBOT_CONTROLLER__MPU6050_NODE_HPP_