#ifndef QBOT_CONTROLLER__SERIAL_BRIDGE_NODE_HPP_
#define QBOT_CONTROLLER__SERIAL_BRIDGE_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <thread>

// Serial includes
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace qbot_controller
{

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode();
  virtual ~SerialBridgeNode();

private:
  /**
   * @brief Callback for command velocity messages.
   * @param msg Shared pointer to the Twist message.
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Sends a raw string command to the serial port.
   * @param cmd The command string.
   */
  void sendCommand(const std::string & cmd);

  /**
   * @brief Opens and configures the serial port.
   * @return true if successful, false otherwise.
   */
  bool openSerialPort();

  /**
   * @brief Reads available data from the serial port into the buffer.
   */
  void readSerialData();

  /**
   * @brief Processes the serial buffer to extract lines.
   */
  void processBuffer();

  /**
   * @brief Parses a single line of data from the microcontroller.
   * @param line The line to parse.
   */
  void parseLine(const std::string & line);

  /**
   * @brief Publishes odometry data.
   * @param x X position.
   * @param y Y position.
   * @param theta Yaw angle.
   */
  void publishOdom(float x, float y, float theta);

  /**
   * @brief Publishes IMU data.
   * @param ax Accelerometer X.
   * @param ay Accelerometer Y.
   * @param az Accelerometer Z.
   * @param gx Gyroscope X.
   * @param gy Gyroscope Y.
   * @param gz Gyroscope Z.
   */
  void publishImu(int ax, int ay, int az, int gx, int gy, int gz);

  std::string m_serialPort;
  int m_baudRate;
  std::string m_frameId;
  std::string m_childFrameId;
  std::string m_imuFrameId;

  int m_serialFd = -1;
  std::string m_serialBuffer;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_p_odomPub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_p_imuPub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_p_cmdVelSub;
  rclcpp::TimerBase::SharedPtr m_p_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;

  // Odometry calculation state
  rclcpp::Time m_lastTime;
  float m_lastX = 0.0f;
  float m_lastY = 0.0f;
  float m_lastTheta = 0.0f;
  double m_lastVx = 0.0;
  double m_lastVy = 0.0;
  double m_lastVth = 0.0;

  // IMU Filter state
  rclcpp::Time m_lastImuTime;
  double m_imuRoll = 0.0;
  double m_imuPitch = 0.0;
  double m_imuYaw = 0.0;
};

}  // namespace qbot_controller

#endif  // QBOT_CONTROLLER__SERIAL_BRIDGE_NODE_HPP_