#include "qbot_controller/mpu6050_node.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

namespace qbot_controller
{

MPU6050Node::MPU6050Node()
: Node("mpu6050_node")
{
  // Parameters
  this->declare_parameter("i2c_bus", "/dev/i2c-1"); // Default for Pi 3/4/5
  this->declare_parameter("i2c_address", 0x68);     // Default MPU6050 address
  this->declare_parameter("frame_id", "imu_link");

  m_i2cBus = this->get_parameter("i2c_bus").as_string();
  m_deviceAddr = this->get_parameter("i2c_address").as_int();
  m_frameId = this->get_parameter("frame_id").as_string();

  // Publisher
  m_pubImu = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  // Initialize Hardware
  initI2C();

  // Timer (100 Hz)
  m_timer = this->create_wall_timer(
    10ms, std::bind(&MPU6050Node::readSensor, this));
  
  RCLCPP_INFO(this->get_logger(), "MPU6050 Node started on bus %s at 0x%02X", m_i2cBus.c_str(), m_deviceAddr);
}

MPU6050Node::~MPU6050Node()
{
  if (m_file >= 0) {
    close(m_file);
  }
}

void MPU6050Node::initI2C()
{
  m_file = open(m_i2cBus.c_str(), O_RDWR);
  if (m_file < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open I2C bus %s. Check permissions.", m_i2cBus.c_str());
    return;
  }

  if (ioctl(m_file, I2C_SLAVE, m_deviceAddr) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave 0x%02X", m_deviceAddr);
    return;
  }

  // Wake up MPU6050 (Write 0 to PWR_MGMT_1 register 0x6B)
  uint8_t buf[2];
  buf[0] = 0x6B; // Register
  buf[1] = 0x00; // Value
  if (write(m_file, buf, 2) != 2) {
    RCLCPP_ERROR(this->get_logger(), "Failed to wake up MPU6050");
  }
}

void MPU6050Node::readSensor()
{
  if (m_file < 0) return;

  // Start reading from ACCEL_XOUT_H (0x3B)
  // We need 14 bytes: Accel(6) + Temp(2) + Gyro(6)
  uint8_t reg = 0x3B;
  if (write(m_file, &reg, 1) != 1) {
    // This might fail occasionally if bus is busy
    return;
  }

  uint8_t data[14];
  if (read(m_file, data, 14) != 14) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to read sensor data");
    return;
  }

  // Combine high and low bytes
  int16_t ax_raw = (data[0] << 8) | data[1];
  int16_t ay_raw = (data[2] << 8) | data[3];
  int16_t az_raw = (data[4] << 8) | data[5];
  // int16_t temp_raw = (data[6] << 8) | data[7]; // Temperature ignored
  int16_t gx_raw = (data[8] << 8) | data[9];
  int16_t gy_raw = (data[10] << 8) | data[11];
  int16_t gz_raw = (data[12] << 8) | data[13];

  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = this->now();
  msg.header.frame_id = m_frameId;

  // Default Sensitivity settings (AFS_SEL=0, FS_SEL=0)
  // Accel: +/- 2g -> 16384 LSB/g
  // Gyro: +/- 250 dps -> 131 LSB/dps
  const double G_TO_MS2 = 9.80665;
  const double DEG_TO_RAD = M_PI / 180.0;

  msg.linear_acceleration.x = (ax_raw / 16384.0) * G_TO_MS2;
  msg.linear_acceleration.y = (ay_raw / 16384.0) * G_TO_MS2;
  msg.linear_acceleration.z = (az_raw / 16384.0) * G_TO_MS2;

  msg.angular_velocity.x = (gx_raw / 131.0) * DEG_TO_RAD;
  msg.angular_velocity.y = (gy_raw / 131.0) * DEG_TO_RAD;
  msg.angular_velocity.z = (gz_raw / 131.0) * DEG_TO_RAD;

  // Orientation is not calculated here (raw data only)
  msg.orientation_covariance[0] = -1;

  m_pubImu->publish(msg);
}

}  // namespace qbot_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qbot_controller::MPU6050Node>());
  rclcpp::shutdown();
  return 0;
}
