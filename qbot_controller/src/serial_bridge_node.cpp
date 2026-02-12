#include "qbot_controller/serial_bridge_node.hpp"

using namespace std::chrono_literals;

namespace qbot_controller
{
SerialBridgeNode::SerialBridgeNode()
: Node("serial_bridge_node")
{
  // --- Parameters ---
  this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<std::string>("frame_id", "odom");
  this->declare_parameter<std::string>("child_frame_id", "base_link");
  this->declare_parameter<std::string>("imu_frame_id", "imu_link");

  m_serialPort = this->get_parameter("serial_port").as_string();
  m_baudRate = this->get_parameter("baud_rate").as_int();
  m_frameId = this->get_parameter("frame_id").as_string();
  m_childFrameId = this->get_parameter("child_frame_id").as_string();
  m_imuFrameId = this->get_parameter("imu_frame_id").as_string();

  // Initialize time for odometry calculation
  m_lastTime = this->now();
  m_lastImuTime = this->now();

  // --- Publishers ---
  m_p_odomPub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  m_p_imuPub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

  // --- TF Broadcaster ---
  m_tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // --- Subscribers ---
  m_p_cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&SerialBridgeNode::cmdVelCallback, this, std::placeholders::_1));

  // --- Serial Connection ---
  if (openSerialPort()) {
    RCLCPP_INFO(this->get_logger(), "Connected to %s at %d baud", m_serialPort.c_str(), m_baudRate);

    // Wait for Arduino to reset after connection (similar to python script)
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Initialize Firmware State
    sendCommand("t2");  // Enable CSV Telemetry (COM mode)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sendCommand("m1");  // Set Mode: VELOCITY

    // Timer for reading serial data
    m_p_timer = this->create_wall_timer(
      10ms, std::bind(&SerialBridgeNode::readSerialData, this));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", m_serialPort.c_str());
  }
}

SerialBridgeNode::~SerialBridgeNode()
{
  if (m_serialFd != -1) {
    sendCommand("s");  // Stop motors on exit
    close(m_serialFd);
  }
}

void SerialBridgeNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Protocol: v<linear> <angular> (e.g., v0.1 0.0)
  std::stringstream ss;
  ss << "v" << msg->linear.x << " " << msg->angular.z << "\n";
  sendCommand(ss.str());
}

void SerialBridgeNode::sendCommand(const std::string & cmd)
{
  if (m_serialFd != -1) {
    std::string full_cmd = cmd;
    if (full_cmd.back() != '\n') full_cmd += "\n";
    write(m_serialFd, full_cmd.c_str(), full_cmd.length());
  }
}

bool SerialBridgeNode::openSerialPort()
{
  m_serialFd = open(m_serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (m_serialFd == -1) {
    return false;
  }

  struct termios options;
  tcgetattr(m_serialFd, &options);

  // Set baud rate
  speed_t baud;
  switch (m_baudRate) {
    case 9600:
      baud = B9600;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unsupported baudrate %d, defaulting to 115200", m_baudRate);
      baud = B115200;
      break;
  }
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  // Configure 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~CRTSCTS;          // No flow control
  options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver

  // Raw input/output
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  tcsetattr(m_serialFd, TCSANOW, &options);
  fcntl(m_serialFd, F_SETFL, FNDELAY);  // Non-blocking read

  return true;
}

void SerialBridgeNode::readSerialData()
{
  char buffer[1024];
  int n = read(m_serialFd, buffer, sizeof(buffer) - 1);
  if (n > 0) {
    buffer[n] = '\0';
    m_serialBuffer += buffer;
    processBuffer();
  }
}

void SerialBridgeNode::processBuffer()
{
  size_t pos;
  while ((pos = m_serialBuffer.find('\n')) != std::string::npos) {
    std::string line = m_serialBuffer.substr(0, pos);
    m_serialBuffer.erase(0, pos + 1);

    // Remove carriage return if present
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    parseLine(line);
  }
}

void SerialBridgeNode::parseLine(const std::string & line)
{
  // Protocol: f;poseX;poseY;poseTheta;ax;ay;az;gx;gy;gz;inputL;inputR
  if (line.rfind("f;", 0) == 0) {
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ';')) {
      parts.push_back(item);
    }

    if (parts.size() >= 12) {
      try {
        float px = std::stof(parts[1]);
        float py = std::stof(parts[2]);
        float pth = std::stof(parts[3]);

        int ax = std::stoi(parts[4]);
        int ay = std::stoi(parts[5]);
        int az = std::stoi(parts[6]);

        int gx = std::stoi(parts[7]);
        int gy = std::stoi(parts[8]);
        int gz = std::stoi(parts[9]);

        publishOdom(px, py, pth);
        publishImu(ax, ay, az, gx, gy, gz);

      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Parse error: %s", e.what());
      }
    }
  }
}

void SerialBridgeNode::publishOdom(float x, float y, float theta)
{
  auto msg = nav_msgs::msg::Odometry();
  rclcpp::Time now = this->now();
  msg.header.stamp = now;
  msg.header.frame_id = m_frameId;
  msg.child_frame_id = m_childFrameId;

  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.position.z = 0.0;

  // Calculate orientation from Theta (Yaw) only - Planar motion
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = theta;

  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
  msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

  // --- Calculate Twist (Velocities) ---
  double dt = (now - m_lastTime).seconds();
  double vx = m_lastVx;
  double vy = m_lastVy;
  double vth = m_lastVth;

  // Only update velocities if enough time has passed (avoids spikes in data bursts)
  if (dt > 0.001) {
    double dx = x - m_lastX;
    double dy = y - m_lastY;
    double dth = theta - m_lastTheta;

    // Handle angle wrapping for theta derivative
    if (dth > M_PI) dth -= 2.0 * M_PI;
    else if (dth < -M_PI) dth += 2.0 * M_PI;

    double vx_world = dx / dt;
    double vy_world = dy / dt;
    vth = dth / dt;

    // Rotate linear velocity from World frame to Body frame (base_link)
    double c = std::cos(theta);
    double s = std::sin(theta);
    vx = c * vx_world + s * vy_world;
    vy = -s * vx_world + c * vy_world;

    m_lastVx = vx;
    m_lastVy = vy;
    m_lastVth = vth;
    m_lastX = x;
    m_lastY = y;
    m_lastTheta = theta;
    m_lastTime = now;
  }

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.angular.z = vth;

  m_p_odomPub->publish(msg);

  // Publish TF (odom -> base_link)
  geometry_msgs::msg::TransformStamped t;
  t.header = msg.header;
  t.child_frame_id = m_childFrameId;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = msg.pose.pose.orientation;
  m_tfBroadcaster->sendTransform(t);
}

void SerialBridgeNode::publishImu(int ax, int ay, int az, int gx, int gy, int gz)
{
  auto msg = sensor_msgs::msg::Imu();
  rclcpp::Time now = this->now();
  msg.header.stamp = now;
  msg.header.frame_id = m_imuFrameId;

  double dt = (now - m_lastImuTime).seconds();
  m_lastImuTime = now;
  if (dt <= 0) dt = 0.01; // Prevent division by zero or negative time on first run

  // Convert units:
  // Accel: +/- 2g range -> 16384 LSB/g. 1g = 9.80665 m/s^2
  // Gyro: +/- 250 dps -> 131 LSB/dps. 1 dps = 0.0174533 rad/s
  const float ACCEL_SCALE = (1.0 / 16384.0) * 9.80665;
  const float GYRO_SCALE = (1.0 / 131.0) * 0.0174533;

  msg.linear_acceleration.x = ax * ACCEL_SCALE;
  msg.linear_acceleration.y = ay * ACCEL_SCALE;
  msg.linear_acceleration.z = az * ACCEL_SCALE;
  
  // Negate gravity from Z-axis (remove 1g offset)
  msg.linear_acceleration.z -= 9.80665;

  msg.angular_velocity.x = gx * GYRO_SCALE;
  msg.angular_velocity.y = gy * GYRO_SCALE;
  msg.angular_velocity.z = gz * GYRO_SCALE;

  // --- Complementary Filter ---
  // 1. Calculate Roll/Pitch from Accelerometer (using raw values including gravity)
  // Note: We use the raw scaled values (before gravity subtraction) for angle calculation
  float ax_m = ax * ACCEL_SCALE;
  float ay_m = ay * ACCEL_SCALE;
  float az_m = az * ACCEL_SCALE;

  float accel_roll = std::atan2(ay_m, az_m);
  float accel_pitch = std::atan2(-ax_m, std::sqrt(ay_m * ay_m + az_m * az_m));

  // 2. Fuse with Gyroscope Integration
  const float alpha = 0.98;
  m_imuRoll = alpha * (m_imuRoll + msg.angular_velocity.x * dt) + (1.0 - alpha) * accel_roll;
  m_imuPitch = alpha * (m_imuPitch + msg.angular_velocity.y * dt) + (1.0 - alpha) * accel_pitch;
  m_imuYaw += msg.angular_velocity.z * dt;

  double cy = std::cos(m_imuYaw * 0.5);
  double sy = std::sin(m_imuYaw * 0.5);
  double cp = std::cos(m_imuPitch * 0.5);
  double sp = std::sin(m_imuPitch * 0.5);
  double cr = std::cos(m_imuRoll * 0.5);
  double sr = std::sin(m_imuRoll * 0.5);

  msg.orientation.w = cr * cp * cy + sr * sp * sy;
  msg.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.orientation.z = cr * cp * sy - sr * sp * cy;

  msg.orientation_covariance[0] = 0.01;

  m_p_imuPub->publish(msg);

  // Publish dynamic TF for IMU (base_link -> imu_link)
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg.header.stamp;
  t.header.frame_id = m_childFrameId;
  t.child_frame_id = m_imuFrameId;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation = msg.orientation;
  m_tfBroadcaster->sendTransform(t);
}

}  // namespace qbot_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qbot_controller::SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
