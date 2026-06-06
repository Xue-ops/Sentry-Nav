#include <cerrno>
#include <cstdint>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include <termios.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr uint16_t CRC16_INIT = 0xffff;
const uint16_t CRC16_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3,
  0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 0x2102, 0x308b, 0x0210, 0x1399,
  0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50,
  0xfbef, 0xea66, 0xd8fd, 0xc974, 0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 0x5285, 0x430c, 0x7197, 0x601e,
  0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5,
  0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 0x8408, 0x9581, 0xa71a, 0xb693,
  0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a,
  0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 0xb58b, 0xa402, 0x9699, 0x8710,
  0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df,
  0x0c60, 0x1de9, 0x2f72, 0x3efb, 0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 0xe70e, 0xf687, 0xc41c, 0xd595,
  0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c,
  0x3de3, 0x2c6a, 0x1ef1, 0x0f78};


struct __attribute__((packed)) CMDPacket
{
  uint8_t head[2] = {'S', 'P'}; // Must be {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  uint8_t is_self_color_red;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;

  float x_vel;
  float y_vel;
  uint8_t spintop_level;
  
  uint16_t crc16;

};

uint16_t get_crc16(const uint8_t * data, uint32_t len)
{
  uint16_t crc16 = CRC16_INIT;
  uint8_t byte;
  uint8_t i;

  while (len--) {
    byte = *data++;
    i = (crc16 ^ byte) & 0x00ff;
    crc16 = (crc16 >> 8) ^ CRC16_TABLE[i];
  }

  return crc16;
}

class SimpleCmdPublish : public rclcpp::Node
{
public:
  SimpleCmdPublish()
  : Node("simple_cmd_publish")
  {
    port_name_ = this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 921600);
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    openSerial();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10),
      std::bind(&SimpleCmdPublish::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Listening to %s, publishing x/y velocity packets to %s at %d baud",
      cmd_vel_topic_.c_str(),
      port_name_.c_str(),
      baud_rate_);
  }

  ~SimpleCmdPublish() override
  {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

private:
  static constexpr uint8_t kFrameHead = 0x5a;

  std::string port_name_;
  std::string cmd_vel_topic_;
  int baud_rate_;
  int serial_fd_{-1};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  static speed_t baudRateToFlag(const int baud_rate)
  {
    switch (baud_rate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 921600:
        return B921600;
      default:
        return B115200;
    }
  }

  void openSerial()
  {
    serial_fd_ = open(port_name_.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to open serial port %s: %s",
        port_name_.c_str(),
        std::strerror(errno));
      return;
    }

    termios options{};
    if (tcgetattr(serial_fd_, &options) != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to read serial options for %s: %s",
        port_name_.c_str(),
        std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }

    cfmakeraw(&options);
    const speed_t speed = baudRateToFlag(baud_rate_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to configure serial port %s: %s",
        port_name_.c_str(),
        std::strerror(errno));
      close(serial_fd_);
      serial_fd_ = -1;
      return;
    }
  }

  static CMDPacket makePacket(const double velocity_x, const double velocity_y)
  {
    CMDPacket packet;
    packet.mode = 0;
    packet.is_self_color_red = 0;
    packet.yaw = 0;
    packet.yaw_vel = 0;
    packet.yaw_acc = 0;
    packet.pitch = 0;
    packet.pitch_vel = 0;
    packet.pitch_acc = 0;
    packet.x_vel = static_cast<float>(velocity_x);
    packet.y_vel = static_cast<float>(velocity_y);
    RCLCPP_INFO(
      rclcpp::get_logger("cmd_packet"),
      "v_x is %f, v_y is %f",
      packet.x_vel,
      packet.y_vel
    );
    packet.spintop_level = 0;
    packet.crc16 = get_crc16(
    reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - sizeof(packet.crc16));
    return packet;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (serial_fd_ < 0) {
      openSerial();
      if (serial_fd_ < 0) {
        return;
      }
    }

    const CMDPacket packet = makePacket(msg->linear.x, msg->linear.y);
    const ssize_t written = write(serial_fd_, &packet, sizeof(packet));
    if (written != static_cast<ssize_t>(sizeof(packet))) {
      RCLCPP_WARN(
        this->get_logger(),
        "Serial write incomplete: wrote %zd of %zu bytes",
        written,
        sizeof(packet));
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleCmdPublish>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
