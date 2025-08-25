#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

// 本节点直接操作串口设备：
// - 参数：port=/dev/ttyACM0, baud=115200
// - 订阅: /rm_comm/tx_packet (handler 打包好的 64 字节，std_msgs/String)
// - 发布: /rm_comm/rx_packet (从串口读取的原始数据，std_msgs/String)

namespace {

static speed_t to_baud_constant(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
  }
}

}

class SerialRwNode : public rclcpp::Node {
public:
  SerialRwNode() : Node("serial_rw_node") {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = this->declare_parameter<int>("baud", 115200);
    reopen_interval_ms_ = this->declare_parameter<int>("reopen_interval_ms", 500);

    tx_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/rm_comm/tx_packet", 10,
      std::bind(&SerialRwNode::onTxPacket, this, std::placeholders::_1));

    rx_pub_ = this->create_publisher<std_msgs::msg::String>("/rm_comm/rx_packet", 10);

    running_.store(true);
    read_thread_ = std::thread(&SerialRwNode::readLoop, this);

    RCLCPP_INFO(this->get_logger(), "serial_rw_node started. Using port %s, baud %d", port_.c_str(), baud_);
  }

  ~SerialRwNode() override {
    running_.store(false);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    closeSerial();
  }

private:
  void onTxPacket(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.size() != 64) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tx_packet size %zu != 64, drop", msg->data.size());
      return;
    }
    if (fd_ < 0) {
      // 触发重连由读线程负责，此处提示
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "serial not open; drop tx");
      return;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t*>(msg->data.data());
    size_t remaining = msg->data.size();
    while (remaining > 0) {
      ssize_t n = ::write(fd_, data, remaining);
      if (n < 0) {
        if (errno == EINTR) continue;
        RCLCPP_ERROR(this->get_logger(), "write error: %s", std::strerror(errno));
        closeSerial();
        return;
      }
      remaining -= static_cast<size_t>(n);
      data += n;
    }
  }

  void readLoop() {
    while (running_.load()) {
      if (fd_ < 0) {
        if (!openSerial()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(reopen_interval_ms_));
          continue;
        }
      }

      uint8_t buf[512];
      ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n > 0) {
        std_msgs::msg::String out;
        out.data.assign(reinterpret_cast<char*>(buf), reinterpret_cast<char*>(buf) + n);
        rx_pub_->publish(out);
        continue;
      }

      if (n == 0) {
        // 无数据，稍作等待
        std::this_thread::sleep_for(1ms);
        continue;
      }

      // n < 0
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(1ms);
        continue;
      }

      RCLCPP_WARN(this->get_logger(), "read error: %s", std::strerror(errno));
      closeSerial();
      std::this_thread::sleep_for(std::chrono::milliseconds(reopen_interval_ms_));
    }
  }

  bool openSerial() {
    closeSerial();
    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "open %s failed: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }

    // 配置 8N1，无流控，原始模式
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;     // 无硬件流控
    tio.c_cflag &= ~CSTOPB;      // 1 个停止位
    tio.c_cflag &= ~PARENB;      // 无校验
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;          // 8 位

    speed_t spd = to_baud_constant(baud_);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // 读阻塞条件：至少 1 字节即可返回
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 1; // 0.1s 单位，设置 1 即 ~100ms 超时

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }

    // 设为阻塞模式，结合 VMIN/VTIME 控制
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) {
      fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    fd_ = fd;
    RCLCPP_INFO(this->get_logger(), "Opened %s @ %d", port_.c_str(), baud_);
    return true;
  }

  void closeSerial() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
      RCLCPP_INFO(this->get_logger(), "Serial closed");
    }
  }

  // members
  std::string port_;
  int baud_ {115200};
  int reopen_interval_ms_ {500};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_pub_;

  std::thread read_thread_;
  std::atomic<bool> running_ {false};
  int fd_ {-1};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialRwNode>());
  rclcpp::shutdown();
  return 0;
} 