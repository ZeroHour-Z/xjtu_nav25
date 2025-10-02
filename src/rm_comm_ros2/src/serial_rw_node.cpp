#include "packet_utils.hpp"
#include "protocol.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;

// 本节点直接操作串口设备：
// - 参数：port=/dev/ttyACM0, baud=115200
// - 订阅: /rm_comm/tx_packet (handler 打包好的 64 字节，std_msgs/String)
// - 发布: /rm_comm/rx_packet (从串口读取的原始数据，std_msgs/String)
// - 可选: 当参数 use_shm=true 时，使用 POSIX 共享内存 + 信号量替代 ROS topic 进行进程间通信

namespace {

  static speed_t to_baud_constant(int baud) {
    switch (baud) {
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
    default:
      return B115200;
    }
  }

  struct ShmTx {
    uint32_t len;
    uint8_t  data[64];
  };
  struct ShmRx {
    uint32_t len;
    uint8_t  data[512];
  };

} // namespace

class SerialRwNode : public rclcpp::Node {
public:
  SerialRwNode() : Node("serial_rw_node") {
    port_               = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_               = this->declare_parameter<int>("baud", 115200);
    reopen_interval_ms_ = this->declare_parameter<int>("reopen_interval_ms", 500);

    use_shm_  = this->declare_parameter<bool>("use_shm", false);
    shm_name_ = this->declare_parameter<std::string>("shm_name", "/rm_comm_shm");

    tx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/rm_comm/tx_packet", 10,
        std::bind(&SerialRwNode::onTxPacket, this, std::placeholders::_1));

    rx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rm_comm/rx_packet", 10);
    running_.store(true);

    if (use_shm_) {
      setupSharedMemory();
      shm_tx_thread_ = std::thread(&SerialRwNode::shmTxLoop, this);
    }

    read_thread_ = std::thread(&SerialRwNode::readLoop, this);

    RCLCPP_INFO(this->get_logger(), "serial_rw_node started. Using port %s, baud %d, use_shm=%s",
                port_.c_str(), baud_, use_shm_ ? "true" : "false");
  }

  ~SerialRwNode() override {
    running_.store(false);
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (shm_tx_thread_.joinable()) {
      // Post semaphore to unblock if waiting
      if (tx_sem_) sem_post(tx_sem_);
      shm_tx_thread_.join();
    }

    closeSharedMemory();
    closeSerial();
  }

private:
  void onTxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (msg->data.size() != 64) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "tx_packet size %zu != 64, drop", msg->data.size());
      return;
    }

    // if (use_shm_) {
    //   if (!tx_shm_) {
    //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                          "tx_shm not ready; drop tx");
    //     return;
    //   }
    //   // copy into shared memory and signal
    //   std::memcpy(tx_shm_->data, msg->data.data(), 64);
    //   tx_shm_->len = 64;
    //   if (tx_sem_) sem_post(tx_sem_);
    //   return;
    // }

    if (fd_ < 0) {
      // 触发重连由读线程负责，此处提示
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "serial not open; drop tx");
      return;
    }

    const uint8_t* data      = reinterpret_cast<const uint8_t*>(msg->data.data());
    size_t         remaining = msg->data.size();
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

  void shmTxLoop() {
    while (running_.load()) {
      if (!tx_sem_) {
        std::this_thread::sleep_for(10ms);
        continue;
      }
      if (sem_wait(tx_sem_) != 0) {
        if (!running_.load()) break;
        continue;
      }
      if (!running_.load()) break;
      if (!tx_shm_) continue;
      uint32_t len = tx_shm_->len;
      if (len == 0 || len > sizeof(tx_shm_->data)) continue;
      // write to serial
      if (fd_ < 0) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "serial not open; drop shm tx");
        continue;
      }
      const uint8_t* data      = tx_shm_->data;
      size_t         remaining = len;
      while (remaining > 0) {
        ssize_t n = ::write(fd_, data, remaining);
        if (n < 0) {
          if (errno == EINTR) continue;
          RCLCPP_ERROR(this->get_logger(), "write error (shmTx): %s", std::strerror(errno));
          closeSerial();
          break;
        }
        remaining -= static_cast<size_t>(n);
        data += n;
      }
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

      char    buf[512];
      ssize_t n = ::read(fd_, buf, sizeof(buf));
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "read n: %ld", n);

      navCommand_t n_data;

      if (n > 0) {
        auto head = int(buf[0]);  // 获取头部
        auto tail = int(buf[63]); // 获取尾部
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Head: %d Tail: %d",
                             head, tail);

        if (head == 0x71) {
          if (n != 64) {
            std::cerr << "Error: Received packet with 0x71 head but size is not 64 bytes"
                      << std::endl;
            continue;
          }
          // std::memcpy(nullptr, buf, n));
        } else if (head == 0x72) {
          if (n > 64) {
            std::cerr << "Error: Received packet with 0x72 head but size is larger than max size 64"
                      << std::endl;
            continue;
          }
          std::memcpy(&n_data, buf, n);
          // RCLCPP_INFO(this->get_logger(), "bullet_remain: %d", (int)n_data.bullet_remain);
          // RCLCPP_INFO(this->get_logger(), "hp_remain: %d", (int)n_data.hp_remain);
          // RCLCPP_INFO(this->get_logger(), "color: %d", (int)n_data.color);
          std_msgs::msg::UInt8MultiArray out_msg;
          const uint8_t*                 byte_ptr  = reinterpret_cast<const uint8_t*>(&n_data);
          size_t                         data_size = sizeof(navCommand_t);
          out_msg.data.assign(byte_ptr, byte_ptr + data_size);
          rx_pub_->publish(out_msg);
          // std::string str_data = rm_comm_ros2::to_bytes(n_data);
          // std_msgs::msg::String out;
          // out.data = str_data;
          // RCLCPP_INFO(this->get_logger(), "str: %zu", str_data.size());
          // rx_pub_->publish(out);
        }
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
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "open %s failed: %s",
                            port_.c_str(), std::strerror(errno));
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
    tio.c_cflag &= ~CRTSCTS; // 无硬件流控
    tio.c_cflag &= ~CSTOPB;  // 1 个停止位
    tio.c_cflag &= ~PARENB;  // 无校验
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8; // 8 位

    speed_t spd = to_baud_constant(baud_);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // 读阻塞条件：至少 1 字节即可返回
    tio.c_cc[VMIN]  = 1;
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

  void setupSharedMemory() {
    std::string tx_name     = shm_name_ + std::string("_tx");
    std::string rx_name     = shm_name_ + std::string("_rx");
    std::string tx_sem_name = shm_name_ + std::string("_tx_sem");
    std::string rx_sem_name = shm_name_ + std::string("_rx_sem");

    // open or create
    tx_shm_fd_ = shm_open(tx_name.c_str(), O_CREAT | O_RDWR, 0666);
    rx_shm_fd_ = shm_open(rx_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (tx_shm_fd_ < 0 || rx_shm_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "shm_open failed: %s", std::strerror(errno));
      return;
    }
    ftruncate(tx_shm_fd_, sizeof(ShmTx));
    ftruncate(rx_shm_fd_, sizeof(ShmRx));

    tx_shm_ = static_cast<ShmTx*>(
        mmap(nullptr, sizeof(ShmTx), PROT_READ | PROT_WRITE, MAP_SHARED, tx_shm_fd_, 0));
    rx_shm_ = static_cast<ShmRx*>(
        mmap(nullptr, sizeof(ShmRx), PROT_READ | PROT_WRITE, MAP_SHARED, rx_shm_fd_, 0));
    if (tx_shm_ == MAP_FAILED || rx_shm_ == MAP_FAILED) {
      RCLCPP_WARN(this->get_logger(), "mmap failed");
      tx_shm_ = nullptr;
      rx_shm_ = nullptr;
      return;
    }
    // initialize lengths
    tx_shm_->len = 0;
    rx_shm_->len = 0;

    tx_sem_ = sem_open(tx_sem_name.c_str(), O_CREAT, 0666, 0);
    rx_sem_ = sem_open(rx_sem_name.c_str(), O_CREAT, 0666, 0);
    if (tx_sem_ == SEM_FAILED || rx_sem_ == SEM_FAILED) {
      RCLCPP_WARN(this->get_logger(), "sem_open failed");
      if (tx_sem_ != SEM_FAILED) sem_close(tx_sem_);
      if (rx_sem_ != SEM_FAILED) sem_close(rx_sem_);
      tx_sem_ = nullptr;
      rx_sem_ = nullptr;
    }
  }

  void closeSharedMemory() {
    if (tx_shm_) {
      munmap(tx_shm_, sizeof(ShmTx));
      tx_shm_ = nullptr;
    }
    if (rx_shm_) {
      munmap(rx_shm_, sizeof(ShmRx));
      rx_shm_ = nullptr;
    }
    if (tx_shm_fd_ >= 0) {
      close(tx_shm_fd_);
      tx_shm_fd_ = -1;
    }
    if (rx_shm_fd_ >= 0) {
      close(rx_shm_fd_);
      rx_shm_fd_ = -1;
    }
    if (tx_sem_ && tx_sem_ != SEM_FAILED) {
      sem_close(tx_sem_);
      tx_sem_ = nullptr;
    }
    if (rx_sem_ && rx_sem_ != SEM_FAILED) {
      sem_close(rx_sem_);
      rx_sem_ = nullptr;
    }
    // Note: do not shm_unlink/sem_unlink here to allow other process reuse
  }

  // members
  std::string port_;
  int         baud_{115200};
  int         reopen_interval_ms_{500};

  bool        use_shm_{false};
  std::string shm_name_{"/rm_comm_shm"};

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr    rx_pub_;

  std::thread       read_thread_;
  std::thread       shm_tx_thread_;
  std::atomic<bool> running_{false};
  int               fd_{-1};

  // shared mem
  int    tx_shm_fd_{-1};
  int    rx_shm_fd_{-1};
  ShmTx* tx_shm_{nullptr};
  ShmRx* rx_shm_{nullptr};
  sem_t* tx_sem_{nullptr};
  sem_t* rx_sem_{nullptr};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialRwNode>());
  rclcpp::shutdown();
  return 0;
}