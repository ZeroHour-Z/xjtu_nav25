#include "protocol.h"
#include "rm_comm_ros2/packet_utils.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <vector>

struct ShmTx {
  uint32_t len;
  uint8_t  data[64];
};
struct ShmRx {
  uint32_t len;
  uint8_t  data[512];
};

class HandlerNode : public rclcpp::Node {
public:
  HandlerNode() : Node("handler_node") {
    patrol_group_pub_ = this->create_publisher<std_msgs::msg::String>("/patrol_group", 10);

    tx_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/rm_comm/tx_packet", 10);

    rx_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/rm_comm/rx_packet", 100,
        std::bind(&HandlerNode::onRxPacket, this, std::placeholders::_1));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&HandlerNode::onCmdVel, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry", 10, std::bind(&HandlerNode::onOdom, this, std::placeholders::_1));

    // 只声明一次参数，避免重复声明异常
    this->declare_parameter<double>("tx_hz", 100.0);
    this->declare_parameter<double>("target_x", 0.0);
    this->declare_parameter<double>("target_y", 0.0);

    this->declare_parameter<bool>("use_shm", false);
    this->declare_parameter<std::string>("shm_name", "/rm_comm_shm");

    this->get_parameter("use_shm", use_shm_);
    this->get_parameter("shm_name", shm_name_);

    double hz = 100.0;
    this->get_parameter("tx_hz", hz);
    tx_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz))),
        std::bind(&HandlerNode::publishTxPacket, this));

    if (use_shm_) setupSharedMemory();

    RCLCPP_INFO(this->get_logger(), "handler_node started. use_shm=%s",
                use_shm_ ? "true" : "false");
  }

  ~HandlerNode() override {
    running_.store(false);
    if (shm_rx_thread_.joinable()) {
      // post to unblock
      if (rx_sem_) sem_post(rx_sem_);
      shm_rx_thread_.join();
    }
    closeSharedMemory();
  }

private:
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(),"cmd_vel:%f",msg->linear.x);
    nav_info_.x_speed     = msg->linear.x / 3.0;
    nav_info_.y_speed     = msg->linear.y / 3.0;
    nav_info_.yaw_current = msg->angular.x;
    nav_info_.yaw_desired = msg->angular.z;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    nav_info_.x_current = static_cast<float>(msg->pose.pose.position.x);
    nav_info_.y_current = static_cast<float>(msg->pose.pose.position.y);
  }

  void onRxPacket(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (msg->data[0] != 0x72) {
      RCLCPP_WARN(this->get_logger(), "Invalid frame header: expected 0x72, got 0x%02X",
                  msg->data[0]);
      return;
    }

    // 2. 数据转换：使用 std::memcpy 将字节流复制到结构体变量中
    navCommand_t received_cmd;
    std::memcpy(&received_cmd,       // 目标地址：结构体变量的内存地址
                msg->data.data(),    // 源地址：vector数据的起始内存地址
                sizeof(navCommand_t) // 复制长度：结构体的大小
    );

    // 3. 协议验证：在转换成结构体后，检查帧尾等成员变量是否符合协议
    if (received_cmd.frame_tail != 0x21) {
      // RCLCPP_WARN(this->get_logger(), "Invalid frame tail: expected 0x21, got 0x%02X",
      // received_cmd.frame_tail);
      return;
    }
    // 4. 调用处理函数：所有检查通过后，数据有效，进行处理
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Successfully decoded navCommand_t packet.");
    last_cmd_ = received_cmd;
    // RCLCPP_WARN(this->get_logger(), "Bullet: %d", (int)received_cmd.bullet_remain);
    // handleNavCommand(last_cmd_);
    // RCLCPP_INFO(this->get_logger(), "Time_test: %f Now: %f", received_cmd.time_test,
    //             static_cast<float>(this->now().seconds() - 1759396608.000000));
  }

  void shmRxLoop() {
    running_.store(true);
    while (running_.load()) {
      if (!rx_sem_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      if (sem_wait(rx_sem_) != 0) {
        if (!running_.load()) break;
        continue;
      }
      if (!running_.load()) break;
      if (!rx_shm_) continue;
      uint32_t len = rx_shm_->len;
      if (len == 0 || len > sizeof(rx_shm_->data)) continue;
      std::string s(reinterpret_cast<char*>(rx_shm_->data), len);
      auto        packets = rm_comm_ros2::extract_packets<navCommand_t>(
          s,
          /*frame_header=*/0x72, [](const navCommand_t& p) { return p.frame_tail == 0x21; });
      for (const auto& p : packets) {
        last_cmd_ = p;
        handleNavCommand(last_cmd_);
      }
    }
  }

  void handleNavCommand(const navCommand_t& cmd) {
    // std::cout << "hp_remain:" << cmd.hp_remain << std::endl;
    printf("bullet_remain:%d\n", cmd.bullet_remain);
    // Best-effort mirror of hp/ammo to BT node params
    static const std::string kBtNodeName = "/rm_bt_decision_node";
    if (!bt_param_client_) {
      bt_param_client_ =
          std::make_shared<rclcpp::SyncParametersClient>(this->shared_from_this(), kBtNodeName);
    }
    if (!bt_param_client_->service_is_ready()) return;

    // Parameter names (fallback to defaults if not configured)
    std::string hp_param   = "hp";
    std::string ammo_param = "ammo";

    // Write values
    std::vector<rclcpp::Parameter> ps = {
        rclcpp::Parameter(hp_param, static_cast<double>(cmd.hp_remain)),
        rclcpp::Parameter(ammo_param, static_cast<double>(cmd.bullet_remain)),
    };
    try {
      (void) bt_param_client_->set_parameters(ps);
    } catch (...) {
      // ignore
    }

    // auto cmd.sentry_command;
    // patrol_group_pub_->publish(std_msgs::msg::String());
  }

  void publishTxPacket() {
    nav_info_.frame_header = 0x72;
    nav_info_.frame_tail   = 0x4D;

    double target_x = 0.0, target_y = 0.0;
    (void) this->get_parameter("target_x", target_x);
    (void) this->get_parameter("target_y", target_y);
    nav_info_.x_target  = static_cast<float>(target_x);
    nav_info_.y_target  = static_cast<float>(target_y);
    // nav_info_.time_test = static_cast<float>(this->now().seconds() - 1759396608.000000);
    // RCLCPP_INFO(this->get_logger(), "send: %f", nav_info_.time_test);

    std_msgs::msg::UInt8MultiArray out_msg;
    const uint8_t*                 byte_ptr  = reinterpret_cast<const uint8_t*>(&nav_info_);
    size_t                         data_size = sizeof(navCommand_t);
    out_msg.data.assign(byte_ptr, byte_ptr + data_size);
    tx_pub_->publish(out_msg);
  }

  void setupSharedMemory() {
    std::string tx_name     = shm_name_ + std::string("_tx");
    std::string rx_name     = shm_name_ + std::string("_rx");
    std::string tx_sem_name = shm_name_ + std::string("_tx_sem");
    std::string rx_sem_name = shm_name_ + std::string("_rx_sem");

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

    // start thread to wait for incoming rx from serial
    shm_rx_thread_ = std::thread(&HandlerNode::shmRxLoop, this);
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
  }

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          patrol_group_pub_;

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_sub_;

  rclcpp::TimerBase::SharedPtr tx_timer_;

  navInfo_t    nav_info_{};
  navCommand_t last_cmd_{};
  uint8_t      last_stop_{0};
  uint8_t      last_color_{0};

  // shm members
  bool              use_shm_{false};
  std::string       shm_name_{"/rm_comm_shm"};
  int               tx_shm_fd_{-1};
  int               rx_shm_fd_{-1};
  ShmTx*            tx_shm_{nullptr};
  ShmRx*            rx_shm_{nullptr};
  sem_t*            tx_sem_{nullptr};
  sem_t*            rx_sem_{nullptr};
  std::thread       shm_rx_thread_;
  std::atomic<bool> running_{true};

  // Parameter client for BT node
  std::shared_ptr<rclcpp::SyncParametersClient> bt_param_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HandlerNode>());
  rclcpp::shutdown();
  return 0;
}
