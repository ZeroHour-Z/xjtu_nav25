#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include <cstring>
#include <cmath>

#include "protocol.h"
#include "rm_comm_ros2/packet_utils.hpp"

#pragma pack(push, 1)
struct NavInfoAligned64 {
	navInfo_t payload;  // 结构体本身应为 64 字节
};
#pragma pack(pop)

static_assert(sizeof(navInfo_t) == 64, "navInfo_t must be 64 bytes");
static_assert(sizeof(NavInfoAligned64) == 64, "Wrapped navInfo must be 64 bytes");

class HandlerNode : public rclcpp::Node {
public:
	HandlerNode() : Node("handler_node") {
		tx_pub_ = this->create_publisher<std_msgs::msg::String>("/rm_comm/tx_packet", 10);

		rx_sub_ = this->create_subscription<std_msgs::msg::String>(
			"/rm_comm/rx_packet", 100,
			std::bind(&HandlerNode::onRxPacket, this, std::placeholders::_1));

		cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
			"/cmd_vel_pid", 10,
			std::bind(&HandlerNode::onCmdVel, this, std::placeholders::_1));

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"/Odometry", 10,
			std::bind(&HandlerNode::onOdom, this, std::placeholders::_1));

		// 只声明一次参数，避免重复声明异常
		this->declare_parameter<double>("tx_hz", 100.0);
		this->declare_parameter<double>("target_x", 0.0);
		this->declare_parameter<double>("target_y", 0.0);

		double hz = 100.0;
		this->get_parameter("tx_hz", hz);
		tx_timer_ = this->create_wall_timer(
			std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, hz))),
			std::bind(&HandlerNode::publishTxPacket, this));

		RCLCPP_INFO(this->get_logger(), "handler_node started.");
	}

private:
	void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
		nav_info_.x_speed = msg->linear.x;
		nav_info_.y_speed = -1.0f * static_cast<float>(msg->linear.y);
		nav_info_.yaw = static_cast<float>(msg->angular.x);
		nav_info_.desire_angle = static_cast<float>(msg->angular.z);
	}

	void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
		nav_info_.coo_x_current = static_cast<float>(msg->pose.pose.position.x);
		nav_info_.coo_y_current = static_cast<float>(msg->pose.pose.position.y);
	}

	void onRxPacket(const std_msgs::msg::String::SharedPtr msg) {
		auto packets = rm_comm_ros2::extract_packets<navCommand_t>(
			msg->data,
			/*frame_header=*/0x72,
			[](const navCommand_t &p){ return p.frame_tail == 0x21; }
		);
		for (const auto &p : packets) {
			last_cmd_ = p;
			handleNavCommand(last_cmd_);
		}
	}

	void handleNavCommand(const navCommand_t &cmd) {
		last_stop_ = cmd.stop;
		last_color_ = cmd.color;
	}

	void publishTxPacket() {
		nav_info_.frame_header = 0x72;
		nav_info_.frame_tail = 0x4D;
		nav_info_.modified_flag = (std::fabs(nav_info_.x_speed) > 1e-6 || std::fabs(nav_info_.y_speed) > 1e-6) ? 1 : 0;

		double target_x = 0.0, target_y = 0.0;
		(void)this->get_parameter("target_x", target_x);
		(void)this->get_parameter("target_y", target_y);
		nav_info_.target_x = static_cast<float>(target_x);
		nav_info_.target_y = static_cast<float>(target_y);

		std_msgs::msg::String out;
		out.data = rm_comm_ros2::to_bytes(nav_info_);
		if (out.data.size() != sizeof(navInfo_t)) {
			RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "serialized size != 64");
			return;
		}
		tx_pub_->publish(out);
	}

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tx_pub_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr rx_sub_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

	rclcpp::TimerBase::SharedPtr tx_timer_;

	navInfo_t nav_info_{};
	navCommand_t last_cmd_{};
	uint8_t last_stop_{0};
	uint8_t last_color_{0};
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HandlerNode>());
	rclcpp::shutdown();
	return 0;
} 