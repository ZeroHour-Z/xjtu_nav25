#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

// 从原代码中保留的仿真器，用于闭环测试
#include "global_velocity_controller/simulator_2d.hpp"
// 注意: gvc::Simulator2D 依赖于 types.hpp, 这里假设它存在
#include "global_velocity_controller/types.hpp"

using std::placeholders::_1;

class SimplifiedControllerNode : public rclcpp::Node {
   public:
    SimplifiedControllerNode()
        : Node("simplified_velocity_controller"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_),
          tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)) {
        // --- 参数声明 ---
        declare_parameter<std::string>("map_frame", "map");
        declare_parameter<std::string>("base_frame", "base_link");
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
        declare_parameter<std::string>("path_topic", "/plan");

        // 位控PID增益
        declare_parameter<double>("kp_xy", 1.5);
        declare_parameter<double>("ki_xy", 0.1);
        declare_parameter<double>("kd_xy", 0.2);
        declare_parameter<double>("kp_yaw", 2.0);

        // 速度和加速度限制
        declare_parameter<double>("max_vx", 0.8);
        declare_parameter<double>("max_vy", 0.0);  // 通常对于非全向机器人，vy为0
        declare_parameter<double>("max_wz", 1.2);
        declare_parameter<double>("cmd_accel_limit_linear", 2.0);
        declare_parameter<double>("cmd_accel_limit_angular", 4.0);

        // 路径跟踪参数
        declare_parameter<double>("goal_tolerance", 0.1);
        // 航向预测时间因子（0.0~1.0，通常0.5~1.0）
        declare_parameter<double>("prediction_time_factor", 1.0);

        // --- 新增: 动态前瞻距离参数 ---
        declare_parameter<bool>("enable_dynamic_lookahead", true);
        declare_parameter<double>("lookahead_distance", 0.5);  // 如果动态关闭，则使用此固定值
        declare_parameter<double>("min_lookahead_distance", 0.3);
        declare_parameter<double>("max_lookahead_distance", 1.0);
        declare_parameter<double>("curvature_window_distance", 2.0);  // 计算曲率的路径长度
        declare_parameter<double>("curvature_low", 0.1);              // 曲率阈值下限
        declare_parameter<double>("curvature_high", 1.0);             // 曲率阈值上限

        // 仿真相关参数
        declare_parameter<bool>("simulate", true);
        declare_parameter<double>("sim_init_x", 0.0);
        declare_parameter<double>("sim_init_y", 0.0);
        declare_parameter<double>("sim_init_yaw", 0.0);
        declare_parameter<double>("max_dt", 0.05);

        // --- 获取参数 ---
        map_frame_ = get_parameter("map_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        goal_tolerance_ = get_parameter("goal_tolerance").as_double();
        max_dt_ = get_parameter("max_dt").as_double();
        prediction_time_factor_ = get_parameter("prediction_time_factor").as_double();

        kp_xy_ = get_parameter("kp_xy").as_double();
        ki_xy_ = get_parameter("ki_xy").as_double();
        kd_xy_ = get_parameter("kd_xy").as_double();
        kp_yaw_ = get_parameter("kp_yaw").as_double();

        max_vx_ = get_parameter("max_vx").as_double();
        max_vy_ = get_parameter("max_vy").as_double();
        max_wz_ = get_parameter("max_wz").as_double();
        cmd_accel_limit_linear_ = get_parameter("cmd_accel_limit_linear").as_double();
        cmd_accel_limit_angular_ = get_parameter("cmd_accel_limit_angular").as_double();

        // --- 新增: 获取动态前瞻距离参数 ---
        enable_dynamic_lookahead_ = get_parameter("enable_dynamic_lookahead").as_bool();
        lookahead_distance_ = get_parameter("lookahead_distance").as_double();
        min_lookahead_distance_ = get_parameter("min_lookahead_distance").as_double();
        max_lookahead_distance_ = get_parameter("max_lookahead_distance").as_double();
        curvature_window_distance_ = get_parameter("curvature_window_distance").as_double();
        curvature_low_ = get_parameter("curvature_low").as_double();
        curvature_high_ = get_parameter("curvature_high").as_double();

        // 配置仿真器
        simulate_ = get_parameter("simulate").as_bool();
        if (simulate_) {
            gvc::SimulatorConfig sim_config;
            sim_config.init_x = get_parameter("sim_init_x").as_double();
            sim_config.init_y = get_parameter("sim_init_y").as_double();
            sim_config.init_yaw = get_parameter("sim_init_yaw").as_double();
            simulator_ = gvc::Simulator2D(sim_config);
            RCLCPP_INFO(get_logger(), "Simulation mode is ON.");
        }

        // --- ROS接口 ---
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(get_parameter("cmd_vel_topic").as_string(), 10);
        path_sub_ = create_subscription<nav_msgs::msg::Path>(get_parameter("path_topic").as_string(), 10,
                                                             std::bind(&SimplifiedControllerNode::onPath, this, _1));

        control_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                           std::bind(&SimplifiedControllerNode::onControlTimer, this));

        RCLCPP_INFO(get_logger(), "Simplified Position-Control PID Node has started.");
    }

   private:
    void onPath(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            has_path_ = false;
            RCLCPP_WARN(get_logger(), "Received an empty path.");
        } else {
            last_path_ = *msg;
            has_path_ = true;
            // 重置PID积分和之前的状态，开始新的路径跟踪
            resetControllerState();
            RCLCPP_INFO(get_logger(), "New path received with %zu poses.", msg->poses.size());
        }
    }

    void onControlTimer() {
        const rclcpp::Time now = get_clock()->now();
        if (!has_path_) {
            // 没有路径时，停止机器人，并在仿真下持续发布TF以确保存在 base_link 帧
            publishZeroTwist();
            if (simulate_) {
                publishSimulatedTransform(now);
            }
            return;
        }

        // 1. 获取机器人当前位姿
        double current_x, current_y, current_yaw;
        if (!getCurrentPose(current_x, current_y, current_yaw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Could not get current robot pose.");
            publishZeroTwist();
            return;
        }

        // 2. 在路径上寻找前瞻点(Lookahead Point)
        geometry_msgs::msg::Point lookahead_pt;
        bool is_final_goal = false;
        if (!findLookaheadPoint(current_x, current_y, lookahead_pt, is_final_goal)) {
            RCLCPP_INFO_ONCE(get_logger(), "Path tracking complete or invalid path.");
            has_path_ = false;  // 标记路径结束
            publishZeroTwist();
            resetControllerState();
            return;
        }

        // 3. 计算位置误差 (在map坐标系下)
        const double ex = lookahead_pt.x - current_x;
        const double ey = lookahead_pt.y - current_y;

        // 如果接近最终目标点，则进行特殊处理
        const double dist_to_goal = std::hypot(ex, ey);
        if (is_final_goal && dist_to_goal < goal_tolerance_) {
            RCLCPP_INFO(get_logger(), "Goal reached!");
            has_path_ = false;  // 标记路径结束
            publishZeroTwist();
            resetControllerState();
            return;
        }

        // 4. 计算时间差 dt
        if (!has_prev_time_) {
            prev_time_ = now;
            has_prev_time_ = true;
            // --- 新增: 第一次循环时也记录航向角 ---
            if (!has_prev_yaw_) {
                prev_yaw_ = current_yaw;
                has_prev_yaw_ = true;
            }
            return;
        }
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;
        if (dt <= 0.0) return;
        dt = std::min(dt, max_dt_);

        // --- 核心修改 1: 航向角预测 ---
        double predicted_yaw = current_yaw;  // 默认情况下，预测值等于当前值
        if (has_prev_yaw_ && dt > 1e-6) {
            // 根据前后两次的角度差，估算当前角速度
            const double estimated_wz = normalizeAngle(current_yaw - prev_yaw_) / dt;
            
            // 预测一个短暂未来(例如半个控制周期)的航向角
            // 可调参数 prediction_time_factor_，建议设为0.5~1.0
            predicted_yaw = current_yaw + estimated_wz * dt * prediction_time_factor_;
            predicted_yaw = normalizeAngle(predicted_yaw);
            RCLCPP_INFO_THROTTLE(this->get_logger(),*this->get_clock(),1000, "Predicted yaw: %.3f (current: %.3f, estimated_wz: %.3f)", predicted_yaw, current_yaw, estimated_wz);
        }
        prev_yaw_ = current_yaw;  // 更新上一次的航向角
        has_prev_yaw_ = true;

        // 5. 位置PID控制器 -> 输出期望速度 (在map坐标系下)
        integral_x_ += ex * dt;
        integral_y_ += ey * dt;
        // 简单的积分抗饱和
        integral_x_ = std::clamp(integral_x_, -1.0, 1.0);
        integral_y_ = std::clamp(integral_y_, -1.0, 1.0);

        const double deriv_ex = (ex - prev_ex_) / dt;
        const double deriv_ey = (ey - prev_ey_) / dt;

        double vx_map_cmd = kp_xy_ * ex + ki_xy_ * integral_x_ + kd_xy_ * deriv_ex;
        double vy_map_cmd = kp_xy_ * ey + ki_xy_ * integral_y_ + kd_xy_ * deriv_ey;

        prev_ex_ = ex;
        prev_ey_ = ey;

        // 6. 航向控制,下位机只需要给出角度即可
        double target_yaw = std::atan2(ey, ex);

        // 7. 将map系速度指令转换为base_link系
        const double cos_yaw = std::cos(predicted_yaw);
        const double sin_yaw = std::sin(predicted_yaw);
        double vx_base_raw = cos_yaw * vx_map_cmd + sin_yaw * vy_map_cmd;
        double vy_base_raw = -sin_yaw * vx_map_cmd + cos_yaw * vy_map_cmd;

        // 8. 应用速度和加速度限制
        double vx_out = std::clamp(vx_base_raw, -max_vx_, max_vx_);
        double vy_out = std::clamp(vy_base_raw, -max_vy_, max_vy_);

        // 加速度限制
        const double max_dv = cmd_accel_limit_linear_ * dt;
        const double max_dw = cmd_accel_limit_angular_ * dt;
        vx_out = std::clamp(vx_out, last_cmd_vx_ - max_dv, last_cmd_vx_ + max_dv);
        vy_out = std::clamp(vy_out, last_cmd_vy_ - max_dv, last_cmd_vy_ + max_dv);

        // 9. 发布指令并更新仿真
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx_out;
        cmd.linear.y = vy_out;

        // angular都是特殊设置,作为当前角度和目标角度的容器
        cmd.angular.x = current_yaw;
        cmd.angular.z = target_yaw;
        cmd_pub_->publish(cmd);

        last_cmd_vx_ = cmd.linear.x;
        last_cmd_vy_ = cmd.linear.y;
        last_cmd_wz_ = cmd.angular.z;

        if (simulate_) {
            simulator_.integrateBodyCommand({cmd.linear.x, cmd.linear.y, 1.0}, dt);
            publishSimulatedTransform(now);
        }
    }

    // --- 辅助函数 ---

    bool getCurrentPose(double &x, double &y, double &yaw) {
        if (simulate_) {
            const auto sim_pose = simulator_.getPose();
            x = sim_pose.x;
            y = sim_pose.y;
            yaw = sim_pose.yaw;
            return true;
        }

        try {
            geometry_msgs::msg::TransformStamped tf_map_to_base =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
            x = tf_map_to_base.transform.translation.x;
            y = tf_map_to_base.transform.translation.y;

            tf2::Quaternion q;
            tf2::fromMsg(tf_map_to_base.transform.rotation, q);
            double roll, pitch;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            return true;

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", ex.what());
            return false;
        }
    }

    bool findLookaheadPoint(double rob_x, double rob_y, geometry_msgs::msg::Point &pt, bool &is_last) {
        if (last_path_.poses.empty()) return false;

        is_last = false;

        // 找到路径上离机器人最近的点的索引
        size_t closest_idx = 0;
        double min_dist_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < last_path_.poses.size(); ++i) {
            double dx = last_path_.poses[i].pose.position.x - rob_x;
            double dy = last_path_.poses[i].pose.position.y - rob_y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }

        // *** 核心修改: 计算动态前瞻距离 ***
        const double current_lookahead_distance = computeDynamicLookaheadDistance(closest_idx);

        // 从最近点开始向前搜索，直到找到一个距离机器人大于前瞻距离的点
        for (size_t i = closest_idx; i < last_path_.poses.size(); ++i) {
            double dx = last_path_.poses[i].pose.position.x - rob_x;
            double dy = last_path_.poses[i].pose.position.y - rob_y;
            if (std::hypot(dx, dy) > current_lookahead_distance) {
                pt = last_path_.poses[i].pose.position;
                return true;
            }
        }

        // 如果遍历完路径都找不到，说明机器人已经接近终点，直接返回路径的最后一个点
        pt = last_path_.poses.back().pose.position;
        is_last = true;
        return true;
    }

    // *** 新增函数: 根据路径曲率计算动态前瞻距离 ***
    double computeDynamicLookaheadDistance(std::size_t start_index) const {
        if (!enable_dynamic_lookahead_) {
            return lookahead_distance_;  // 返回固定的前瞻距离
        }
        if (last_path_.poses.size() < 3) {
            return min_lookahead_distance_;  // 路径太短，使用最小前瞻距离
        }

        double accumulated_length_m = 0.0;
        double sum_abs_curvature = 0.0;
        int curvature_samples = 0;

        std::size_t i = start_index;
        // 在一个窗口内计算平均曲率
        while (i + 2 < last_path_.poses.size() && accumulated_length_m < curvature_window_distance_) {
            const auto &p0 = last_path_.poses[i].pose.position;
            const auto &p1 = last_path_.poses[i + 1].pose.position;
            const auto &p2 = last_path_.poses[i + 2].pose.position;

            const double dx01 = p1.x - p0.x;
            const double dy01 = p1.y - p0.y;
            const double dx12 = p2.x - p1.x;
            const double dy12 = p2.y - p1.y;
            const double len01 = std::hypot(dx01, dy01);
            const double len12 = std::hypot(dx12, dy12);

            if (len01 > 1e-6 && len12 > 1e-6) {
                const double th0 = std::atan2(dy01, dx01);
                const double th1 = std::atan2(dy12, dx12);
                double dth = normalizeAngle(th1 - th0);
                const double seg_len = len12;
                const double kappa = std::fabs(dth) / std::max(seg_len, 1e-6);  // 曲率 = d_theta / d_length
                sum_abs_curvature += kappa;
                curvature_samples += 1;
            }
            accumulated_length_m += len12;
            i += 1;
        }

        const double mean_abs_curvature =
            (curvature_samples > 0) ? (sum_abs_curvature / static_cast<double>(curvature_samples)) : 0.0;

        // 根据平均曲率线性插值计算前瞻距离
        // 曲率越大，前瞻距离越小
        double alpha = 0.0;
        if (mean_abs_curvature <= curvature_low_) {
            alpha = 0.0;  // 低曲率，使用最大前瞻距离
        } else if (mean_abs_curvature >= curvature_high_) {
            alpha = 1.0;  // 高曲率，使用最小前瞻距离
        } else {
            // 在中间区域线性插值
            alpha = (mean_abs_curvature - curvature_low_) / std::max(1e-9, (curvature_high_ - curvature_low_));
        }

        const double lookahead = max_lookahead_distance_ - alpha * (max_lookahead_distance_ - min_lookahead_distance_);

        return std::clamp(lookahead, min_lookahead_distance_, max_lookahead_distance_);
    }

    void publishZeroTwist() {
        geometry_msgs::msg::Twist zero_twist;
        cmd_pub_->publish(zero_twist);
        last_cmd_vx_ = 0.0;
        last_cmd_vy_ = 0.0;
        last_cmd_wz_ = 0.0;
    }

    void resetControllerState() {
        integral_x_ = 0.0;
        integral_y_ = 0.0;
        prev_ex_ = 0.0;
        prev_ey_ = 0.0;
        has_prev_time_ = false;
        has_prev_yaw_ = false;
        prev_yaw_ = 0.0;
    }

    void publishSimulatedTransform(const rclcpp::Time &now) {
        const auto s = simulator_.getPose();
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = s.x;
        tf_msg.transform.translation.y = s.y;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, s.yaw);
        tf_msg.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(tf_msg);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ROS 接口
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 路径和状态
    nav_msgs::msg::Path last_path_;
    bool has_path_{false};

    // 参数
    std::string map_frame_, base_frame_;
    double goal_tolerance_;
    double max_dt_;
    double kp_xy_, ki_xy_, kd_xy_, kp_yaw_;
    double max_vx_, max_vy_, max_wz_;
    double cmd_accel_limit_linear_, cmd_accel_limit_angular_;
    double prediction_time_factor_;

    // 新增: 动态前瞻距离参数
    bool enable_dynamic_lookahead_;
    double lookahead_distance_;  // 固定值 (备用)
    double min_lookahead_distance_, max_lookahead_distance_;
    double curvature_window_distance_;
    double curvature_low_, curvature_high_;

    // PID 控制器状态
    bool has_prev_time_{false};
    rclcpp::Time prev_time_;
    double integral_x_{0.0}, integral_y_{0.0};
    double prev_ex_{0.0}, prev_ey_{0.0};
    double last_cmd_vx_{0.0}, last_cmd_vy_{0.0}, last_cmd_wz_{0.0};

    // --- 新增: 用于航向角预测 ---
    bool has_prev_yaw_{false};
    double prev_yaw_{0.0};

    // 仿真
    bool simulate_{false};
    gvc::Simulator2D simulator_{};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplifiedControllerNode>());
    rclcpp::shutdown();
    return 0;
}
