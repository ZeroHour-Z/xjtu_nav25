// =============================
// FILE: src/traversability_costmap_node.cpp
// =============================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cmath>

class TraversabilityCostmapNode : public rclcpp::Node {
public:
  TraversabilityCostmapNode()
  : Node("traversability_costmap_node") {
    // ---- Parameters ----
    input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/points");
    frame_id_    = this->declare_parameter<std::string>("frame_id", "map");
    resolution_  = this->declare_parameter<double>("resolution", 0.05); // m
    width_m_     = this->declare_parameter<double>("width_m", 20.0);    // m
    height_m_    = this->declare_parameter<double>("height_m", 20.0);   // m
    origin_x_    = this->declare_parameter<double>("origin_x", -10.0);  // m
    origin_y_    = this->declare_parameter<double>("origin_y", -10.0);  // m
    z_clip_min_  = this->declare_parameter<double>("z_clip_min", -2.0);
    z_clip_max_  = this->declare_parameter<double>("z_clip_max",  2.0);
    min_points_per_cell_ = this->declare_parameter<int>("min_points_per_cell", 3);
    step_threshold_m_    = this->declare_parameter<double>("step_threshold_m", 0.07); // 6~8cm for wheels
    step_max_threshold_m_ = this->declare_parameter<double>("step_max_threshold_m", 1000.0); // upper bound for obstacle range
    // Density-based criterion parameters
    density_min_pts_per_m3_ = this->declare_parameter<double>("density_min_pts_per_m3", 30.0); // pts/m^3
    min_points_for_density_ = this->declare_parameter<int>("min_points_for_density", 3);

    width_px_  = static_cast<int>(std::lround(width_m_  / resolution_));
    height_px_ = static_cast<int>(std::lround(height_m_ / resolution_));

    RCLCPP_INFO(get_logger(), "Grid %dx%d @ %.3fm, origin(%.2f, %.2f)", width_px_, height_px_,
                resolution_, origin_x_, origin_y_);

    // Precompute per-cell volume for density (m^3)
    cell_volume_m3_ = resolution_ * resolution_ * std::max(0.0, z_clip_max_ - z_clip_min_);
    if (cell_volume_m3_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Computed non-positive cell volume (%.6f). Check z_clip_min/max and resolution.", cell_volume_m3_);
      cell_volume_m3_ = std::numeric_limits<double>::epsilon();
    }

    // Publisher
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/traversability_costmap", 10);

    // Prepare fixed parts of OccupancyGrid
    grid_.header.frame_id = frame_id_;
    grid_.info.resolution = static_cast<float>(resolution_);
    grid_.info.width  = static_cast<uint32_t>(width_px_);
    grid_.info.height = static_cast<uint32_t>(height_px_);
    grid_.info.origin.position.x = origin_x_;
    grid_.info.origin.position.y = origin_y_;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0; // identity

    // Subscriber (sensor data QoS, best effort, transient local off)
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&TraversabilityCostmapNode::pcCallback, this, std::placeholders::_1));
  }

private:
  static inline float percentileSorted(const std::vector<float>& a, double p) {
    // p in [0,1], a must be sorted ascending
    if (a.empty()) return std::numeric_limits<float>::quiet_NaN();
    const double idx = p * (static_cast<double>(a.size()) - 1.0);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    if (hi == lo) return a[lo];
    const double w = idx - static_cast<double>(lo);
    return static_cast<float>(a[lo] + (a[hi] - a[lo]) * w);
  }

  void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert to PCL XYZ (we only need x,y,z)
    pcl::PointCloud<pcl::PointXYZ> cloud;
    try {
      pcl::fromROSMsg(*msg, cloud);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "fromROSMsg failed: %s", e.what());
      return;
    }
    if (cloud.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "Empty point cloud");
      return;
    }

    // Hash map: linear cell index -> vector of z's
    std::unordered_map<int, std::vector<float>> cell_z;
    cell_z.reserve(cloud.size() / 8 + 1);

    const int width = width_px_;
    const int height = height_px_;

    // Fill per-cell z lists
    for (const auto& p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.z < z_clip_min_ || p.z > z_clip_max_) continue;
      const int ix = static_cast<int>(std::floor((p.x - origin_x_) / resolution_));
      const int iy = static_cast<int>(std::floor((p.y - origin_y_) / resolution_));
      if (ix < 0 || ix >= width || iy < 0 || iy >= height) continue;
      const int idx = iy * width + ix;
      auto& vec = cell_z[idx];
      vec.push_back(static_cast<float>(p.z));
    }

    // Prepare OccupancyGrid data
    const int8_t UNKNOWN = -1;
    const int8_t FREE = 0;
    const int8_t LETHAL = 100;
    std::vector<int8_t> data(width * height, UNKNOWN);

    // Compute criteria per observed cell
    size_t n_obs = 0, n_lethal = 0;
    for (auto& kv : cell_z) {
      auto& zs = kv.second;
      const int num_points = static_cast<int>(zs.size());

      // Density criterion (pts/m^3)
      const double density_pts_per_m3 = static_cast<double>(num_points) / cell_volume_m3_;
      const bool density_sparse = (num_points >= min_points_for_density_) &&
                                  (density_pts_per_m3 < density_min_pts_per_m3_);

      bool lethal = false;

      // Step/elevation criterion only if enough points for robust quantiles
      if (num_points >= min_points_per_cell_) {
        std::sort(zs.begin(), zs.end());
        const float q10 = percentileSorted(zs, 0.10);
        const float q90 = percentileSorted(zs, 0.90);
        const float dh = q90 - q10;
        if ((dh >= static_cast<float>(step_threshold_m_)) &&
            (dh <= static_cast<float>(step_max_threshold_m_))) {
          lethal = true;
        }
      }

      // Apply density rule to capture voids/holes (low occupancy)
      if (density_sparse) {
        lethal = true;
      }

      data[kv.first] = lethal ? LETHAL : FREE;
      ++n_obs;
      if (lethal) ++n_lethal;
    }

    // Publish
    grid_.header.stamp = this->now();
    grid_.data = data;
    costmap_pub_->publish(grid_);

    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "cells observed=%zu, lethal=%zu", n_obs, n_lethal);
  }

  // Params
  std::string input_topic_;
  std::string frame_id_;
  double resolution_;
  double width_m_, height_m_;
  double origin_x_, origin_y_;
  double z_clip_min_, z_clip_max_;
  int min_points_per_cell_;
  double step_threshold_m_;
  double step_max_threshold_m_;
  double density_min_pts_per_m3_;
  int min_points_for_density_;

  // Derived
  int width_px_;
  int height_px_;
  double cell_volume_m3_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  nav_msgs::msg::OccupancyGrid grid_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityCostmapNode>());
  rclcpp::shutdown();
  return 0;
}