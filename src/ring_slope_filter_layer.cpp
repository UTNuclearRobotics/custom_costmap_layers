// Copyright 2024 University of Texas Nuclear Robotics Group
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "custom_costmap_layers/ring_slope_filter_layer.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace custom_costmap_layers
{

RingSlopeFilterLayer::RingSlopeFilterLayer()
: slope_threshold_(1.0),
  min_dz_(0.03)
{
}

void RingSlopeFilterLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RingSlopeFilterLayer: failed to lock node");
  }

  declareParameter("input_topic", rclcpp::ParameterValue(std::string("/ouster/points")));
  declareParameter("output_topic", rclcpp::ParameterValue(std::string("/ouster/points_filtered")));
  declareParameter("slope_threshold", rclcpp::ParameterValue(1.0));
  declareParameter("min_dz", rclcpp::ParameterValue(0.03));

  node->get_parameter(name_ + "." + "input_topic",  input_topic_);
  node->get_parameter(name_ + "." + "output_topic", output_topic_);
  node->get_parameter(name_ + "." + "slope_threshold", slope_threshold_);
  node->get_parameter(name_ + "." + "min_dz", min_dz_);

  RCLCPP_INFO(
    node->get_logger(),
    "[RingSlopeFilterLayer] input: %s  output: %s  slope_threshold: %.2f  min_dz: %.3f m",
    input_topic_.c_str(), output_topic_.c_str(), slope_threshold_, min_dz_);

  // QoS matched to the typical sensor best-effort profile
  auto qos = rclcpp::SensorDataQoS();

  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, qos,
    std::bind(&RingSlopeFilterLayer::cloudCallback, this, std::placeholders::_1));

  pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

  current_ = true;
}

void RingSlopeFilterLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * /*min_x*/, double * /*min_y*/,
  double * /*max_x*/, double * /*max_y*/)
{
  // This layer does not mark the costmap directly; nothing to do here.
}

void RingSlopeFilterLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  // This layer does not mark the costmap directly; nothing to do here.
}

void RingSlopeFilterLayer::cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Work on a copy so we don't mutate the original message in the middleware
  auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
  applyRingSlopeFilter(*filtered);
  pub_->publish(std::move(*filtered));
}

void RingSlopeFilterLayer::applyRingSlopeFilter(sensor_msgs::msg::PointCloud2 & cloud) const
{
  if (cloud.height <= 1) { return; }

  int x_off = -1, y_off = -1, z_off = -1;
  for (const auto & field : cloud.fields) {
    if (field.name == "x") { x_off = static_cast<int>(field.offset); }
    if (field.name == "y") { y_off = static_cast<int>(field.offset); }
    if (field.name == "z") { z_off = static_cast<int>(field.offset); }
  }
  if (x_off < 0 || y_off < 0 || z_off < 0) { return; }

  const uint32_t W  = cloud.width;
  const uint32_t H  = cloud.height;
  const uint32_t ps = cloud.point_step;
  const uint32_t rs = cloud.row_step;
  uint8_t * data    = cloud.data.data();

  auto read_f = [&](uint32_t row, uint32_t col, int off) -> float {
    float v;
    std::memcpy(&v, data + row * rs + col * ps + off, sizeof(float));
    return v;
  };

  auto nan_point = [&](uint32_t row, uint32_t col) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    uint8_t * p = data + row * rs + col * ps;
    std::memcpy(p + x_off, &nan, sizeof(float));
    std::memcpy(p + y_off, &nan, sizeof(float));
    std::memcpy(p + z_off, &nan, sizeof(float));
  };

  // Per-column working buffer: only finite returns, sorted by z
  struct RingPoint { float x, y, z; uint32_t row; };
  std::vector<RingPoint> col_pts;
  col_pts.reserve(H);

  const float slope_thresh = static_cast<float>(slope_threshold_);
  const float min_dz_f     = static_cast<float>(min_dz_);
  constexpr float kMinDxy  = 1e-3f;

  // Returns dz/dxy between two points.
  // Returns FLT_MAX (i.e. "keep") if below noise floor or near-vertical.
  auto slope_ratio = [&](const RingPoint & a, const RingPoint & b) -> float {
    const float dz  = std::abs(b.z - a.z);
    const float dxy = std::sqrt((b.x - a.x) * (b.x - a.x) +
                                (b.y - a.y) * (b.y - a.y));
    if (dz < min_dz_f || dxy < kMinDxy) {
      return std::numeric_limits<float>::max();  // treat as obstacle (keep)
    }
    return dz / dxy;
  };

  for (uint32_t col = 0; col < W; ++col) {
    col_pts.clear();

    // Collect all finite returns in this azimuth column
    for (uint32_t row = 0; row < H; ++row) {
      const float x = read_f(row, col, x_off);
      const float y = read_f(row, col, y_off);
      const float z = read_f(row, col, z_off);
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        col_pts.push_back({x, y, z, row});
      }
    }

    // Need at least 3 points to have any interior points to evaluate
    if (col_pts.size() < 3) { continue; }

    // Sort by z (ascending) to get true elevation order regardless of
    // how the driver orders beam rows in the organized cloud
    std::sort(col_pts.begin(), col_pts.end(),
      [](const RingPoint & a, const RingPoint & b) { return a.z < b.z; });

    // For each interior point, require BOTH its elevation neighbors to
    // independently agree it looks like ground before suppressing it.
    // This protects obstacle edges (one neighbor is on the obstacle,
    // one is on the ground → they won't both agree → point is kept).
    const size_t N = col_pts.size();
    for (size_t i = 1; i < N - 1; ++i) {
      const float r_below = slope_ratio(col_pts[i - 1], col_pts[i]);
      const float r_above = slope_ratio(col_pts[i],     col_pts[i + 1]);

      if (r_below <= slope_thresh && r_above <= slope_thresh) {
        nan_point(col_pts[i].row, col);
      }
    }
    // First and last points in the column are skipped — they have only
    // one neighbor so there is no second vote to confirm ground.
    // The lowest return in a column is almost always ground anyway.
  }
}

}  // namespace custom_costmap_layers

PLUGINLIB_EXPORT_CLASS(custom_costmap_layers::RingSlopeFilterLayer, nav2_costmap_2d::Layer)