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
  // The filter requires an organized cloud so that adjacent rows correspond
  // to adjacent elevation rings at the same azimuth column.
  // Ouster lidars always publish organized clouds. Skip silently otherwise.
  if (cloud.height <= 1) {
    return;
  }

  // Locate x, y, z field byte offsets within the point stride
  int x_off = -1, y_off = -1, z_off = -1;
  for (const auto & field : cloud.fields) {
    if (field.name == "x") { x_off = static_cast<int>(field.offset); }
    if (field.name == "y") { y_off = static_cast<int>(field.offset); }
    if (field.name == "z") { z_off = static_cast<int>(field.offset); }
  }
  if (x_off < 0 || y_off < 0 || z_off < 0) {
    return;
  }

  const uint32_t W          = cloud.width;
  const uint32_t H          = cloud.height;
  const uint32_t point_step = cloud.point_step;
  const uint32_t row_step   = cloud.row_step;
  uint8_t * data             = cloud.data.data();

  auto read_f = [&](uint32_t row, uint32_t col, int off) -> float {
    float v;
    std::memcpy(&v, data + row * row_step + col * point_step + off, sizeof(float));
    return v;
  };

  auto nan_point = [&](uint32_t row, uint32_t col) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    uint8_t * p = data + row * row_step + col * point_step;
    std::memcpy(p + x_off, &nan, sizeof(float));
    std::memcpy(p + y_off, &nan, sizeof(float));
    std::memcpy(p + z_off, &nan, sizeof(float));
  };

  // Walk every azimuth column, comparing each ring with the next.
  for (uint32_t col = 0; col < W; ++col) {
    for (uint32_t row = 0; row < H - 1; ++row) {
      const float x0 = read_f(row,     col, x_off);
      const float y0 = read_f(row,     col, y_off);
      const float z0 = read_f(row,     col, z_off);
      const float x1 = read_f(row + 1, col, x_off);
      const float y1 = read_f(row + 1, col, y_off);
      const float z1 = read_f(row + 1, col, z_off);

      // Skip pairs that contain invalid returns
      if (!std::isfinite(x0) || !std::isfinite(x1)) {
        continue;
      }

      const float dz  = std::abs(z1 - z0);
      const float dxy = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));

      // Ignore transitions below the noise floor
      if (dz < static_cast<float>(min_dz_)) {
        continue;
      }

      // Near-vertical face (dxy ≈ 0): slope ratio → ∞, definitely an obstacle
      constexpr float kMinDxy = 1e-3f;
      if (dxy < kMinDxy) {
        continue;
      }

      // Ground slope: dz proportional to dxy → ratio at or below threshold
      // Suppress the upper-ring point from downstream marking
      if ((dz / dxy) <= static_cast<float>(slope_threshold_)) {
        nan_point(row + 1, col);
      }
    }
  }
}

}  // namespace custom_costmap_layers

PLUGINLIB_EXPORT_CLASS(custom_costmap_layers::RingSlopeFilterLayer, nav2_costmap_2d::Layer)