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

#include "custom_costmap_layers/ring_slope_stvl_layer.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace custom_costmap_layers
{

RingSlopeSTVLLayer::RingSlopeSTVLLayer()
: slope_threshold_(1.0),
  min_dz_(0.03)
{
}

void RingSlopeSTVLLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("RingSlopeSTVLLayer: failed to lock node");
  }

  // ── 1. Declare and read our filter-specific params ─────────────────────────
  declareParameter("slope_threshold", rclcpp::ParameterValue(1.0));
  declareParameter("min_dz",          rclcpp::ParameterValue(0.03));
  node->get_parameter(name_ + ".slope_threshold", slope_threshold_);
  node->get_parameter(name_ + ".min_dz",          min_dz_);

  RCLCPP_INFO(
    node->get_logger(),
    "[RingSlopeSTVLLayer] slope_threshold=%.2f  min_dz=%.3f m",
    slope_threshold_, min_dz_);

  // ── 2. Discover which observation sources have marking=true ────────────────
  //    We must declare observation_sources ourselves before calling the parent
  //    so that we can read the YAML value at this point.
  declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
  std::string sources_str;
  node->get_parameter(name_ + ".observation_sources", sources_str);

  std::istringstream iss(sources_str);
  std::vector<std::string> sources{
    std::istream_iterator<std::string>(iss),
    std::istream_iterator<std::string>()};

  auto qos = rclcpp::SensorDataQoS();

  for (const auto & source : sources) {
    // Read whether this source is a marking source
    bool marking = false;
    declareParameter(source + ".marking", rclcpp::ParameterValue(false));
    node->get_parameter(name_ + "." + source + ".marking", marking);
    if (!marking) {
      continue;
    }

    // Read the original topic the user configured
    std::string original_topic;
    declareParameter(source + ".topic", rclcpp::ParameterValue(std::string("")));
    node->get_parameter(name_ + "." + source + ".topic", original_topic);
    if (original_topic.empty()) {
      RCLCPP_WARN(
        node->get_logger(),
        "[RingSlopeSTVLLayer] marking source '%s' has no topic configured, skipping.",
        source.c_str());
      continue;
    }

    // ── 3. Build an internal relay topic name ────────────────────────────────
    //    Append source name to disambiguate when multiple marking sources
    //    point at the same topic.
    const std::string relay_topic = original_topic + "_rsf_" + source;

    RCLCPP_INFO(
      node->get_logger(),
      "[RingSlopeSTVLLayer] intercepting marking source '%s': %s -> %s",
      source.c_str(), original_topic.c_str(), relay_topic.c_str());

    // ── 4. Override the topic param so STVL subscribes to the relay topic ────
    node->set_parameter(
      rclcpp::Parameter(name_ + "." + source + ".topic", relay_topic));

    // ── 5. Create publisher (relay topic) and subscription (original topic) ──
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(relay_topic, qos);
    pubs_.push_back(pub);

    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      original_topic, qos,
      [this, pub](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        cloudCallback(msg, pub);
      });
    subs_.push_back(sub);
  }

  // ── 6. Hand off to STVL ────────────────────────────────────────────────────
  //    STVL reads all the same params. For marking sources it now sees the
  //    relay topic names we set above. Everything else (decay, voxel size,
  //    clearing sources, frustum models) is untouched.
  SpatioTemporalVoxelLayer::onInitialize();
}

void RingSlopeSTVLLayer::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
{
  // Copy so we can mutate in-place without touching the original message
  auto filtered = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
  applyRingSlopeFilter(*filtered);
  pub->publish(std::move(*filtered));
}

void RingSlopeSTVLLayer::applyRingSlopeFilter(sensor_msgs::msg::PointCloud2 & cloud) const
{
  // Requires an organized cloud (height > 1). Unorganized clouds pass through.
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

  const uint32_t W  = cloud.width;
  const uint32_t H  = cloud.height;
  const uint32_t ps = cloud.point_step;
  const uint32_t rs = cloud.row_step;
  uint8_t * data    = cloud.data.data();

  // Read a float field from the raw buffer
  auto read_f = [&](uint32_t row, uint32_t col, int off) -> float {
    float v;
    std::memcpy(&v, data + row * rs + col * ps + off, sizeof(float));
    return v;
  };

  // NaN out a point so downstream PCL/STVL filters discard it
  auto nan_point = [&](uint32_t row, uint32_t col) {
    constexpr float nan = std::numeric_limits<float>::quiet_NaN();
    uint8_t * p = data + row * rs + col * ps;
    std::memcpy(p + x_off, &nan, sizeof(float));
    std::memcpy(p + y_off, &nan, sizeof(float));
    std::memcpy(p + z_off, &nan, sizeof(float));
  };

  // Per-column buffer: finite returns sorted by z for true elevation order.
  // Sorting is independent of how the Ouster driver orders beam rows, fixing
  // the ring-ordering issue.
  struct RingPoint { float x, y, z; uint32_t row; };
  std::vector<RingPoint> col_pts;
  col_pts.reserve(H);

  const float slope_thresh = static_cast<float>(slope_threshold_);
  const float min_dz_f     = static_cast<float>(min_dz_);
  constexpr float kMinDxy  = 1e-3f;  // 1 mm — treat smaller as near-vertical

  // slope_ratio between two ring points.
  // Returns FLT_MAX ("keep as obstacle") when:
  //   - |dz| is below the noise floor, or
  //   - dxy is near-zero (near-vertical face).
  auto slope_ratio = [&](const RingPoint & a, const RingPoint & b) -> float {
    const float dz  = std::abs(b.z - a.z);
    const float dxy = std::sqrt(
      (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
    if (dz < min_dz_f || dxy < kMinDxy) {
      return std::numeric_limits<float>::max();
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
    if (col_pts.size() < 3) {
      continue;
    }

    // Sort ascending by z so adjacent indices are adjacent elevation angles
    std::sort(
      col_pts.begin(), col_pts.end(),
      [](const RingPoint & a, const RingPoint & b) { return a.z < b.z; });

    // For each interior point, require BOTH elevation neighbours to agree
    // it looks like ground before suppressing it.
    //
    // This protects obstacle edges: at a wall/ground boundary one neighbour
    // is on the obstacle (high slope_ratio) and one is on the ground
    // (low slope_ratio). They will not both agree -> point is kept.
    //
    // First and last points in the sorted column are skipped — they have
    // only one neighbour so there is no confirmation vote available.
    // The lowest return per column is almost always ground; leaving it
    // intact is correct and conservative.
    const size_t N = col_pts.size();
    for (size_t i = 1; i < N - 1; ++i) {
      const float r_below = slope_ratio(col_pts[i - 1], col_pts[i]);
      const float r_above = slope_ratio(col_pts[i],     col_pts[i + 1]);

      if (r_below <= slope_thresh && r_above <= slope_thresh) {
        nan_point(col_pts[i].row, col);
      }
    }
  }
}

}  // namespace custom_costmap_layers

PLUGINLIB_EXPORT_CLASS(
  custom_costmap_layers::RingSlopeSTVLLayer,
  nav2_costmap_2d::Layer)