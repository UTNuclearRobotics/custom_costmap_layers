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

#ifndef CUSTOM_COSTMAP_LAYERS__RING_SLOPE_STVL_LAYER_HPP_
#define CUSTOM_COSTMAP_LAYERS__RING_SLOPE_STVL_LAYER_HPP_

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "spatio_temporal_voxel_layer/spatio_temporal_voxel_layer.hpp"

namespace custom_costmap_layers
{

/**
 * @brief Drop-in replacement for SpatioTemporalVoxelLayer that applies a
 *        ring-based slope filter to each organized PointCloud2 before it
 *        reaches STVL's measurement buffers.
 *
 * How it works
 * ------------
 * During onInitialize() this class:
 *   1. Reads the same observation_sources param that STVL would read.
 *   2. For every marking source, saves the configured topic and redirects
 *      the param to an internal relay topic (<original>_rsf_<source>).
 *   3. Subscribes to the original topic, applies the ring slope filter
 *      in-place on a copy, and publishes to the relay topic.
 *   4. Calls SpatioTemporalVoxelLayer::onInitialize(), which picks up the
 *      redirected topic params and sets up all voxel machinery as normal.
 *
 * The relay topics are an implementation detail — users configure this
 * layer exactly like a normal STVL layer and add two extra params:
 *   slope_threshold  (dz/dxy, default 1.0 = 45 deg)
 *   min_dz           (metres, default 0.03 = 3 cm noise floor)
 *
 * Ring slope filter logic
 * -----------------------
 * For each azimuth column in the organized cloud:
 *   - Collect all finite returns and sort by z (true elevation order,
 *     independent of how the driver orders beam rows).
 *   - For each interior point, compute slope_ratio to both its lower and
 *     upper elevation neighbours.
 *   - Only suppress the point if BOTH neighbours independently agree the
 *     transition looks like sloped ground (ratio <= slope_threshold).
 *     This protects obstacle edges where one neighbour is on the obstacle
 *     and one is on the ground — they will not both agree.
 *   - Points below min_dz or with near-zero dxy are left intact.
 */
class RingSlopeSTVLLayer : public spatio_temporal_voxel_layer::SpatioTemporalVoxelLayer
{
public:
  RingSlopeSTVLLayer();
  ~RingSlopeSTVLLayer() override = default;

  void onInitialize() override;

private:
  /**
   * @brief Subscription callback: filter and republish.
   * @param msg   Original organized cloud from the sensor.
   * @param pub   Publisher to the relay topic STVL is subscribed to.
   */
  void cloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);

  /**
   * @brief Mutates cloud in-place, NaN-ing points classified as ground slope.
   *        Requires an organized cloud (height > 1). Unorganized clouds are
   *        passed through unchanged.
   */
  void applyRingSlopeFilter(sensor_msgs::msg::PointCloud2 & cloud) const;

  // Filter parameters
  double slope_threshold_;  // dz/dxy <= this is treated as ground and suppressed
  double min_dz_;           // |dz| below this is ignored (noise floor, metres)

  // One subscription + publisher pair per intercepted marking source
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pubs_;
};

}  // namespace custom_costmap_layers

#endif  // CUSTOM_COSTMAP_LAYERS__RING_SLOPE_STVL_LAYER_HPP_