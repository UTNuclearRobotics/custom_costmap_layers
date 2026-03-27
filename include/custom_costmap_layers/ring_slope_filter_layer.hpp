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

#ifndef CUSTOM_COSTMAP_LAYERS__RING_SLOPE_FILTER_LAYER_HPP_
#define CUSTOM_COSTMAP_LAYERS__RING_SLOPE_FILTER_LAYER_HPP_

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace custom_costmap_layers
{

/**
 * @brief A passthrough costmap layer that subscribes to an organized
 *        PointCloud2, suppresses returns that are characteristic of rising
 *        ground rather than true obstacles, and republishes the filtered
 *        cloud on a new topic for STVL (or any other consumer) to mark from.
 *
 * The filter walks adjacent elevation rings at the same azimuth column.
 * For each pair of vertically-adjacent ring points it computes:
 *
 *     slope_ratio = |dz| / dxy
 *
 * A high ratio (near-vertical) means the lidar hit a wall/obstacle.
 * A low ratio (proportional horizontal travel) means it hit sloped ground.
 * Points whose ratio is at or below `slope_threshold` are NaN-ed out so
 * downstream filters discard them.
 *
 * This layer does NOT write to the costmap itself; it only acts as a
 * preprocessing relay. Place it before your STVL layer in the plugin list.
 */
class RingSlopeFilterLayer : public nav2_costmap_2d::Layer
{
public:
  RingSlopeFilterLayer();
  ~RingSlopeFilterLayer() override = default;

  // nav2_costmap_2d::Layer interface
  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  bool isClearable() override { return false; }
  void reset() override {}

private:
  /**
   * @brief Subscriber callback: apply the ring slope filter in-place and
   *        republish on output_topic_.
   */
  void cloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Mutates cloud in-place, NaN-ing points classified as ground slope.
   *        Requires an organized cloud (height > 1).
   */
  void applyRingSlopeFilter(sensor_msgs::msg::PointCloud2 & cloud) const;

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  double slope_threshold_;   // dz/dxy; points at or below this ratio are suppressed
  double min_dz_;            // noise floor in metres; pairs with |dz| < this are skipped

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace custom_costmap_layers

#endif  // CUSTOM_COSTMAP_LAYERS__RING_SLOPE_FILTER_LAYER_HPP_