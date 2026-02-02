#ifndef TERRAIN_GLOBAL_LAYER_HPP_
#define TERRAIN_GLOBAL_LAYER_HPP_

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace nav2_costmap_2d
{

class TerrainGlobalLayer : public CostmapLayer
{
public:
  TerrainGlobalLayer();

  virtual ~TerrainGlobalLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                   int max_i, int max_j) override;

  void reset() override;
  bool isClearable() override;

private:
  void terrainCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // Helper function to safely read a value from the terrain grid
  // Returns -1 if the coordinates are out of bounds
  int8_t getTerrainValue(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid,
                         int x, int y) const;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_terrain_;
  std::string topic_name_;
  bool received_data_ = false;
};

}  // namespace nav2_costmap_2d

#endif  // TERRAIN_GLOBAL_LAYER_HPP_