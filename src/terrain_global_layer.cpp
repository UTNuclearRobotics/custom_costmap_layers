#include "custom_costmap_layers/terrain_global_layer.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <memory>

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::TerrainGlobalLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

TerrainGlobalLayer::TerrainGlobalLayer()
  : received_data_(false)
{
}

void TerrainGlobalLayer::onInitialize()
{
  // Correct way to declare and get a parameter in a costmap layer
  auto node = node_.lock();
  if (!node)
  {
    RCLCPP_ERROR(logger_, "Failed to get node handle");
    return;
  }

  // Declare parameter if not already declared
  node->declare_parameter<std::string>("terrain_topic", "/panther/sterling/global_costmap");

  // Get the value
  topic_name_ = node->get_parameter("terrain_topic").as_string();

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_name_,
    rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 10))
      .transient_local()
      .reliable(),
    std::bind(&TerrainGlobalLayer::terrainCallback, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "TerrainGlobalLayer subscribed to: %s", topic_name_.c_str());

  current_ = true;
}

void TerrainGlobalLayer::terrainCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_terrain_ = msg;
  received_data_ = true;
  current_ = false;  // force update
}

void TerrainGlobalLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!received_data_ || !latest_terrain_)
    return;

  // Simple: mark whole area as changed (you can make this more precise later)
  *min_x = std::min(*min_x, latest_terrain_->info.origin.position.x);
  *min_y = std::min(*min_y, latest_terrain_->info.origin.position.y);
  *max_x = std::max(*max_x,
                    latest_terrain_->info.origin.position.x +
                    latest_terrain_->info.width * latest_terrain_->info.resolution);
  *max_y = std::max(*max_y,
                    latest_terrain_->info.origin.position.y +
                    latest_terrain_->info.height * latest_terrain_->info.resolution);
}

void TerrainGlobalLayer::updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!received_data_ || !latest_terrain_)
    return;

  auto terrain = latest_terrain_;
  if (!terrain->data.size())
    return;

  double t_res = terrain->info.resolution;
  double g_res = master_grid.getResolution();

  double t_origin_x = terrain->info.origin.position.x;
  double t_origin_y = terrain->info.origin.position.y;

  // Iterate over the **global** cells in the update window
  for (int gy = min_j; gy <= max_j; ++gy)
  {
    for (int gx = min_i; gx <= max_i; ++gx)
    {
      // Get center of global cell in world coords
      double wx, wy;
      master_grid.mapToWorld(gx, gy, wx, wy);

      // Corresponding position in terrain grid (fractional)
      double tx = (wx - t_origin_x) / t_res;
      double ty = (wy - t_origin_y) / t_res;

      // Skip if clearly outside terrain grid
      if (tx < -0.5 || ty < -0.5 || tx >= terrain->info.width - 0.5 || ty >= terrain->info.height - 0.5)
        continue;

      // Bilinear interpolation (simple version)
      int tx0 = std::floor(tx);
      int ty0 = std::floor(ty);
      double dx = tx - tx0;
      double dy = ty - ty0;

      int8_t v00 = getTerrainValue(terrain, tx0,   ty0);
      int8_t v10 = getTerrainValue(terrain, tx0+1, ty0);
      int8_t v01 = getTerrainValue(terrain, tx0,   ty0+1);
      int8_t v11 = getTerrainValue(terrain, tx0+1, ty0+1);

      float interp = (1-dy)*(1-dx)*v00 + dy*(1-dx)*v01 + (1-dy)*dx*v10 + dy*dx*v11;

      if (interp < -0.5f) continue;  // treat as unknown

      unsigned char new_cost = static_cast<unsigned char>(std::round(interp));
      unsigned char current = master_grid.getCost(gx, gy);

      // Decide update policy — here we overwrite (most common for terrain)
      // You can also do max(current, new_cost) if you want conservative merging
      master_grid.setCost(gx, gy, new_cost);
    }
  }

  current_ = true;
}

// Helper to safely get value (clamp to grid)
int8_t TerrainGlobalLayer::getTerrainValue(const nav_msgs::msg::OccupancyGrid::SharedPtr& grid,
                                           int x, int y) const
{
  if (!grid)
  {
    return -1;
  }

  if (x < 0 || y < 0 ||
      x >= static_cast<int>(grid->info.width) ||
      y >= static_cast<int>(grid->info.height))
  {
    return -1;
  }

  return grid->data[y * grid->info.width + x];
}

void TerrainGlobalLayer::reset()
{
  // Required override — what to do when costmap is reset
  latest_terrain_.reset();
  received_data_ = false;
  current_ = true;
}

bool TerrainGlobalLayer::isClearable()
{
  // Most layers return true — means this layer can be cleared on reset
  return true;
}

}  // namespace nav2_costmap_2d