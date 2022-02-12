// Copyright (c) 2022 Vinny Ruia
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

#include "explored_space_costmap_plugin/explored_space_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

namespace explored_space_costmap_plugin
{

ExploredSpaceLayer::ExploredSpaceLayer()
{
}

void ExploredSpaceLayer::onInitialize()
{
  auto node = node_.lock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("robot_radius", rclcpp::ParameterValue(1));
  node->get_parameter(name_ + "." + "robot_radius", robot_radius_m_);

  declareParameter("cost_when_explored", rclcpp::ParameterValue(233));
  node->get_parameter(name_ + "." + "cost_when_explored", cost_when_explored_);
}

void ExploredSpaceLayer::updateBounds(
  double robot_x, double robot_y, double /**robot_yaw**/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  matchSize();
  *min_x = *min_y = -std::numeric_limits<double>::max();
  *max_x = *max_y = std::numeric_limits<double>::max();

  robot_x_ = robot_x;
  robot_y_ = robot_y;

}

void ExploredSpaceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{

  if (!enabled_) {
    current_ = true;
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  unsigned int map_i, map_j;
  master_grid.worldToMap(robot_x_, robot_y_, map_i, map_j);
  
  unsigned int cell_dist = master_grid.cellDistance(robot_radius_m_);
  min_i = map_i - (2 * cell_dist);
  max_i = map_i - (0.5 * cell_dist);
  min_j = map_j - cell_dist;
  max_j = map_j + (cell_dist);

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(explored_space_costmap_plugin::ExploredSpaceLayer, nav2_costmap_2d::Layer)
