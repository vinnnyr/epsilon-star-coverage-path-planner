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

#ifndef EXPLORED_SPACE_LAYER_HPP__
#define EXPLORED_SPACE_LAYER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace explored_space_costmap_plugin
{

class ExploredSpaceLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ExploredSpaceLayer();
  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double /**robot_yaw**/, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  // TODO(vinnnyr): make clearable
  virtual bool isClearable() {return false;}

private:
  // TODO(vinnnyr): should be able to get footprint dims from system
  int robot_radius_m_;
  int cost_when_explored_;
  double robot_x_;
  double robot_y_;
};
}  // namespace explored_space_costmap_plugin

#endif  // EXPLORED_SPACE_LAYER_HPP__
