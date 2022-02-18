// Copyright (c) 2022, Vinny Ruia
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
// limitations under the License. Reserved.

#include <cmath>
#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>

#include <epsilon_star_coverage_path_planner/multiscale_tiling.hpp>

namespace epsilon_star_coverage_path_planner
{

MultiscaleTiler::MultiscaleTiler(const nav2_costmap_2d::Costmap2D input_costmap, const float eta_resolution_m)
{
    input_costmap_ = input_costmap;
    eta_resolution_m_ = eta_resolution_m;
}

MultiscaleTiler::~MultiscaleTiler()
{
}

std::vector<nav2_costmap_2d::Costmap2D> MultiscaleTiler::get_tiles()
{
  std::vector<nav2_costmap_2d::Costmap2D> tiles = {input_costmap_};
  return subdivide(tiles, eta_resolution_m_);
}

std::vector<nav2_costmap_2d::Costmap2D> MultiscaleTiler::subdivide(std::vector<nav2_costmap_2d::Costmap2D> tiles, const float resolution_m)
{
  // let n be the max number of eta cells along the costmap width
  // TODO(vinnnyr): the below is a simplification we can improve on later
  // force n to be an even number, at risk of not exactly meeting the requested eta resolution
  auto costmap = tiles[-1];  // is this copy constructor?
  int n_2 = std::nearbyint((costmap.getSizeInCellsX() / resolution_m) * 0.5f);
  float resolution_2 = costmap.getResolution() * 2.0f;
  if (n_2 <= 2) {
    return tiles;
  }

  // resize
  costmap.resizeMap(n_2, n_2, resolution_2, costmap.getOriginX(),
                           costmap.getOriginY());
  tiles.push_back(costmap);
  return subdivide(tiles, resolution_2);
}

}  // namespace epsilon_star_coverage_path_planner