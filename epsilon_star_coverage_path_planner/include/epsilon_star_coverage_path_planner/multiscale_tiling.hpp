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


#ifndef EPSILON_STAR_COVERAGE_PATH_PLANNER__MULTISCALE_TILING_HPP_
#define EPSILON_STAR_COVERAGE_PATH_PLANNER__MULTISCALE_TILING_HPP_

#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace epsilon_star_coverage_path_planner
{

class MultiscaleTiler
{
public:
  MultiscaleTiler(const nav2_costmap_2d::Costmap2D input_costmap, const float eta_resolution_m);
  ~MultiscaleTiler();
  std::vector<nav2_costmap_2d::Costmap2D> get_tiles();
private:
  std::vector<nav2_costmap_2d::Costmap2D> subdivide(std::vector<nav2_costmap_2d::Costmap2D> tiles, const float resolution_m);
  float eta_resolution_m_;
  nav2_costmap_2d::Costmap2D input_costmap_;
};

}  // namespace epsilon_star_coverage_path_planner

#endif  // EPSILON_STAR_COVERAGE_PATH_PLANNER__MULTISCALE_TILING_HPP_
