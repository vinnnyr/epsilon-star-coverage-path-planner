cmake_minimum_required(VERSION 3.8)
project(epsilon_star_coverage_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_core SHARED
  src/multiscale_tiling.cpp
)

set(dependencies
  rclcpp
  nav2_costmap_2d
)

ament_target_dependencies(${PROJECT_NAME}_core
  ${dependencies}
)

install(TARGETS
  ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(${dependencies})
ament_package()
