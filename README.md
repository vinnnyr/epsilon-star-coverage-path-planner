# epsilon-star-coverage-path-planner
Epsilon Star Coverage Path Planner implementation for Nav2 / Ros2, based on : https://linkslab.uconn.edu/wp-content/uploads/sites/246/2018/11/e_star_Preprint.pdf

## running
`colcon build --packages-up-to explored_space_costmap_plugin && source install/setup.bash && ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/home/vinny/Code/nav2_ws/src/epsilon-star-coverage-path-planner/nav2_params_epsilon_star_coverage_planner.yaml  | grep -v -F 'rviz2'`
