-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  
  publish_tracked_pose=true,
  
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = true,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
MAP_BUILDER.num_background_threads = 7
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 170
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 55.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 100
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 40
-- TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.constraint_builder.min_score = 0.8
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
-- MAP_BUILDER.num_background_threads = 7

-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_3D.min_range = 2.5
-- TRAJECTORY_BUILDER_3D.max_range = 50
-- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 20

-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1.
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.optimization_problem.log_solver_summary = true
-- POSE_GRAPH.optimization_problem.acceleration_weight = 500 --1e3 500
-- POSE_GRAPH.optimization_problem.rotation_weight = 100 --3e5 1000
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 1000
-- POSE_GRAPH.optimization_problem.huber_scale = 5e2
-- POSE_GRAPH.optimize_every_n_nodes = 80
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
-- POSE_GRAPH.global_sampling_ratio = 0.003
-- POSE_GRAPH.constraint_builder.min_score = 0.50
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 20.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 250.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 30.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50

return options
