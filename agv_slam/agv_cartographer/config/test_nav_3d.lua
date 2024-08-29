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

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
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
-- -- 3D LiDAR를 사용하여 트랙킹 빌더 설정
-- TRAJECTORY_BUILDER_3D = {
--   submaps = {
--     high_resolution = 0.10,
--     high_resolution_max_range = 20.0,
--     low_resolution = 0.45,
--     num_range_data = 160,
--   },
--   motion_filter = {
--     max_time_seconds = 0.5,
--     max_distance_meters = 0.2,
--     max_angle_radians = 0.04,
--   },
--   imu_gravity_time_constant = 10.0,
--   ceres_scan_matcher = {
--     occupied_space_weight_0 = 10.0,
--     occupied_space_weight_1 = 20.0,
--     translation_weight = 1.0,
--     rotation_weight = 0.1,
--   },
--   use_online_correlative_scan_matching = true,
--   real_time_correlative_scan_matcher = {
--     linear_search_window = 0.1,
--     angular_search_window = math.rad(1.),
--   },
-- }

-- MAP_BUILDER.use_trajectory_builder_3d = true
-- MAP_BUILDER.num_background_threads = 7


TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(0.1)

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 15
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100.
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 15.

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 480
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options