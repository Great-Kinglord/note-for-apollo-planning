/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file trajectory_stitcher.cc
 **/

#include "modules/planning/trajectory_stitcher/trajectory_stitcher.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;

namespace {

std::vector<TrajectoryPoint> ComputeReinitStitchingTrajectory() {
  TrajectoryPoint init_point;
  const auto& vehicle_state = *VehicleState::instance();
  init_point.mutable_path_point()->set_x(vehicle_state.x());
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());

  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  DCHECK(!std::isnan(init_point.path_point().x()));
  DCHECK(!std::isnan(init_point.path_point().y()));
  DCHECK(!std::isnan(init_point.path_point().z()));
  DCHECK(!std::isnan(init_point.path_point().theta()));
  DCHECK(!std::isnan(init_point.path_point().kappa()));
  DCHECK(!std::isnan(init_point.v()));
  DCHECK(!std::isnan(init_point.a()));

  return std::vector<TrajectoryPoint>(1, init_point);
}
}  // namespace

// Planning from current vehicle state:
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const bool is_on_auto_mode, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory& prev_trajectory) {
  if (!is_on_auto_mode) {
    return ComputeReinitStitchingTrajectory();
  }

  std::size_t prev_trajectory_size = prev_trajectory.NumOfPoints();

  if (prev_trajectory_size == 0) {
    AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
          << "] size is zero! Previous planning not exist or failed. Use "
             "origin car status instead.";
    return ComputeReinitStitchingTrajectory();
  }

  const double veh_rel_time = current_timestamp - prev_trajectory.header_time();

  std::size_t matched_index = prev_trajectory.QueryNearestPoint(veh_rel_time);

  if (matched_index == prev_trajectory_size) {
    AWARN << "The previous trajectory is not long enough, something is wrong";
    return ComputeReinitStitchingTrajectory();
  }

  if (matched_index == 0 &&
      veh_rel_time < prev_trajectory.StartPoint().relative_time()) {
    AWARN << "the previous trajectory doesn't cover current time";
    return ComputeReinitStitchingTrajectory();
  }

  const auto& vehicle_state = *VehicleState::instance();
  auto matched_point = prev_trajectory.TrajectoryPointAt(matched_index);
  const double position_diff =
      std::hypot(matched_point.path_point().x() - vehicle_state.x(),
                 matched_point.path_point().y() - vehicle_state.y());

  if (position_diff > FLAGS_replan_distance_threshold) {
    AWARN << "the distance between matched point and actual position is too "
             "large";
    return ComputeReinitStitchingTrajectory();
  }

  double forward_rel_time = veh_rel_time + planning_cycle_time;
  std::size_t forward_index =
      prev_trajectory.QueryNearestPoint(forward_rel_time);

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory.trajectory_points().begin() + matched_index,
      prev_trajectory.trajectory_points().begin() + forward_index + 1);

  const double zero_time = veh_rel_time;
  const double zero_s =
      prev_trajectory.TrajectoryPointAt(forward_index).path_point().s();

  for (auto& tp : stitching_trajectory) {
    tp.set_relative_time(tp.relative_time() - zero_time);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

}  // namespace planning
}  // namespace apollo
