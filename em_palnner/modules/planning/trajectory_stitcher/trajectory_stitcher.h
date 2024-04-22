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
 * @file trajectory_stitcher.h
 **/

#ifndef MODULES_PLANNING_TRAJECTORY_STITCHER_TRAJECTORY_STITCHER_H_
#define MODULES_PLANNING_TRAJECTORY_STITCHER_TRAJECTORY_STITCHER_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

class TrajectoryStitcher {
 public:
  TrajectoryStitcher() = delete;

  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
      const bool is_on_auto_mode, const double current_timestamp,
      const double planning_cycle_time,
      const PublishableTrajectory& prev_trajectory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_TRAJECTORY_STITCHER_TRAJECTORY_STITCHER_H_
