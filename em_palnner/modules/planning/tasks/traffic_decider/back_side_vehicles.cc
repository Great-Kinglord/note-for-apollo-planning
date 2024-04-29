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
 * @file
 **/

#include "modules/planning/tasks/traffic_decider/back_side_vehicles.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {
/// @brief 后面新的apollo代码会增加更多的交通规则
BackSideVehicles::BackSideVehicles() : TrafficRule("BackSideVehicles") {}

bool BackSideVehicles::ApplyRule(ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  ObjectDecisionType ignore;
  ignore.mutable_ignore();
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    if (path_obstacle->perception_sl_boundary().end_s() >=
        adc_sl_boundary.end_s()) {///< 忽略前方障碍物,主要是太远了
      continue;
    }

    if (path_obstacle->st_boundary().IsEmpty()) {///< 参考线上没有障碍物轨迹，则忽略，应该是静止物体
      path_decision->AddLongitudinalDecision(Name(), path_obstacle->Id(),
                                             ignore);
      path_decision->AddLateralDecision(Name(), path_obstacle->Id(), ignore);
      continue;
    }
    if (path_obstacle->st_boundary().min_s() < 0) {///< 障碍物轨迹距离无人车的最近距离小于0,则忽略
      path_decision->AddLongitudinalDecision(Name(), path_obstacle->Id(),
                                             ignore);
      path_decision->AddLateralDecision(Name(), path_obstacle->Id(), ignore);
      continue;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
