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

#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

#include <algorithm>
#include <cmath>

#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::TrajectoryPoint;

/// @brief 
/// @param config 
/// @param reference_line 
/// @param obstacles 
/// @param vehicle_param 
/// @param heuristic_speed_data 
TrajectoryCost::TrajectoryCost(
    const DpPolyPathConfig &config, const ReferenceLine &reference_line,
    const std::vector<const PathObstacle *> &obstacles,
    const common::VehicleParam &vehicle_param,
    const SpeedData &heuristic_speed_data)
    : config_(config),
      reference_line_(&reference_line),
      vehicle_param_(vehicle_param),
      heuristic_speed_data_(heuristic_speed_data) {
  const double total_time =
      std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);///< prediction_total_time = 5s

  num_of_time_stamps_ = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));///< eval_time_interval = 0.1s

  for (const auto ptr_path_obstacle : obstacles) {
    if (ptr_path_obstacle->IsIgnore()) {
      continue;
    }
    const auto ptr_obstacle = ptr_path_obstacle->obstacle();
    if (ptr_obstacle->PerceptionId() < 0) {
      // Virtual obsatcle
      continue;
    }
    std::vector<Box2d> box_by_time;
    for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
      TrajectoryPoint trajectory_point =
          ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());
      Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);///<得到障碍物的box
      box_by_time.push_back(obstacle_box);
    }
    obstacle_boxes_.push_back(box_by_time); ///<把障碍物的box信息存储到obstacle_boxes_中,最外是按照障碍物，里面一层是按照时间
  }
}
/// @brief 五次多项式的代价计算
/// @param curve 五次多项式曲线
/// @param start_s 两个点，第一点的s
/// @param end_s 两个点，第二个点的s
/// @return 
double TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                 const double start_s,
                                 const double end_s) const {
  double total_cost = 0.0;
  ///路径的cost
  double path_s = 0.0;
  while (path_s < (end_s - start_s)) {
    const double l = std::fabs(curve.Evaluate(0, path_s)); ///< 得l
    total_cost += l;
    const double dl = std::fabs(curve.Evaluate(1, path_s));///< 得dl
    total_cost += dl;
    path_s += config_.path_resolution(); ///< 默认为0.1m
  }
  // Obstacle cost 障碍物cost
  uint32_t start_index = 0;
  bool is_found_start_index = false;
  uint32_t end_index = 0;
  uint32_t index = 0;
  double time_stamp = 0.0;
  while (end_index != 0) {
    common::SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);
    ///大于上层点的s
    if (speed_point.s() >= start_s && !is_found_start_index) {
      start_index = index;
      is_found_start_index = true;
    }
    ///大于当前层点的s
    if (speed_point.s() > end_s) {
      end_index = index;
    }
    time_stamp += config_.eval_time_interval(); ///<0.1s，这应该是为了找障碍物容器
    ++index;
  }

  for (; start_index < end_index; ++start_index) {
    common::SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(
        start_index * config_.eval_time_interval(), &speed_point); ///< start_index * 0.1
    const double s = speed_point.s() - start_s;
    const double l = curve.Evaluate(0, s);
    const double dl = curve.Evaluate(1, s);
    Vec2d ego_xy_point;
    common::SLPoint sl;
    sl.set_s(speed_point.s());
    sl.set_l(l);
    reference_line_->SLToXY(sl, &ego_xy_point);
    ReferencePoint reference_point =
        reference_line_->GetReferencePoint(speed_point.s());

    double one_minus_kappa_r_d = 1 - reference_point.kappa() * l;
    double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    double theta =
        common::math::NormalizeAngle(delta_theta + reference_point.heading());
    Box2d ego_box = {ego_xy_point, theta, vehicle_param_.length(),
                     vehicle_param_.width()}; ///<车辆的box
    ///<遍历障碍物
    for (const auto &obstacle_trajectory : obstacle_boxes_) {
      auto &obstacle_box = obstacle_trajectory[start_index];///< 障碍物对应时间的信息
      // Simple version: calculate obstacle cost by distance 通过距离来计算障碍物的cost
      double distance = obstacle_box.DistanceTo(ego_box);///<计算障碍物和车辆的距,两个box之间的最近距离
      if (distance > config_.obstacle_ignore_distance()) {
        ///距离大于20m
        continue;
      } else if (distance <= config_.obstacle_collision_distance()) {
        ///距离小于0.2m
        total_cost += config_.obstacle_collision_cost(); ///<  默认为1e3
      } else if (distance <= config_.obstacle_risk_distance()) {
        /// 距离下雨2.0m
        total_cost += RiskDistanceCost(distance);
      } else {
        ///距离在2m到20m之间
        total_cost += RegularDistanceCost(distance);
      }
    }
  }
  return total_cost;
}

double TrajectoryCost::RiskDistanceCost(const double distance) const {
  ///(5.0 - distance) * ((5.0 - distance)) * 10
  return (5.0 - distance) * ((5.0 - distance)) * 10;
}

double TrajectoryCost::RegularDistanceCost(const double distance) const {
  return std::max(20.0 - distance, 0.0);
}

}  // namespace planning
}  // namespace apollo
