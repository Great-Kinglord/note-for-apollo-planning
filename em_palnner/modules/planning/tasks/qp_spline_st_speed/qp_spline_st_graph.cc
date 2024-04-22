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
 * @file qp_spline_st_graph.cc
 **/

#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleParam;
using apollo::planning_internal::STGraphDebug;

QpSplineStGraph::QpSplineStGraph(
    const QpSplineStSpeedConfig& qp_spline_st_speed_config,
    const VehicleParam& veh_param)
    : qp_spline_st_speed_config_(qp_spline_st_speed_config),
      t_knots_resolution_(
          qp_spline_st_speed_config_.total_time() /
          qp_spline_st_speed_config_.number_of_discrete_graph_t()),
      t_evaluated_resolution_(
          qp_spline_st_speed_config_.total_time() /
          qp_spline_st_speed_config_.number_of_evaluated_graph_t()) {
  Init();
}

void QpSplineStGraph::Init() {
  // init knots
  double curr_t = 0.0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_discrete_graph_t(); ++i) {
    t_knots_.push_back(curr_t);
    curr_t += t_knots_resolution_;
  }

  // init evaluated t positions
  curr_t = 0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_evaluated_graph_t(); ++i) {
    t_evaluated_.push_back(curr_t);
    curr_t += t_evaluated_resolution_;
  }
}

void QpSplineStGraph::SetDebugLogger(
    planning_internal::STGraphDebug* st_graph_debug) {
  if (st_graph_debug) {
    st_graph_debug->Clear();
    st_graph_debug_ = st_graph_debug;
  }
}

Status QpSplineStGraph::Search(const StGraphData& st_graph_data,
                               SpeedData* const speed_data,
                               const std::pair<double, double>& accel_bound) {
  cruise_.clear();

  // reset spline generator
  spline_generator_.reset(new Spline1dGenerator(
      t_knots_, qp_spline_st_speed_config_.spline_order()));

  // start to search for best st points
  init_point_ = st_graph_data.init_point();
  if (st_graph_data.path_data_length() <
      qp_spline_st_speed_config_.total_path_length()) {
    qp_spline_st_speed_config_.set_total_path_length(
        st_graph_data.path_data_length());
  }

  if (!ApplyConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
                       st_graph_data.st_boundaries(), accel_bound)
           .ok()) {
    const std::string msg = "Apply constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!ApplyKernel(st_graph_data.st_boundaries(), st_graph_data.speed_limit())
           .ok()) {
    const std::string msg = "Apply kernel failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!Solve().ok()) {
    const std::string msg = "Solve qp problem failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // extract output
  speed_data->Clear();
  const Spline1d& spline = spline_generator_->spline();

  double t_output_resolution =
      qp_spline_st_speed_config_.output_time_resolution();
  double time = 0.0;
  while (time < qp_spline_st_speed_config_.total_time() + t_output_resolution) {
    double s = spline(time);
    double v = spline.Derivative(time);
    double a = spline.SecondOrderDerivative(time);
    double da = spline.ThirdOrderDerivative(time);
    speed_data->AppendSpeedPoint(s, time, v, a, da);
    time += t_output_resolution;
  }

  return Status::OK();
}

Status QpSplineStGraph::ApplyConstraint(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    const std::vector<StBoundary>& boundaries,
    const std::pair<double, double>& accel_bound) {
  Spline1dConstraint* constraint =
      spline_generator_->mutable_spline_constraint();
  // position, velocity, acceleration

  if (!constraint->AddPointConstraint(0.0, 0.0)) {
    const std::string msg = "add st start point constraint failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ADEBUG << "init point constraint:" << init_point.DebugString();
  if (!constraint->AddPointDerivativeConstraint(0.0, init_point_.v())) {
    const std::string msg = "add st start point velocity constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->AddPointSecondDerivativeConstraint(
          spline_generator_->spline().x_knots().back(), 0.0)) {
    const std::string msg = "add st end point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // monotone constraint
  if (!constraint->AddMonotoneInequalityConstraintAtKnots()) {
    const std::string msg = "add monotone inequality constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // smoothness constraint
  if (!constraint->AddSecondDerivativeSmoothConstraint()) {
    const std::string msg = "add smoothness joint constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // boundary constraint
  std::vector<double> s_upper_bound;
  std::vector<double> s_lower_bound;

  for (const double curr_t : t_evaluated_) {
    double lower_s = 0.0;
    double upper_s = 0.0;
    GetSConstraintByTime(boundaries, curr_t,
                         qp_spline_st_speed_config_.total_path_length(),
                         &upper_s, &lower_s);
    s_upper_bound.push_back(upper_s);
    s_lower_bound.push_back(lower_s);
    ADEBUG << "Add constraint by time: " << curr_t << " upper_s: " << upper_s
           << " lower_s: " << lower_s;
  }

  DCHECK_EQ(t_evaluated_.size(), s_lower_bound.size());
  DCHECK_EQ(t_evaluated_.size(), s_upper_bound.size());
  if (!constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound)) {
    const std::string msg = "Fail to apply distance constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // speed constraint
  std::vector<double> speed_upper_bound;
  if (!EstimateSpeedUpperBound(init_point, speed_limit, &speed_upper_bound)
           .ok()) {
    std::string msg = "Fail to estimate speed upper constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);

  DCHECK_EQ(t_evaluated_.size(), speed_upper_bound.size());
  DCHECK_EQ(t_evaluated_.size(), speed_lower_bound.size());

  if (st_graph_debug_) {
    for (size_t i = 0; i < t_evaluated_.size(); ++i) {
      auto speed_constraint =
          st_graph_debug_->mutable_speed_constraint()->Add();
      speed_constraint->add_t(t_evaluated_[i]);
      speed_constraint->add_lower_bound(speed_lower_bound[i]);
      speed_constraint->add_upper_bound(speed_upper_bound[i]);
    }
  }

  if (!constraint->AddDerivativeBoundary(t_evaluated_, speed_lower_bound,
                                         speed_upper_bound)) {
    const std::string msg = "Fail to apply speed constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    ADEBUG << "t_evaluated_: " << t_evaluated_[i]
           << "; speed_lower_bound: " << speed_lower_bound[i]
           << "; speed_upper_bound: " << speed_upper_bound[i];
  }

  // acceleration constraint
  std::vector<double> accel_lower_bound(t_evaluated_.size(), accel_bound.first);
  std::vector<double> accel_upper_bound(t_evaluated_.size(),
                                        accel_bound.second);

  bool has_follow = false;
  double delta_s = 1.0;
  for (const auto& boundary : boundaries) {
    if (boundary.boundary_type() == StBoundary::BoundaryType::FOLLOW) {
      has_follow = true;
      delta_s = std::fmin(
          delta_s, boundary.min_s() - fabs(boundary.characteristic_length()));
    }
  }
  if (FLAGS_enable_follow_accel_constraint && has_follow && delta_s < 0.0) {
    accel_upper_bound.front() = 0.0;
  } else {
    constexpr double kInitPointAccelRelaxedSpeed = 1.0;

    if (init_point_.v() > kInitPointAccelRelaxedSpeed) {
      constexpr double kInitPointAccelRelaxedRange = 0.25;
      accel_lower_bound.front() = init_point_.a() - kInitPointAccelRelaxedRange;
      accel_upper_bound.front() = init_point_.a() + kInitPointAccelRelaxedRange;
    }
  }

  DCHECK_EQ(t_evaluated_.size(), accel_lower_bound.size());
  DCHECK_EQ(t_evaluated_.size(), accel_upper_bound.size());
  if (!constraint->AddSecondDerivativeBoundary(t_evaluated_, accel_lower_bound,
                                               accel_upper_bound)) {
    const std::string msg = "Fail to apply acceleration constraints.";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    ADEBUG << "t_evaluated_: " << t_evaluated_[i]
           << "; accel_lower_bound: " << accel_lower_bound[i]
           << "; accel_upper_bound: " << accel_upper_bound[i];
  }

  return Status::OK();
}

Status QpSplineStGraph::ApplyKernel(const std::vector<StBoundary>& boundaries,
                                    const SpeedLimit& speed_limit) {
  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();

  if (qp_spline_st_speed_config_.accel_kernel_weight() > 0) {
    spline_kernel->AddSecondOrderDerivativeMatrix(
        qp_spline_st_speed_config_.accel_kernel_weight());
  }

  if (qp_spline_st_speed_config_.jerk_kernel_weight() > 0) {
    spline_kernel->AddThirdOrderDerivativeMatrix(
        qp_spline_st_speed_config_.jerk_kernel_weight());
  }

  if (!AddCruiseReferenceLineKernel(speed_limit,
                                    qp_spline_st_speed_config_.cruise_weight())
           .ok()) {
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::ApplyKernel");
  }

  if (!AddFollowReferenceLineKernel(boundaries,
                                    qp_spline_st_speed_config_.follow_weight())
           .ok()) {
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::ApplyKernel");
  }
  return Status::OK();
}

Status QpSplineStGraph::Solve() {
  return spline_generator_->Solve()
             ? Status::OK()
             : Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::solve");
}

Status QpSplineStGraph::AddCruiseReferenceLineKernel(
    const SpeedLimit& speed_limit, const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  if (speed_limit.speed_limit_points().size() == 0) {
    std::string msg = "Fail to apply_kernel due to empty speed limits.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  double dist_ref = 0.0;
  cruise_.push_back(dist_ref);
  for (uint32_t i = 1; i < t_evaluated_.size(); ++i) {
    dist_ref += (t_evaluated_[i] - t_evaluated_[i - 1]) *
                speed_limit.GetSpeedLimitByS(dist_ref);
    cruise_.push_back(dist_ref);
  }
  if (st_graph_debug_) {
    auto kernel_cruise_ref = st_graph_debug_->mutable_kernel_cruise_ref();
    kernel_cruise_ref->mutable_t()->Add(t_evaluated_[0]);
    kernel_cruise_ref->mutable_cruise_line_s()->Add(dist_ref);
    for (uint32_t i = 1; i < t_evaluated_.size(); ++i) {
      kernel_cruise_ref->mutable_t()->Add(t_evaluated_[i]);
      kernel_cruise_ref->mutable_cruise_line_s()->Add(cruise_[i]);
    }
  }
  DCHECK_EQ(t_evaluated_.size(), cruise_.size());

  for (std::size_t i = 0; i < t_evaluated_.size(); ++i) {
    ADEBUG << "Cruise Ref S: " << cruise_[i]
           << " Relative time: " << t_evaluated_[i] << std::endl;
  }

  if (t_evaluated_.size() > 0) {
    spline_kernel->AddReferenceLineKernelMatrix(
        t_evaluated_, cruise_,
        weight * qp_spline_st_speed_config_.total_time() / t_evaluated_.size());
  }
  spline_kernel->AddRegularization(0.01);
  return Status::OK();
}

Status QpSplineStGraph::AddFollowReferenceLineKernel(
    const std::vector<StBoundary>& boundaries, const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  std::vector<double> ref_s;
  std::vector<double> filtered_evaluate_t;
  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    const double curr_t = t_evaluated_[i];
    double s_min = std::numeric_limits<double>::infinity();
    bool success = false;
    for (const auto& boundary : boundaries) {
      if (boundary.boundary_type() != StBoundary::BoundaryType::FOLLOW) {
        continue;
      }
      if (curr_t < boundary.min_t() || curr_t > boundary.max_t()) {
        continue;
      }
      double s_upper = 0.0;
      double s_lower = 0.0;
      if (boundary.GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        success = true;
        s_min = std::min(s_min,
                         s_upper - boundary.characteristic_length() -
                             qp_spline_st_speed_config_.follow_drag_distance());
      }
    }
    if (success && s_min < cruise_[i]) {
      filtered_evaluate_t.push_back(curr_t);
      ref_s.push_back(s_min);
      if (st_graph_debug_) {
        auto kernel_follow_ref = st_graph_debug_->mutable_kernel_follow_ref();
        kernel_follow_ref->mutable_t()->Add(curr_t);
        kernel_follow_ref->mutable_follow_line_s()->Add(s_min);
      }
    }
  }
  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    spline_kernel->AddReferenceLineKernelMatrix(
        filtered_evaluate_t, ref_s,
        weight * qp_spline_st_speed_config_.total_time() / t_evaluated_.size());
  }

  for (std::size_t i = 0; i < filtered_evaluate_t.size(); ++i) {
    ADEBUG << "Follow Ref S: " << ref_s[i]
           << " Relative time: " << filtered_evaluate_t[i] << std::endl;
  }
  return Status::OK();
}

Status QpSplineStGraph::GetSConstraintByTime(
    const std::vector<StBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  *s_upper_bound = total_path_s;

  for (const StBoundary& boundary : boundaries) {
    double s_upper = 0.0;
    double s_lower = 0.0;

    if (!boundary.GetUnblockSRange(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary.boundary_type() == StBoundary::BoundaryType::STOP ||
        boundary.boundary_type() == StBoundary::BoundaryType::FOLLOW ||
        boundary.boundary_type() == StBoundary::BoundaryType::YIELD) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else {
      DCHECK(boundary.boundary_type() == StBoundary::BoundaryType::OVERTAKE);
      *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
    }
  }

  return Status::OK();
}

Status QpSplineStGraph::EstimateSpeedUpperBound(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    std::vector<double>* speed_upper_bound) const {
  DCHECK_NOTNULL(speed_upper_bound);

  speed_upper_bound->clear();

  // use v to estimate position: not accurate, but feasible in cyclic
  // processing. We can do the following process multiple times and use
  // previous cycle's results for better estimation.
  const double v = init_point.v();

  if (static_cast<double>(t_evaluated_.size() +
                          speed_limit.speed_limit_points().size()) <
      t_evaluated_.size() * std::log(static_cast<double>(
                                speed_limit.speed_limit_points().size()))) {
    uint32_t i = 0;
    uint32_t j = 0;
    const double kDistanceEpsilon = 1e-6;
    while (i < t_evaluated_.size() &&
           j + 1 < speed_limit.speed_limit_points().size()) {
      const double distance = v * t_evaluated_[i];
      if (fabs(distance - speed_limit.speed_limit_points()[j].first) <
          kDistanceEpsilon) {
        speed_upper_bound->push_back(
            speed_limit.speed_limit_points()[j].second);
        ++i;
        ADEBUG << "speed upper bound:" << speed_upper_bound->back();
      } else if (distance < speed_limit.speed_limit_points()[j].first) {
        ++i;
      } else if (distance <= speed_limit.speed_limit_points()[j + 1].first) {
        speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
        ADEBUG << "speed upper bound:" << speed_upper_bound->back();
        ++i;
      } else {
        ++j;
      }
    }

    for (uint32_t k = speed_upper_bound->size(); k < t_evaluated_.size(); ++k) {
      speed_upper_bound->push_back(qp_spline_st_speed_config_.max_speed());
      ADEBUG << "speed upper bound:" << speed_upper_bound->back();
    }
  } else {
    auto cmp = [](const std::pair<double, double>& p1, const double s) {
      return p1.first < s;
    };

    const auto& speed_limit_points = speed_limit.speed_limit_points();
    for (const double t : t_evaluated_) {
      const double s = v * t;

      // NOTICE: we are using binary search here based on two assumptions:
      // (1) The s in speed_limit_points increase monotonically.
      // (2) The evaluated_t_.size() << number of speed_limit_points.size()
      //
      // If either of the two assumption is failed, a new algorithm must be
      // used
      // to replace the binary search.

      const auto& it = std::lower_bound(speed_limit_points.begin(),
                                        speed_limit_points.end(), s, cmp);
      if (it != speed_limit_points.end()) {
        speed_upper_bound->push_back(it->second);
      } else {
        speed_upper_bound->push_back(speed_limit_points.back().second);
      }
    }
  }

  const double kTimeBuffer = 2.0;
  const double kSpeedBuffer = 0.1;
  for (uint32_t k = 0; k < t_evaluated_.size() && t_evaluated_[k] < kTimeBuffer;
       ++k) {
    speed_upper_bound->at(k) =
        std::fmax(init_point_.v() + kSpeedBuffer, speed_upper_bound->at(k));
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
