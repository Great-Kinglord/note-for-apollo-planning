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
          qp_splsine_st_speed_config_.number_of_evaluated_graph_t()) {
  ///total_time:8s number_of_discrete_graph_t:4 number_of_evaluated_graph_t:10，我们在txt中看到是50
  ///!txt中的信息就是正确的，就是覆盖了默认的参数值，4表示四段五次多项式连接，按时间就是0，2，4，6，8
  ///! t_evaluated_resolution_ = 8/50 = 0.16s
  Init();
}

void QpSplineStGraph::Init() {
  // init knots，t_knots_的size为5
  double curr_t = 0.0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_discrete_graph_t(); ++i) {
    t_knots_.push_back(curr_t);
    curr_t += t_knots_resolution_; ///< 如果是4的话，就是2s
  }

  ///! number_of_evaluated_graph_t在txt中可不是4了，而是50
  curr_t = 0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_evaluated_graph_t(); ++i) {
    t_evaluated_.push_back(curr_t); ///< 大小就是51了
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

  // reset spline generator，这里reset用法改变智能指针的指向，它会先销毁原来的对象（如果有的话），然后指向新的对象
  spline_generator_.reset(new Spline1dGenerator(
      t_knots_, qp_spline_st_speed_config_.spline_order()));///? spline_order因该是五次多项式，但是默认6,是不是应该理解为6个未知数，理解正确

  // start to search for best st points
  init_point_ = st_graph_data.init_point();///< 起始点,包含了xyz，速度，加速度，时间，角度等等
  ///规划的路径长度小于80m的话，就是上限就是80m
  if (st_graph_data.path_data_length() <
      qp_spline_st_speed_config_.total_path_length()) {
    qp_spline_st_speed_config_.set_total_path_length(
        st_graph_data.path_data_length());
  }
  ///!约束
  if (!ApplyConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
                       st_graph_data.st_boundaries(), accel_bound)
           .ok()) {
    const std::string msg = "Apply constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ///!目标函数，包含H和g
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

  ///提取输出
  speed_data->Clear();
  const Spline1d& spline = spline_generator_->spline();

  double t_output_resolution =
      qp_spline_st_speed_config_.output_time_resolution();///< 0.05s
  double time = 0.0;
  ///总共8s
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
/// @brief 添加约束,包含等式和不等式约束 lb <= Ax <= ub以及lba <= x <= uba,向量都是24*1矩阵
/// @param init_point 
/// @param speed_limit 
/// @param boundaries 
/// @param accel_bound 
/// @return 
Status QpSplineStGraph::ApplyConstraint(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    const std::vector<StBoundary>& boundaries,
    const std::pair<double, double>& accel_bound) {
  Spline1dConstraint* constraint =
      spline_generator_->mutable_spline_constraint();
  /// position, velocity, acceleration，位置、速度、加速度
  ///必须过原点
  if (!constraint->AddPointConstraint(0.0, 0.0)) {
    const std::string msg = "add st start point constraint failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ADEBUG << "init point constraint:" << init_point.DebugString();
  ///原点处的斜率必须等于起始速度
  if (!constraint->AddPointDerivativeConstraint(0.0, init_point_.v())) {
    const std::string msg = "add st start point velocity constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ///st末端点的加速度限制为0
  if (!constraint->AddPointSecondDerivativeConstraint(
          spline_generator_->spline().x_knots().back(), 0.0)) {
    const std::string msg = "add st end point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // monotone constraint 单调约束，不等式约束
  if (!constraint->AddMonotoneInequalityConstraintAtKnots()) {
    const std::string msg = "add monotone inequality constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // smoothness constraint 平滑约束
  if (!constraint->AddSecondDerivativeSmoothConstraint()) {
    const std::string msg = "add smoothness joint constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // boundary constraint 边界约束
  std::vector<double> s_upper_bound;
  std::vector<double> s_lower_bound;
  /// 遍历时间，当前每2s一个间隔，获取s的上下界
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
  ///增加边界约束
  if (!constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound)) {
    const std::string msg = "Fail to apply distance constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // 速度约束
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

  // acceleration constraint加速度约束
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
/// @brief 这应该是核心之一，目标函数
/// @param boundaries 
/// @param speed_limit 
/// @return 
Status QpSplineStGraph::ApplyKernel(const std::vector<StBoundary>& boundaries,
                                    const SpeedLimit& speed_limit) {
  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();
  ///加速度权重，五次多项式二阶导是和t相关的，t不同对应不同的加速度，这里用到在这个时间段的二阶导平方的积分
  if (qp_spline_st_speed_config_.accel_kernel_weight() > 0) {
    spline_kernel->AddSecondOrderDerivativeMatrix(
        qp_spline_st_speed_config_.accel_kernel_weight());///< 权重是1000
  }
  ///加加速度权重
  if (qp_spline_st_speed_config_.jerk_kernel_weight() > 0) {
    spline_kernel->AddThirdOrderDerivativeMatrix(
        qp_spline_st_speed_config_.jerk_kernel_weight());///< 权重是500
  }
  ///位置的cost矩阵，保证拟合的st路径和最终的st路径相似
  if (!AddCruiseReferenceLineKernel(speed_limit,
                                    qp_spline_st_speed_config_.cruise_weight()) 
           .ok()) {
    ///< cruise_weight值为0.3
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::ApplyKernel");
  }
  ///跟车的cost矩阵，需求就是跟车的时候，距离一定要保持在一定范围内，不能被甩的太开
  if (!AddFollowReferenceLineKernel(boundaries,
                                    qp_spline_st_speed_config_.follow_weight())
           .ok()) {
    ///< follow_weight值为2.0
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::ApplyKernel");
  }
  return Status::OK();
}

Status QpSplineStGraph::Solve() {
  return spline_generator_->Solve()
             ? Status::OK()
             : Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::solve");
}
/// @brief 
/// @param speed_limit 
/// @param weight —— 0.3
/// @return 
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
  ///这里的t_evaluated_的size为51了
  for (uint32_t i = 1; i < t_evaluated_.size(); ++i) {
    dist_ref += (t_evaluated_[i] - t_evaluated_[i - 1]) *
                speed_limit.GetSpeedLimitByS(dist_ref);
    cruise_.push_back(dist_ref);///<当然0处的s是0，时间为0 ，0.16 ...8
  }
  if (st_graph_debug_) {
    ///STGraphDebug是proto中的结构体，mutable_kernel_cruise_ref是一个结构体，包含了t和s
    auto kernel_cruise_ref = st_graph_debug_->mutable_kernel_cruise_ref();
    kernel_cruise_ref->mutable_t()->Add(t_evaluated_[0]);///< 赋值t,0处的t是0
    kernel_cruise_ref->mutable_cruise_line_s()->Add(dist_ref);///< 赋值s，0处的s
    for (uint32_t i = 1; i < t_evaluated_.size(); ++i) {
      kernel_cruise_ref->mutable_t()->Add(t_evaluated_[i]);
      kernel_cruise_ref->mutable_cruise_line_s()->Add(cruise_[i]);
    }
  }
 
  if (t_evaluated_.size() > 0) {
    spline_kernel->AddReferenceLineKernelMatrix(
        t_evaluated_, cruise_,
        weight * qp_spline_st_speed_config_.total_time() / t_evaluated_.size());
  }///<其中weight为0.3 0.3*8/51
  spline_kernel->AddRegularization(0.01);///todo 不太懂，后面继续
  return Status::OK();
}

/// @brief 跟车的cost
/// @param boundaries st图
/// @param weight —— 2.0
/// @return 
Status QpSplineStGraph::AddFollowReferenceLineKernel(
    const std::vector<StBoundary>& boundaries, const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  std::vector<double> ref_s;
  std::vector<double> filtered_evaluate_t;
  /// 遍历时间0 0.16 0.32 ...8，总共50次循环
  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    const double curr_t = t_evaluated_[i];///<当前时间
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
      if (boundary.GetUnblockSRange(curr_t, &s_upper, &s_lower)) {///!跟车的话s_supper就是block的下边缘
        success = true;
        ///characteristic_length：0.1m
        s_min = std::min(s_min, s_upper - boundary.characteristic_length() -
                    qp_spline_st_speed_config_.follow_drag_distance()); ///< follow_drag_distance：17m
        
      }
    }
    /// 按照限速的话大于s_min的s就要考虑了，如果限速都小于s_min的话，就不用考虑了
    /// 也就是需要在考虑限速下，车辆跟随前车在17m的距离内
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
  ///filtered_evaluate_t：满足条件点的时间，ref_s：考虑跟车距离的s_ref
  if (!ref_s.empty()) {
    spline_kernel->AddReferenceLineKernelMatrix(
        filtered_evaluate_t, ref_s,
        weight * qp_spline_st_speed_config_.total_time() / t_evaluated_.size());///< 2.0*8/51
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
    ///获取s的上下界
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
