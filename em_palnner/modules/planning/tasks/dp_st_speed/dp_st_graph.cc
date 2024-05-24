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
 * @file dp_st_graph.cc
 **/

#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::math::Vec2d;
using apollo::common::VehicleConfigHelper;
using apollo::common::VehicleParam;

namespace {
/// @brief 
/// @param boundaries 
/// @param p1 
/// @param p2 
/// @return 
bool CheckOverlapOnDpStGraph(const std::vector<StBoundary>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  for (const auto& boundary : boundaries) {
    common::math::LineSegment2d seg(p1.point(), p2.point());
    if (boundary.HasOverlap(seg)) {
      return true;
    }
  }
  return false;
}
}  // namespace

DpStGraph::DpStGraph(const ReferenceLine& reference_line,
                     const StGraphData& st_graph_data,
                     const DpStSpeedConfig& dp_config,
                     const PathData& path_data,
                     const SLBoundary& adc_sl_boundary)
    : reference_line_(reference_line),
      dp_st_speed_config_(dp_config),
      st_graph_data_(st_graph_data),
      path_data_(path_data),
      adc_sl_boundary_(adc_sl_boundary),
      dp_st_cost_(dp_config),
      init_point_(st_graph_data.init_point()) {
  dp_st_speed_config_.set_total_path_length(
      std::fmin(dp_st_speed_config_.total_path_length(),
                st_graph_data_.path_data_length())); ///< 标定total_path_length = 80m
  vehicle_param_ = VehicleConfigHelper::GetConfig().vehicle_param();
}
/// @brief DP主函数
/// @param path_decision 
/// @param speed_data 
/// @return 
Status DpStGraph::Search(PathDecision* const path_decision,
                         SpeedData* const speed_data) {
  
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  CalculatePointwiseCost(st_graph_data_.st_boundaries());

  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ///找到最佳的速度profile
  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!MakeObjectDecision(*speed_data, path_decision).ok()) {
    const std::string msg = "Get object decision by speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status DpStGraph::InitCostTable() {
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s(); ///? 默认100，s的维数？
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();///? 默认为10，t的维数？
  /// total_path_length 默认为80m
  if (Double::Compare(dp_st_speed_config_.total_path_length(), 0.0) == 0) {///< 长度 == 0
    unit_s_ = 1e-8;
    dim_s =
        std::min(dim_s, static_cast<uint32_t>(
                            dp_st_speed_config_.total_path_length() / unit_s_) + 1);///< 基本不会进入
  } else {
    unit_s_ = dp_st_speed_config_.total_path_length() / dim_s; ///< 80m/100,每个维度0.8
  }

  unit_t_ = dp_st_speed_config_.total_time() /
            dp_st_speed_config_.matrix_dimension_t(); ///< 8s/10,每个维度0.8s
  DCHECK_GT(dim_s, 2);
  DCHECK_GT(dim_t, 2);
  ///二维向量也就是一个矩阵，dim_t表示外层向量的大小，dim_s表示内层向量的大小
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  double curr_t = 0.0;
  ///矩阵建立起来了，每个刻度对应s,或者t
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    double curr_s = 0.0;
    for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  return Status::OK();
}

void DpStGraph::CalculatePointwiseCost(
    const std::vector<StBoundary>& boundaries) {
  // TODO(all): extract reference line from decision first
  std::vector<STPoint> reference_points;
  double curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i) {///< 这里size就是10
    reference_points.emplace_back(curr_t * dp_st_speed_config_.max_speed(),curr_t); ///< 位移，以及时间
    curr_t += unit_t_;
  }

  for (uint32_t i = 0; i < cost_table_.size(); ++i) {
    for (auto& st_graph_point : cost_table_[i]) {
      double ref_cost = dp_st_cost_.GetReferenceCost(st_graph_point.point(),
                                                     reference_points[i]);
      double obs_cost = dp_st_cost_.GetObstacleCost(st_graph_point.point(), boundaries);
      st_graph_point.SetReferenceCost(ref_cost);
      st_graph_point.SetObstacleCost(obs_cost);
      st_graph_point.SetTotalCost(std::numeric_limits<double>::infinity());
    }
  }
}

Status DpStGraph::CalculateTotalCost() {
  // s corresponding to row    行
  // time corresponding to col 列
  uint32_t next_highest_row = 0;
  uint32_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) { ///< 列，就是时间t
    uint32_t highest_row = 0;
    uint32_t lowest_row = cost_table_.back().size() - 1; ///< 行，就是s
    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      CalculateCostAt(c, r);
      uint32_t h_r = 0;
      uint32_t l_r = 0;
      ///小于无穷大，就是有效的cost
      if (cost_cr.total_cost() < std::numeric_limits<double>::infinity()) {
        GetRowRange(cost_cr, &h_r, &l_r); ///< 获取下一个点的行范围
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = std::max(next_lowest_row, lowest_row);
  }

  return Status::OK();
}

void DpStGraph::GetRowRange(const StGraphPoint& point,
                            uint32_t* next_highest_row,
                            uint32_t* next_lowest_row) {
  double v0 = 0.0;
  if (!point.pre_point()) { ///< 没有前一个点，自己就是初始点
    v0 = init_point_.v();
  } else {
    v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_; ///< 速度 = 位移/时间
  }
  const double speed_coeff = unit_t_ * unit_t_;

  const double delta_s_upper_bound =
      v0 * unit_t_ + vehicle_param_.max_acceleration() * speed_coeff; ///< delat位移上限
  *next_highest_row =
      point.index_s() + static_cast<uint32_t>(delta_s_upper_bound / unit_s_);///< 下一个点上限的索引
  if (*next_highest_row >= cost_table_.back().size()) {
    *next_highest_row = cost_table_.back().size() - 1;
  }

  const double delta_s_lower_bound = std::fmax(
      0.0, v0 * unit_t_ + vehicle_param_.max_deceleration() * speed_coeff);///< 车辆不能后退
  ///!是不是应该*next_lowest_row = point.index_s() + static_cast<int32_t>(delta_s_lower_bound / unit_s_);
  *next_lowest_row += static_cast<int32_t>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row >= cost_table_.back().size()) {
    *next_lowest_row = cost_table_.back().size() - 1;
  }
}
/// @brief 在st图上某个索引出的点计算cost
/// @param c 列 对应t
/// @param r 行 对应s
void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {
  auto& cost_cr = cost_table_[c][r];
  const auto& cost_init = cost_table_[0][0];
  /// [0 0]是可以的，但是列是0，行不能是0
  if (c == 0) {
    DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    return;
  }

  double speed_limit =
      st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);///<获取s处的速度限制
  /// 对应第一列，c=1需要单独处理，因为c=0的1列只有1个节点，即初始节点
  if (c == 1) {
    ///如果从起点到当前点有障碍物，那么直接返回，不计算cost
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr, cost_init)) {
      return;
    }
    ///和障碍物没有重叠，计算cost，点内cost,点间cost，从起点到[c=1,r]的各项cost，即[t0,s0]->[t1,sn]的cost
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_init.total_cost() +
                         CalculateEdgeCostForSecondCol(r, speed_limit));
    cost_cr.SetPrePoint(cost_init);
    return;
  }
  /// c>1的情况，计算cost
  const uint32_t max_s_diff = static_cast<uint32_t>(
      dp_st_speed_config_.max_speed() * unit_t_ / unit_s_);///< 速度*时间/位移，单位时间，最大s的delta索引,max_speed = 20m/s
  /// 缩小行范围到[r_low, r]，是缩小上一点的范围，当前点根据速度限值去找上一个点的范围
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);

  const auto& pre_col = cost_table_[c - 1];
  ///对应第二列,由于计算jerk需要三个点，c = 2那第一个点就是车辆的初始点
  if (c == 2) {
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
      ///上一列和当前列的点有碰撞，直接返回，cost_cr为[2][r]
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        return; ///! 应该是continue,而不是return
      }
      ///没有碰撞，计算cost，当前cost + 上个点的cost
      const double cost = cost_cr.obstacle_cost() +
                          pre_col[r_pre].total_cost() +
                          CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);
      ///在未赋有效cost值之前，cost_cr的total_cost是+inf
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
    return;
  }

  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) { ///< [0 r]
    ///上个点不可达
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }
    ///当前加速度，超出范围，直接跳过，当前点和上个点之间是匀速，上个点和上上个点之间也是匀速，求两个匀速之间的加速度
    const double curr_a =
        (cost_cr.index_s() + pre_col[r_pre].pre_point()->index_s() -
         2 * pre_col[r_pre].index_s()) * unit_s_ / (unit_t_ * unit_t_);
    if (curr_a > vehicle_param_.max_acceleration() ||
        curr_a < vehicle_param_.max_deceleration()) {
      continue;
    }

    uint32_t lower_bound = 0;
    uint32_t upper_bound = 0;
    ///缩小上上点的范围，正就是根据加速度来限值，上个点是通过速度来限值
    if (!CalculateFeasibleAccelRange(static_cast<double>(r_pre),
                                     static_cast<double>(r), &lower_bound, &upper_bound)) {
      continue;
    }
    ///和障碍物位置相交不可行驶
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      return; ///! 应该是continue,而不是return
    }
    ///遍历上上个点的范围，计算cost
    for (uint32_t r_prepre = lower_bound; r_prepre <= upper_bound; ++r_prepre) {
      const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
      if (std::isinf(prepre_graph_point.total_cost())) {///<此点不存在
        continue;
      }

      if (!prepre_graph_point.pre_point()) {
        continue;
      }
      ///连个点只能计算速度，三个点可以计算加速度，四个点才可以计算jerk
      const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
      const STPoint& prepre_point = prepre_graph_point.point();
      const STPoint& pre_point = pre_col[r_pre].point();
      const STPoint& curr_point = cost_cr.point();
      ///triple_pre_point和prepre_point只是计算jerk cost需要
      double cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                    CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                      curr_point, speed_limit);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
  }
}
/// @brief 通过加减速计算可行驶的区域，上上个点的范围
/// @param r_pre 
/// @param r_cur 
/// @param lower_bound 上上个点的索引下限
/// @param upper_bound 上上个点的索引上限
/// @return 
bool DpStGraph::CalculateFeasibleAccelRange(const double r_pre,
                                            const double r_cur,
                                            uint32_t* const lower_bound,
                                            uint32_t* const upper_bound) const {
  double tcoef = unit_t_ * unit_t_ / unit_s_;
  ///完全正确
  double lval = std::max(
      2 * r_pre - r_cur + dp_st_speed_config_.max_deceleration() * tcoef, 0.0);///< max_deceleration,可标，-6
  ///pre到cur是匀速，通过加减速阈值，得到cur上下范围，
  double rval = std::min(
      2 * r_pre - r_cur + dp_st_speed_config_.max_acceleration() * tcoef, r_pre);///< max_acceleration,可标，2

  if (rval < lval) {
    return false;
  }
  *lower_bound = static_cast<uint32_t>(lval);
  *upper_bound = static_cast<uint32_t>(rval);
  return true;
}
/// @brief 寻找到最佳的速度profile
/// @param speed_data 
/// @return 
Status DpStGraph::RetrieveSpeedProfile(SpeedData* const speed_data) const {
  double min_cost = std::numeric_limits<double>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  ///搜索最后t时间的s，搜索s
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  ///最后一个点可能不是落在最后的时间t列,也有可能在最大s行，搜索t
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  ///回溯出完整的st
  while (cur_point != nullptr) {
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());///<顺序颠倒下
  ///第一个点就是初始点，t和s都是0
  if (Double::Compare(speed_profile.front().t(), 0.0) != 0 ||
      Double::Compare(speed_profile.front().s(), 0.0) != 0) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  speed_data->set_speed_vector(speed_profile);
  return Status::OK();
}
/// @brief 给障碍物做决策标签
/// @param speed_profile 
/// @param path_decision 
/// @return 
Status DpStGraph::MakeObjectDecision(const SpeedData& speed_profile,
                                     PathDecision* const path_decision) const {
  ///速度profile的点数小于2，失败
  if (speed_profile.speed_vector().size() < 2) {
    const std::string msg = "dp_st_graph failed to get speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ///目标在后方或者最大预测时间小于0，不需要做决策
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    if (boundary.max_s() < 0.0 || boundary.max_t() < 0.0) {
      continue;
    }

    auto* path_obstacle = path_decision->Find(boundary.id());///<通过ID找到障碍物
    CHECK(path_obstacle) << "Failed to find obstacle " << boundary.id();
    ///已经有决策了，跳过
    if (path_obstacle->HasLongitudinalDecision()) {
      continue;
    }

    double start_t = boundary.min_t();
    double end_t = boundary.max_t();

    bool go_down = true;
    ///遍历速度profile的点
    for (const auto& speed_point : speed_profile.speed_vector()) {
      if (speed_point.t() < start_t) {
        continue;
      }
      if (speed_point.t() > end_t) {
        break;
      }

      STPoint st_point(speed_point.s(), speed_point.t());
      ///点在目标boundary内，这是有碰撞风险的
      if (boundary.IsPointInBoundary(st_point)) {
        const std::string msg =
            "dp_st_graph failed: speed profile cross st_boundaries.";
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }

      double s_upper = dp_st_speed_config_.total_path_length(); ///< 80m，标定量
      double s_lower = 0.0;
      ///在该t的时候，看boundary的s范围
      if (boundary.GetBoundarySRange(speed_point.t(), &s_upper, &s_lower)) {
        if (s_lower > speed_point.s()) {
          ///规划的速度在障碍物此时s的下方，表示减速让行或者跟车
          go_down = true;
        } else if (s_upper < speed_point.s()) {
          go_down = false;
        }
      }
    }
    ///规划的速度在障碍物此时s的下方
    if (go_down) {
      ///先检查目标在这个boundary是否正常
      if (CheckIsFollowByT(boundary)) {
        ///跟车决策
        ObjectDecisionType follow_decision;
        if (!CreateFollowDecision(*path_obstacle, boundary, &follow_decision)) {
          AERROR << "Failed to create follow decision for boundary with id "
                 << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to create follow decision");
        }
        ///更新决策
        if (!path_decision->AddLongitudinalDecision(
                "dp_st_graph", boundary.id(), follow_decision)) {
          AERROR << "Failed to add follow decision to object " << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to add follow decision");
        }
      } else {
        ///车辆对对向来车，或者静止，让行，应该就是停车的意思
        ObjectDecisionType yield_decision;
        if (!CreateYieldDecision(boundary, &yield_decision)) {
          AERROR << "Failed to create yield decision for boundary with id "
                 << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to create yield decision");
        }
        if (!path_decision->AddLongitudinalDecision(
                "dp_st_graph", boundary.id(), yield_decision)) {
          AERROR << "Failed to add yield decision to object " << boundary.id();
          return Status(ErrorCode::PLANNING_ERROR,
                        "faind to add yield decision");
        }
      }
    } else {
      ///超车
      ObjectDecisionType overtake_decision;
      if (!CreateOvertakeDecision(*path_obstacle, boundary,
                                  &overtake_decision)) {
        AERROR << "Failed to create overtake decision for boundary with id "
               << boundary.id();
        return Status(ErrorCode::PLANNING_ERROR,
                      "faind to create overtake decision");
      }
      if (!path_decision->AddLongitudinalDecision("dp_st_graph", boundary.id(),
                                                  overtake_decision)) {
        AERROR << "Failed to add overtake decision to object " << boundary.id();
        return Status(ErrorCode::PLANNING_ERROR,
                      "faind to add overtake decision");
      }
    }
  }
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    ///没有决策，忽略
    if (!path_obstacle->HasLongitudinalDecision()) {
      ObjectDecisionType ignore_decision;
      ignore_decision.mutable_ignore();
      path_decision->AddLongitudinalDecision("dp_st_graph", path_obstacle->Id(),
                                             ignore_decision);
    }
  }
  return Status::OK();
}

bool DpStGraph::CreateFollowDecision(
    const PathObstacle& path_obstacle, const StBoundary& boundary,
    ObjectDecisionType* const follow_decision) const {
  DCHECK_NOTNULL(follow_decision);

  auto* follow = follow_decision->mutable_follow();///< proto自动生成mutable_

  const double follow_speed = init_point_.v();
  ///跟车距离follow_time_buffer：4.0 follow_min_distance：10m，或者4s*车速的距离
  const double follow_distance_s = -std::fmax(
      follow_speed * FLAGS_follow_time_buffer, FLAGS_follow_min_distance);

  follow->set_distance_s(follow_distance_s);

  const double refence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + follow_distance_s;
  auto ref_point = reference_line_.GetReferencePoint(refence_s);
  auto* fence_point = follow->mutable_fence_point();
  fence_point->set_x(ref_point.x());
  fence_point->set_y(ref_point.y());
  fence_point->set_z(0.0);
  follow->set_fence_heading(ref_point.heading());

  return true;
}

bool DpStGraph::CreateYieldDecision(
    const StBoundary& boundary,
    ObjectDecisionType* const yield_decision) const {
  auto* yield = yield_decision->mutable_yield();

  // in meters
  constexpr double kMinYieldDistance = 10.0;
  const double yield_distance_s =
      std::max(-boundary.min_s(), -1.0 * kMinYieldDistance);
  yield->set_distance_s(yield_distance_s);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + yield_distance_s;
  auto ref_point = reference_line_.GetReferencePoint(reference_line_fence_s);

  yield->mutable_fence_point()->set_x(ref_point.x());
  yield->mutable_fence_point()->set_y(ref_point.y());
  yield->mutable_fence_point()->set_z(0.0);
  yield->set_fence_heading(ref_point.heading());

  return true;
}

bool DpStGraph::CreateOvertakeDecision(
    const PathObstacle& path_obstacle, const StBoundary& boundary,
    ObjectDecisionType* const overtake_decision) const {
  DCHECK_NOTNULL(overtake_decision);

  auto* overtake = overtake_decision->mutable_overtake();

  // in seconds
  constexpr double kOvertakeTimeBuffer = 3.0;
  // in meters
  constexpr double kMinOvertakeDistance = 10.0;

  const auto& velocity = path_obstacle.obstacle()->Perception().velocity();
  const double obstacle_speed =
      Vec2d::CreateUnitVec2d(init_point_.path_point().theta())
          .InnerProd(Vec2d(velocity.x(), velocity.y()));

  const double overtake_distance_s = std::fmax(
      std::fmax(init_point_.v(), obstacle_speed) * kOvertakeTimeBuffer,
      kMinOvertakeDistance);
  overtake->set_distance_s(overtake_distance_s);

  const double reference_line_fence_s =
      adc_sl_boundary_.end_s() + boundary.min_s() + overtake_distance_s;

  auto ref_point = reference_line_.GetReferencePoint(reference_line_fence_s);
  overtake->mutable_fence_point()->set_x(ref_point.x());
  overtake->mutable_fence_point()->set_y(ref_point.y());
  overtake->mutable_fence_point()->set_z(0.0);
  overtake->set_fence_heading(ref_point.heading());

  return true;
}

double DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                    const STPoint& third, const STPoint& forth,
                                    const double speed_limit) const {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

double DpStGraph::CalculateEdgeCostForSecondCol(
    const uint32_t row, const double speed_limit) const {
  double init_speed = init_point_.v();///<初始速度
  double init_acc = init_point_.a();///<初始加速度
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

double DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                               const uint32_t pre_row,
                                               const double speed_limit) const {
  double init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

bool DpStGraph::CheckIsFollowByT(const StBoundary& boundary) const {
  if (boundary.BottomLeftPoint().s() > boundary.BottomRightPoint().s()) {
    return false;
  }
  const double kFollowTimeEpsilon = 1e-3;
  if (boundary.min_t() > kFollowTimeEpsilon ||
      boundary.max_t() < kFollowTimeEpsilon) {
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
