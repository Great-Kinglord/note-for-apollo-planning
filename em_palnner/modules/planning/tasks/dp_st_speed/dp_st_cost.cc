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
 * @file dp_st_cost.cc
 **/


#include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"

#include <limits>

#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

DpStCost::DpStCost(const DpStSpeedConfig& dp_st_speed_config)
    : dp_st_speed_config_(dp_st_speed_config),
      unit_s_(dp_st_speed_config_.total_path_length() /
              dp_st_speed_config_.matrix_dimension_s()),
      unit_t_(dp_st_speed_config_.total_time() /
              dp_st_speed_config_.matrix_dimension_t()) {}

// TODO(all): normalize cost with time
double DpStCost::GetObstacleCost(
    const STPoint& point, const std::vector<StBoundary>& st_boundaries) const {
  double total_cost = 0.0;
  for (const StBoundary& boundary : st_boundaries) {
    if (point.s() < 0 || boundary.IsPointInBoundary(point)) { ///<某些s在障碍物st图上，那就代表碰撞，cost设置为无穷大，s也不能小于0，就是不能往回走
      total_cost = std::numeric_limits<double>::infinity();
      break;
    } else {
      const double distance = boundary.DistanceS(point);
      ///default_obstacle_cost = 1e10，obstacle_cost_factor = -300，distance越小表示越近，cost越大
      total_cost += dp_st_speed_config_.default_obstacle_cost() *
                    std::exp(dp_st_speed_config_.obstacle_cost_factor() /
                             boundary.characteristic_length() * distance);
    }
  }
  return total_cost * unit_t_;
}
/// @brief 
/// @param point 
/// @param reference_point 
/// @return 
double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const {
  ///! 这个cost竟然没用到，应为reference_weight为0
  return dp_st_speed_config_.reference_weight() *
         (point.s() - reference_point.s()) * (point.s() - reference_point.s()) * unit_t_;///<权重*(实际位置-参考位置)^2 * 单位时间
}

/// @brief 速度cost，速度超过限制，或者速度太低，都会有cost
/// @param first 上一点
/// @param second 当前点
/// @param speed_limit 限速
/// @return 
double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit) const {
  double cost = 0.0;
  const double speed = (second.s() - first.s()) / unit_t_;///< delta_s / delta_t = v
  if (Double::Compare(speed, 0.0) < 0) {
    return std::numeric_limits<double>::infinity();///<速度小于0，cost设置为无穷大,不准往回走
  }
  double det_speed = (speed - speed_limit) / speed_limit;///<超过限速的百分比
  if (Double::Compare(det_speed, 0.0) > 0) {
    ///10 * 1.0 * 速度^2 * 单位时间 优先级最低
    cost = dp_st_speed_config_.exceed_speed_penalty() *
           dp_st_speed_config_.default_speed_cost() * fabs(speed * speed) *
           unit_t_;
  } else if (Double::Compare(det_speed, 0.0) < 0) {
    ///2.5 * 1.0 * 速度^2 * -det_speed * 单位时间，看起来是第二优先
    cost = dp_st_speed_config_.low_speed_penalty() *
           dp_st_speed_config_.default_speed_cost() * -det_speed * unit_t_;
  } else {
    ///速度等于限速，这是最优先的
    cost = 0.0;
  }
  return cost;
}
/// @brief 获取加速度cost
/// @param accel 
/// @return 
double DpStCost::GetAccelCost(const double accel) const {
  const double accel_sq = accel * accel;
  double max_acc = dp_st_speed_config_.max_acceleration(); ///<最大加速度，可以4.5，可以2.0，可标定
  double max_dec = dp_st_speed_config_.max_deceleration(); ///<最大减速度，可以-6，可以-4.5，可标定
  double accel_penalty = dp_st_speed_config_.accel_penalty(); ///<加速度惩罚，2.0，可标定
  double decel_penalty = dp_st_speed_config_.decel_penalty(); ///<减速度惩罚，2.0，可标定
  double cost = 0.0;
  if (accel > 0.0) {
    cost = accel_penalty * accel_sq;
  } else {
    cost = decel_penalty * accel_sq;
  }
  cost += accel_sq * decel_penalty * decel_penalty /
              (1 + std::exp(1.0 * (accel - max_dec))) +
          accel_sq * accel_penalty * accel_penalty /
              (1 + std::exp(-1.0 * (accel - max_acc)));
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) const {
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}
/// @brief 加速度cost，根据两个点的速度和位置，计算加速度，然后计算cost
/// @param pre_speed 
/// @param pre_point 
/// @param curr_point 
/// @return 
double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) const {
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  double accel = (current_speed - pre_speed) / unit_t_; ///<(v1 - v0) / t = a
  return GetAccelCost(accel);
}

double DpStCost::JerkCost(const double jerk) const {
  double jerk_sq = jerk * jerk;
  double cost = 0.0;
  const auto diff = Double::Compare(jerk, 0.0);
  if (diff > 0) {
    cost = dp_st_speed_config_.positive_jerk_coeff() * jerk_sq * unit_t_; ///<positive_jerk_coeff = 1.0
  } else if (diff < 0) {
    cost = dp_st_speed_config_.negative_jerk_coeff() * jerk_sq * unit_t_;///<negative_jerk_coeff = 300
  }
  ///优先jerk = 0，其次正jerk，最后负jerk
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) const {
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) const {
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_accel - pre_acc) / unit_t_; ///< 算加加速度
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) const {
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;
  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
