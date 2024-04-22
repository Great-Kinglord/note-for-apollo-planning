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
 * @file spline_2d.cc
 **/

#include "modules/planning/math/smoothing_spline/spline_2d.h"

#include <algorithm>
#include <utility>

namespace apollo {
namespace planning {

Spline2d::Spline2d(const std::vector<double>& t_knots,
                   const std::uint32_t order)
    : t_knots_(t_knots), spline_order_(order) {
  if (t_knots.size() > 1) {
    for (std::uint32_t i = 1; i < t_knots_.size(); ++i) {
      splines_.emplace_back(spline_order_);
    }
  }
}

std::pair<double, double> Spline2d::operator()(const double t) const {
  if (splines_.size() == 0) {
    return std::make_pair(0.0, 0.0);
  }
  std::uint32_t index = find_index(t);
  return splines_[index](t - t_knots_[index]);
}

double Spline2d::x(const double t) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].x(t - t_knots_[index]);
}

double Spline2d::y(const double t) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].y(t - t_knots_[index]);
}

double Spline2d::DerivativeX(const double t) const {
  // zero order spline
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].DerivativeX(t - t_knots_[index]);
}

double Spline2d::derivative_y(const double t) const {
  // zero order spline
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].derivative_y(t - t_knots_[index]);
}

double Spline2d::SecondDerivativeX(const double t) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].SecondDerivativeX(t - t_knots_[index]);
}

double Spline2d::second_derivative_y(const double t) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].second_derivative_y(t - t_knots_[index]);
}

double Spline2d::ThirdDerivativeX(const double t) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].ThirdDerivativeX(t - t_knots_[index]);
}

double Spline2d::third_derivative_y(const double t) const {
  if (splines_.size() == 0) {
    return 0.0;
  }
  std::uint32_t index = find_index(t);
  return splines_[index].third_derivative_y(t - t_knots_[index]);
}
/**
*   @brief: set splines
**/
bool Spline2d::set_splines(const Eigen::MatrixXd& params,
                           const std::uint32_t order) {
  // check if the parameter size fit
  if (2 * t_knots_.size() * order !=
      2 * order + static_cast<std::uint32_t>(params.rows())) {
    return false;
  }
  for (std::uint32_t i = 0; i < splines_.size(); ++i) {
    std::vector<double> spline_piece_x(order, 0.0);
    std::vector<double> spline_piece_y(order, 0.0);
    for (std::uint32_t j = 0; j < order; ++j) {
      spline_piece_x[j] = params(2 * i * order + j, 0);
      spline_piece_y[j] = params((2 * i + 1) * order + j, 0);
    }
    splines_[i].SetParams(spline_piece_x, spline_piece_y);
  }
  spline_order_ = order;
  return true;
}

// get the mutable single smoothing spline, if index out of range, nullptr will
// returned;
Spline2dSeg* Spline2d::mutable_smoothing_spline(const std::uint32_t index) {
  return &splines_[index];
}

const Spline2dSeg& Spline2d::smoothing_spline(const std::uint32_t index) const {
  return splines_[index];
}

const std::vector<double>& Spline2d::t_knots() const { return t_knots_; }

std::uint32_t Spline2d::spline_order() const { return spline_order_; }

std::uint32_t Spline2d::find_index(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), t);
  return std::min(static_cast<std::uint32_t>(t_knots_.size() - 1),
                  static_cast<std::uint32_t>(upper_bound - t_knots_.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo
