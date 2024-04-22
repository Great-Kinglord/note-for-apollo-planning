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
 * @file : piecewise_smoothing_spline_constraint.h
 * @brief: wrapp up solver constraint interface with direct methods and preset
 *methods
 **/

#include "modules/planning/math/smoothing_spline/spline_1d_kernel.h"

#include <algorithm>

#include "modules/planning/math/smoothing_spline/spline_seg_kernel.h"

namespace apollo {
namespace planning {

Spline1dKernel::Spline1dKernel(const Spline1d& spline1d)
    : x_knots_(spline1d.x_knots()), spline_order_(spline1d.spline_order()) {
  total_params_ =
      (x_knots_.size() > 1 ? (x_knots_.size() - 1) * spline_order_ : 0);
  kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
  offset_ = Eigen::MatrixXd::Zero(total_params_, 1);
}

Spline1dKernel::Spline1dKernel(const std::vector<double>& x_knots,
                               const std::uint32_t spline_order)
    : x_knots_(x_knots), spline_order_(spline_order) {
  total_params_ =
      (x_knots.size() > 1 ? (x_knots.size() - 1) * spline_order_ : 0);
  kernel_matrix_ = Eigen::MatrixXd::Zero(total_params_, total_params_);
  offset_ = Eigen::MatrixXd::Zero(total_params_, 1);
}

void Spline1dKernel::AddRegularization(const double regularized_param) {
  Eigen::MatrixXd id_matrix =
      Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols());
  kernel_matrix_ += id_matrix * regularized_param;
}

bool Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                               const Eigen::MatrixXd& offset,
                               const double weight) {
  if (kernel.rows() != kernel.cols() ||
      kernel.rows() != kernel_matrix_.rows() || offset.cols() != 1 ||
      offset.rows() != offset_.rows()) {
    return false;
  }
  kernel_matrix_ += kernel * weight;
  offset_ += offset * weight;
  return true;
}

bool Spline1dKernel::AddKernel(const Eigen::MatrixXd& kernel,
                               const double weight) {
  Eigen::MatrixXd offset = Eigen::MatrixXd::Zero(kernel.rows(), 1);
  return AddKernel(kernel, offset, weight);
}

Eigen::MatrixXd* Spline1dKernel::mutable_kernel_matrix() {
  return &kernel_matrix_;
}

Eigen::MatrixXd* Spline1dKernel::mutable_offset() { return &offset_; }

const Eigen::MatrixXd& Spline1dKernel::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& Spline1dKernel::offset() const { return offset_; }

// build-in kernel methods
void Spline1dKernel::AddDerivativeKernelMatrix(const double weight) {
  for (std::uint32_t i = 0; i + 1 < x_knots_.size(); ++i) {
    Eigen::MatrixXd cur_kernel =
        SplineSegKernel::instance()->DerivativeKernel(
            spline_order_, x_knots_[i + 1] - x_knots_[i]) *
        weight;
    kernel_matrix_.block(i * spline_order_, i * spline_order_, spline_order_,
                         spline_order_) += cur_kernel;
  }
}

void Spline1dKernel::AddSecondOrderDerivativeMatrix(const double weight) {
  for (std::uint32_t i = 0; i + 1 < x_knots_.size(); ++i) {
    Eigen::MatrixXd cur_kernel =
        SplineSegKernel::instance()->SecondOrderDerivativeKernel(
            spline_order_, x_knots_[i + 1] - x_knots_[i]) *
        weight;
    kernel_matrix_.block(i * spline_order_, i * spline_order_, spline_order_,
                         spline_order_) += cur_kernel;
  }
}

void Spline1dKernel::AddThirdOrderDerivativeMatrix(const double weight) {
  for (std::uint32_t i = 0; i + 1 < x_knots_.size(); ++i) {
    Eigen::MatrixXd cur_kernel =
        SplineSegKernel::instance()->ThirdOrderDerivativeKernel(
            spline_order_, x_knots_[i + 1] - x_knots_[i]) *
        weight;
    kernel_matrix_.block(i * spline_order_, i * spline_order_, spline_order_,
                         spline_order_) += cur_kernel;
  }
}

bool Spline1dKernel::AddReferenceLineKernelMatrix(
    const std::vector<double>& x_coord, const std::vector<double>& ref_x,
    const double weight) {
  if (ref_x.size() != x_coord.size()) {
    return false;
  }

  for (std::uint32_t i = 0; i < x_coord.size(); ++i) {
    double cur_index = find_index(x_coord[i]);
    double cur_rel_x = x_coord[i] - x_knots_[cur_index];
    // update offset
    double offset_coef = -2.0 * ref_x[i] * weight;
    for (std::uint32_t j = 0; j < spline_order_; ++j) {
      offset_(j + cur_index * spline_order_, 0) += offset_coef;
      offset_coef *= cur_rel_x;
    }
    // update kernel matrix
    Eigen::MatrixXd ref_kernel(spline_order_, spline_order_);

    double cur_x = 1.0;
    std::vector<double> power_x;
    for (std::uint32_t n = 0; n + 1 < 2 * spline_order_; ++n) {
      power_x.emplace_back(cur_x);
      cur_x *= cur_rel_x;
    }

    for (std::uint32_t r = 0; r < spline_order_; ++r) {
      for (std::uint32_t c = 0; c < spline_order_; ++c) {
        ref_kernel(r, c) = power_x[r + c];
      }
    }

    kernel_matrix_.block(cur_index * spline_order_, cur_index * spline_order_,
                         spline_order_, spline_order_) += weight * ref_kernel;
  }
  return true;
}

std::uint32_t Spline1dKernel::find_index(const double x) const {
  auto upper_bound = std::upper_bound(x_knots_.begin() + 1, x_knots_.end(), x);
  return std::min(static_cast<std::uint32_t>(x_knots_.size() - 1),
                  static_cast<std::uint32_t>(upper_bound - x_knots_.begin())) -
         1;
}

void Spline1dKernel::add_distance_offset(const double weight) {
  for (std::uint32_t i = 1; i < x_knots_.size(); ++i) {
    const double cur_x = x_knots_[i] - x_knots_[i - 1];
    double pw_x = 2.0 * weight;
    for (std::uint32_t j = 0; j < spline_order_; ++j) {
      offset_((i - 1) * spline_order_ + j, 0) -= pw_x;
      pw_x *= cur_x;
    }
  }
}
}  // namespace planning
}  // namespace apollo
