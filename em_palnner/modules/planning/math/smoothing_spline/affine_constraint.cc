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
 * @file : affine_constraint.cc
 **/

#include "modules/planning/math/smoothing_spline/affine_constraint.h"

#include "modules/common/log.h"

namespace apollo {
namespace planning {

AffineConstraint::AffineConstraint(const bool is_equality)
    : is_equality_(is_equality) {}

AffineConstraint::AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_boundary,
                                   const bool is_equality)
    : constraint_matrix_(constraint_matrix),
      constraint_boundary_(constraint_boundary),
      is_equality_(is_equality) {
  CHECK_EQ(constraint_boundary.rows(), constraint_matrix.rows());
}

void AffineConstraint::SetIsEquality(const double is_equality) {
  is_equality_ = is_equality;
}

const Eigen::MatrixXd& AffineConstraint::constraint_matrix() const {
  return constraint_matrix_;
}

const Eigen::MatrixXd& AffineConstraint::constraint_boundary() const {
  return constraint_boundary_;
}
/// @brief 
/// @param constraint_matrix 
/// @param constraint_boundary 
/// @return 
bool AffineConstraint::AddConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
///todo 暂时不理解，后面再看，稍稍理解了
  if (constraint_matrix.rows() != constraint_boundary.rows()) {
    return false;
  }
  ///如果约束矩阵为空，直接赋值，也就是第一次添加约束，后面再添加就要去插入了
  if (constraint_matrix_.rows() == 0) {
    constraint_matrix_ = constraint_matrix;
    constraint_boundary_ = constraint_boundary;
    return true;
  }

  if (constraint_matrix_.cols() != constraint_matrix.cols() ||
      constraint_boundary.cols() != 1) {
    return false;
  }
  Eigen::MatrixXd n_matrix(constraint_matrix_.rows() + constraint_matrix.rows(),
                           constraint_matrix_.cols());
  Eigen::MatrixXd n_boundary(
      constraint_boundary_.rows() + constraint_boundary.rows(), 1);
  ///使用<<操作符，将constraint_matrix_和constraint_matrix插入到n_matrix
  n_matrix << constraint_matrix_, constraint_matrix;///< 先插入constraint_matrix_，在后面再插入constraint_matrix
  n_boundary << constraint_boundary_, constraint_boundary;
  constraint_matrix_ = n_matrix;///< constraint_matrix_是动态大小的矩阵
  constraint_boundary_ = n_boundary;
  return true;
}

}  // namespace planning
}  // namespace apollo
