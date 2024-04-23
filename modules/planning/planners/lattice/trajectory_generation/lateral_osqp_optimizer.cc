/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planners/lattice/trajectory_generation/lateral_osqp_optimizer.h"

#include "cyber/common/log.h"
#include "modules/common/math/matrix_operations.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

bool LateralOSQPOptimizer::optimize(
    const std::array<double, 3>& d_state, const double delta_s,
    const std::vector<std::pair<double, double>>& d_bounds) {
  std::vector<c_float> P_data; ///< 非零的数据
  std::vector<c_int> P_indices; ///<非0元素对应的行
  std::vector<c_int> P_indptr; ///<稀疏矩阵中表示非0元素几个
  ///稀疏矩阵
  CalculateKernel(d_bounds, &P_data, &P_indices, &P_indptr); ///<目标就是得P矩阵
  delta_s_ = delta_s;
  const int num_var = static_cast<int>(d_bounds.size()); ///< 60
  const int kNumParam = 3 * static_cast<int>(d_bounds.size()); ///< 3* 60 
  const int kNumConstraint = kNumParam + 3 * (num_var - 1) + 3; ///< 180 + 3*59  + 3 = 360
  c_float lower_bounds[kNumConstraint];
  c_float upper_bounds[kNumConstraint];

  const int prime_offset = num_var; ///< 60
  const int pprime_offset = 2 * num_var; ///< 120

  std::vector<std::vector<std::pair<c_int, c_float>>> columns;
  columns.resize(kNumParam); ///< 矩阵 180

  int constraint_index = 0;

  ///约束1横向加加速度应在一定范围内
  ///其实这里就是实现第二行第三块部分,按列来着
  for (int i = 0; i + 1 < num_var; ++i) { ///< 60
    columns[pprime_offset + i].emplace_back(constraint_index, -1.0); ///< 120 ~ 178
    columns[pprime_offset + i + 1].emplace_back(constraint_index, 1.0);///< 121 ~ 179

    lower_bounds[constraint_index] =
        -FLAGS_lateral_third_order_derivative_max * delta_s_; ///< -0.1*1
    upper_bounds[constraint_index] =
        FLAGS_lateral_third_order_derivative_max * delta_s_; ///< 0.1*1
    ++constraint_index;
  }

  ///约束2轨迹应该光滑，即导数连续相邻两个采样点的斜率
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[prime_offset + i].emplace_back(constraint_index, -1.0); ///< prime_offset = 60, 60 ~ 118
    columns[prime_offset + i + 1].emplace_back(constraint_index, 1.0);///< 61 ~ 119
    columns[pprime_offset + i].emplace_back(constraint_index, -0.5 * delta_s_); ///< pprime_offset = 120 120 ~ 178
    columns[pprime_offset + i + 1].emplace_back(constraint_index, -0.5 * delta_s_);///< 121 ~ 179

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  ///约束3轨迹应该保持连续
  for (int i = 0; i + 1 < num_var; ++i) {
    columns[i].emplace_back(constraint_index, -1.0); ///< 0 ~ 58
    columns[i + 1].emplace_back(constraint_index, 1.0); ///< 1 ~ 59
    columns[prime_offset + i].emplace_back(constraint_index, -delta_s_); ///< 60 ~ 118
    columns[pprime_offset + i].emplace_back(constraint_index,
                                            -delta_s_ * delta_s_ / 3.0); ///< 120 ~ 178
    columns[pprime_offset + i + 1].emplace_back(constraint_index,
                                                -delta_s_ * delta_s_ / 6.0); ///< 121 ~ 179

    lower_bounds[constraint_index] = 0.0;
    upper_bounds[constraint_index] = 0.0;
    ++constraint_index;
  }

  columns[0].emplace_back(constraint_index, 1.0);
  lower_bounds[constraint_index] = d_state[0]; ///< l
  upper_bounds[constraint_index] = d_state[0];
  ++constraint_index;

  columns[prime_offset].emplace_back(constraint_index, 1.0); ///< prime_offset = 60
  lower_bounds[constraint_index] = d_state[1]; ///< l_d
  upper_bounds[constraint_index] = d_state[1];
  ++constraint_index;

  columns[pprime_offset].emplace_back(constraint_index, 1.0); ///< pprime_offset = 120
  lower_bounds[constraint_index] = d_state[2]; ///< l_dd
  upper_bounds[constraint_index] = d_state[2];
  ++constraint_index;

  const double LARGE_VALUE = 2.0;
  for (int i = 0; i < kNumParam; ++i) { ///< kNumParam = 180
    columns[i].emplace_back(constraint_index, 1.0);
    if (i < num_var) {
      lower_bounds[constraint_index] = d_bounds[i].first; ///< 下边界
      upper_bounds[constraint_index] = d_bounds[i].second; ///< 上边界
    } else {
      lower_bounds[constraint_index] = -LARGE_VALUE; ///< -2,2作为缺省值不充值，凑够矩阵运算数量
      upper_bounds[constraint_index] = LARGE_VALUE;
    }
    ++constraint_index;
  }

  CHECK_EQ(constraint_index, kNumConstraint);

  // change affine_constraint to CSC format
  std::vector<c_float> A_data; ///< 非零的数据
  std::vector<c_int> A_indices; ///< 非0元素对应的行
  std::vector<c_int> A_indptr; ///< 稀疏矩阵中表示非0元素几个
  int ind_p = 0;
  for (int j = 0; j < kNumParam; ++j) {
    A_indptr.push_back(ind_p);
    for (const auto& row_data_pair : columns[j]) {
      A_data.push_back(row_data_pair.second);
      A_indices.push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  A_indptr.push_back(ind_p);

  // offset
  double q[kNumParam];
  for (int i = 0; i < kNumParam; ++i) { ///< 180
    if (i < num_var) {
      q[i] = -2.0 * FLAGS_weight_lateral_obstacle_distance *
             (d_bounds[i].first + d_bounds[i].second);
    } else {
      q[i] = 0.0;
    }
  }

  // Problem settings动态分配内存来创建一个 OSQPSettings 类型的对象，返回指向该对象的指针
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));///< reinterpret_cast 用于强制类型转换

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;  ///< 影响求解器的收敛速度和稳定性
  settings->eps_abs = 1.0e-05;///<绝对精度,终止条件
  settings->eps_rel = 1.0e-05;///<相对精度,终止条件
  settings->max_iter = 5000; ///<最大迭代次数
  settings->polish = true;///< 是否在求解结束后进行额外的步骤来提高解的精度
  settings->verbose = FLAGS_enable_osqp_debug; ///<false 决定了 OSQP 求解器是否输出详细的调试信息

  // Populate data
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = kNumParam; ///< 180
  data->m = kNumConstraint; ///< 360
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q;
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds;
  data->u = upper_bounds;

  // Workspace
  OSQPWorkspace* work = nullptr;///<定义了一个指向 OSQPWorkspace 类型的指针 work，并将其初始化为 nullptr
  // osqp_setup(&work, data, settings);
  work = osqp_setup(data, settings);///< 为求解器分配内存并初始化求解器

  // Solve Problem
  osqp_solve(work);

  // extract primal results
  for (int i = 0; i < num_var; ++i) {
    opt_d_.push_back(work->solution->x[i]);
    opt_d_prime_.push_back(work->solution->x[i + num_var]);
    opt_d_pprime_.push_back(work->solution->x[i + 2 * num_var]);
  }
  opt_d_prime_[num_var - 1] = 0.0;
  opt_d_pprime_[num_var - 1] = 0.0;

  // Cleanup
  osqp_cleanup(work); ///< 释放内存
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);

  return true;
}
/// @brief 也就是算P
/// @param d_bounds 
/// @param P_data 
/// @param P_indices 
/// @param P_indptr 
///使用的是csc_matrix，压速CSC 
void LateralOSQPOptimizer::CalculateKernel(
    const std::vector<std::pair<double, double>>& d_bounds,
    std::vector<c_float>* P_data, std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr) {
  const int kNumParam = 3 * static_cast<int>(d_bounds.size());///< 3* 60
  P_data->resize(kNumParam); ///<权重
  P_indices->resize(kNumParam);
  P_indptr->resize(kNumParam + 1);
 ///状态量就是前60维是l,再60维是l_d，再60维是l_dd
  for (int i = 0; i < kNumParam; ++i) {
    if (i < static_cast<int>(d_bounds.size())) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_offset +
                      2.0 * FLAGS_weight_lateral_obstacle_distance;///< 2* 1.0 + 2* 0
    } else if (i < 2 * static_cast<int>(d_bounds.size())) {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_derivative; ///< 2 * 500
    } else {
      P_data->at(i) = 2.0 * FLAGS_weight_lateral_second_order_derivative; ///< 2*1000
    }
    P_indices->at(i) = i;
    P_indptr->at(i) = i;
  }
  P_indptr->at(kNumParam) = kNumParam;
  CHECK_EQ(P_data->size(), P_indices->size());
}

}  // namespace planning
}  // namespace apollo
