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
 * @file : spline_seg_kernel.cc
 * @brief: generating spline kernels
 **/

#include "modules/planning/math/smoothing_spline/spline_seg_kernel.h"

#include <vector>

namespace apollo {
namespace planning {

SplineSegKernel::SplineSegKernel() {
  reserved_order_ = 6;
  calculate_fx(reserved_order_);
  CalculateDerivative(reserved_order_);
  CalculateSecondOrderDerivative(reserved_order_);
  CalculateThirdOrderDerivative(reserved_order_);
}

Eigen::MatrixXd SplineSegKernel::Kernel(const std::uint32_t order,
                                        const double accumulated_x) {
  if (order > reserved_order_) {
    calculate_fx(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "fx", &term_matrix);
  return kernel_fx_.block(0, 0, order, order).cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::DerivativeKernel(const std::uint32_t order,
                                                  const double accumulated_x) {
  if (order > reserved_order_) {
    CalculateDerivative(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "derivative", &term_matrix);
  return kernel_derivative_.block(0, 0, order, order).cwiseProduct(term_matrix);
}
/// @brief 二阶导，也就是速度
/// @param order 这里是6
/// @param accumulated_x 其实也就是2s，下一个节点-当前节点
/// @return 
Eigen::MatrixXd SplineSegKernel::SecondOrderDerivativeKernel(
    const std::uint32_t order, const double accumulated_x) {
  ///由于reserved_order_给了6，所以这里不会执行
  if (order > reserved_order_) {
    CalculateSecondOrderDerivative(order);
  }
  Eigen::MatrixXd term_matrix;///<任意大小的动态矩阵，元素为double类型，这里应该是6*6
  integrated_term_matrix(order, accumulated_x, "second_order", &term_matrix);
  /* 解释下面的使用
  * block先获取一个子模块，其实这里就是获取的整个矩阵
  */
 ///!这里得到二阶导后积分不包含t的矩阵*t的矩阵(元素对应相乘) 
  return kernel_second_order_derivative_.block(0, 0, order, order).cwiseProduct(term_matrix);
}

Eigen::MatrixXd SplineSegKernel::ThirdOrderDerivativeKernel(
    const std::uint32_t order, const double accumulated_x) {
  if (order > reserved_order_) {
    CalculateThirdOrderDerivative(order);
  }
  Eigen::MatrixXd term_matrix;
  integrated_term_matrix(order, accumulated_x, "third_order", &term_matrix);
  return (kernel_third_order_derivative_.block(0, 0, order, order))
      .cwiseProduct(term_matrix);
}

void SplineSegKernel::integrated_term_matrix(
    const std::uint32_t order, const double x, const std::string& type,
    Eigen::MatrixXd* term_matrix) const {
  if (term_matrix->rows() != term_matrix->cols() ||
      term_matrix->rows() != static_cast<int>(order)) {
    term_matrix->resize(order, order);
  }
  ///生成一个x_pow向量，大小13，每个元素都是1
  std::vector<double> x_pow(2 * order + 1, 1.0);
  for (std::uint32_t i = 1; i < 2 * order + 1; ++i) {
    x_pow[i] = x_pow[i - 1] * x; ///<获得各次幂的值，从0次幂到12次幂
  }

  if (type == "fx") {
    for (std::uint32_t r = 0; r < order; ++r) {
      for (std::uint32_t c = 0; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c + 1];
      }
    }

  } else if (type == "derivative") {
    for (std::uint32_t r = 1; r < order; ++r) {
      for (std::uint32_t c = 1; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 1];
      }
    }
    (*term_matrix).block(0, 0, order, 1) = Eigen::MatrixXd::Zero(order, 1);
    (*term_matrix).block(0, 0, 1, order) = Eigen::MatrixXd::Zero(1, order);

  } else if (type == "second_order") {
    ///?二阶导数的情景，矩阵term_matrix是6*6的矩阵
    for (std::uint32_t r = 2; r < order; ++r) {///<循环2到5
      for (std::uint32_t c = 2; c < order; ++c) {///<循环2到5
        (*term_matrix)(r, c) = x_pow[r + c - 3];///?最高是7次幂，需要这么高？
        ///!答案就是，五次多项式的二阶导数是三次多项式，平方那最高就是6次幂，再积分就是7次幂了
      }
    }
    ///左上角的两列设置为0，通过bloack方法并赋值来实现，方向如下
    /*
    * block 方法用于获取或设置矩阵的一个子块，用法为：
    * block(起始行索引, 起始列索引, 子块行数, 子块列数)，列数和行数是实际的数量，不是索引不是从0开始
    * 可以想象给出两个顶点，两个顶点间的子块都设置为一个数值
    */
    (*term_matrix).block(0, 0, order, 2) = Eigen::MatrixXd::Zero(order, 2);///<也就是矩阵的左侧两列都设置为0
    (*term_matrix).block(0, 0, 2, order) = Eigen::MatrixXd::Zero(2, order);///<也就是矩阵的上面两行都设置为0

  } else {
    ///三阶导数
    for (std::uint32_t r = 3; r < order; ++r) {
      for (std::uint32_t c = 3; c < order; ++c) {
        (*term_matrix)(r, c) = x_pow[r + c - 5];///< 平方后最高是4次幂，积分后最高是5次幂
      }
    }
    (*term_matrix).block(0, 0, order, 3) = Eigen::MatrixXd::Zero(order, 3);
    (*term_matrix).block(0, 0, 3, order) = Eigen::MatrixXd::Zero(3, order);
  }
}
/// @brief 
/// @param order 
/// kernel_fx_是一个6*6的矩阵
void SplineSegKernel::calculate_fx(const std::uint32_t order) {
  kernel_fx_ = Eigen::MatrixXd::Zero(order, order);///< 6*6的矩阵,初始化为0
// kernel_fx_ 是一个6x6的矩阵，其元素通过公式 1.0 / (r + c + 1.0) 计算得出：
// * | 1.000 0.500 0.333 0.250 0.200 0.167 | /@note 1/1 1/2 1/3 1/4 1/5 1/6
// * | 0.500 0.333 0.250 0.200 0.167 0.143 | /@note 1/2 1/3 1/4 1/5 1/6 1/7
// * | 0.333 0.250 0.200 0.167 0.143 0.125 | /@note 1/3 1/4 1/5 1/6 1/7 1/8
// * | 0.250 0.200 0.167 0.143 0.125 0.111 | /@note 1/4 1/5 1/6 1/7 1/8 1/9
// * | 0.200 0.167 0.143 0.125 0.111 0.100 | /@note 1/5 1/6 1/7 1/8 1/9 1/10
// * | 0.167 0.143 0.125 0.111 0.100 0.091 | /@note 1/6 1/7 1/8 1/9 1/10 1/11
  for (int r = 0; r < kernel_fx_.rows(); ++r) { ///< 循环0到5
    for (int c = 0; c < kernel_fx_.cols(); ++c) {///< 循环0到5
      kernel_fx_(r, c) = 1.0 / (r + c + 1.0);
    }
  }
}

void SplineSegKernel::CalculateDerivative(const std::uint32_t order) {
  kernel_derivative_ = Eigen::MatrixXd::Zero(order, order);
// * | 0     0     0     0     0     0     |
// * | 0     1     1     1     1     1     |
// * | 0     1     4/3   3/2   8/5   5/3   |
// * | 0     1     3/2   9/5   2     15/7  |
// * | 0     1     8/5   2     16/7  5/2   |
// * | 0     1     2/3   15/7  5/2   25/9  |
  for (int r = 1; r < kernel_derivative_.rows(); ++r) {
    for (int c = 1; c < kernel_derivative_.cols(); ++c) {
      kernel_derivative_(r, c) = r * c / (r + c - 1.0);
    }
  }
}
/// @brief 二阶导平方后积分的矩阵，不含t，不含五次项的系数
/// @param order 
void SplineSegKernel::CalculateSecondOrderDerivative(
    const std::uint32_t order) {
  kernel_second_order_derivative_ = Eigen::MatrixXd::Zero(order, order);///<6*6的矩阵
// * | 0     0     0     0     0     0     |
// * | 0     0     0     0     0     0     |
// * | 0     0     4     6     8     10    |
// * | 0     0     6     12    18    24    |
// * | 0     0     8     18    144/5 40    |
// * | 0     0     10    24    40    400/7 |
  for (int r = 2; r < kernel_second_order_derivative_.rows(); ++r) {
    for (int c = 2; c < kernel_second_order_derivative_.cols(); ++c) {
      kernel_second_order_derivative_(r, c) =
          (r * r - r) * (c * c - c) / (r + c - 3.0);
    }
  }
}
/// @brief 三阶导平方后积分的矩阵，不含t，不含五次项的系数
/// @param order 
void SplineSegKernel::CalculateThirdOrderDerivative(const std::uint32_t order) {
  kernel_third_order_derivative_ = Eigen::MatrixXd::Zero(order, order);
// * | 0     0     0     0     0     0     |
// * | 0     0     0     0     0     0     |
// * | 0     0     0     0     0     0     |
// * | 0     0     0     36    72    120   |
// * | 0     0     0     72    192   360   |
// * | 0     0     0     120   360   720   |  
  for (int r = 3; r < kernel_third_order_derivative_.rows(); ++r) {
    for (int c = 3; c < kernel_third_order_derivative_.cols(); ++c) {
      kernel_third_order_derivative_(r, c) =
          (r * r - r) * (r - 2) * (c * c - c) * (c - 2) / (r + c - 5.0);
    }
  }
}

}  // namespace planning
}  // namespace apollo
