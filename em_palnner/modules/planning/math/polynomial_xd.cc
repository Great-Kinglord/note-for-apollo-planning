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
 * @file: polynomial_xd.cc
 **/

#include "modules/planning/math/polynomial_xd.h"

#include <iomanip>
#include <sstream>

namespace apollo {
namespace planning {

PolynomialXd::PolynomialXd(const std::uint32_t order) : params_(order, 0.0) {}

PolynomialXd::PolynomialXd(const std::vector<double>& params)
    : params_(params) {}

std::uint32_t PolynomialXd::order() const { return params_.size(); }

void PolynomialXd::SetParams(const std::vector<double>& params) {
  params_ = params;
}

const std::vector<double>& PolynomialXd::params() const { return params_; }

void PolynomialXd::DerivedFrom(const PolynomialXd& base) {
  if (base.order() <= 1) {
    params_.clear();
  } else {
    params_.resize(base.order() - 1);
    for (std::uint32_t i = 1; i < base.order(); ++i) {
      params_[i - 1] = base[i] * i;
    }
  }
}

void PolynomialXd::IntegratedFrom(const PolynomialXd& base) {
  params_.resize(base.order() + 1);
  params_[0] = 0.0;
  for (std::uint32_t i = 0; i < base.order(); ++i) {
    params_[i + 1] = base[i] / (i + 1);
  }
}

void PolynomialXd::IntegratedFrom(const PolynomialXd& base,
                                  const double intercept) {
  params_.resize(base.order() + 1);
  params_[0] = intercept;
  for (std::uint32_t i = 0; i < base.order(); ++i) {
    params_[i + 1] = base[i] / (i + 1);
  }
}

double PolynomialXd::operator()(const double value) const {
  double result = 0.0;
  for (auto rit = params_.rbegin(); rit != params_.rend(); ++rit) {
    result *= value;
    result += (*rit);
  }
  return result;
}

double PolynomialXd::operator[](const std::uint32_t index) const {
  if (index >= params_.size()) {
    return 0.0;
  } else {
    return params_[index];
  }
}

}  // namespace planning
}  // namespace apollo
