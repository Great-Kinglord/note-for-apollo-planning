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

#include "modules/planning/math/double.h"

#include <cmath>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

Double::Double(const double value) : value_(value) {
  CHECK(!std::isnan(value));
}

double Double::Value() const { return value_; }

int Double::Compare(const double d1, const double d2, const double epsilon) {
  CHECK(!std::isnan(d1));
  CHECK(!std::isnan(d2));

  if (DefinitelyGreaterThan(d1, d2, epsilon)) {
    return 1;
  } else if (DefinitelyLessThan(d1, d2, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::Compare(const double d1, const double d2) {
  return Compare(d1, d2, kEpsilon_);
}

int Double::Compare(const Double& d1, const Double& d2, const double epsilon) {
  return Compare(d1.Value(), d2.Value(), epsilon);
}

int Double::Compare(const Double& d1, const Double& d2) {
  return Compare(d1.Value(), d2.Value());
}

int Double::CompareTo(const double d1, const double epsilon) const {
  CHECK(!std::isnan(d1));
  if (DefinitelyGreaterThan(value_, d1, epsilon)) {
    return 1;
  } else if (DefinitelyLessThan(value_, d1, epsilon)) {
    return -1;
  } else {
    return 0;
  }
}

int Double::CompareTo(const double d1) const {
  return CompareTo(d1, kEpsilon_);
}

int Double::CompareTo(const Double& d1, const double epsilon) const {
  return CompareTo(d1.Value(), epsilon);
}

int Double::CompareTo(const Double& d1) const {
  return CompareTo(d1.Value(), kEpsilon_);
}

Double& Double::operator=(const Double& other) {
  value_ = other.Value();
  return *this;
}

Double Double::operator+(const Double& other) const {
  return Double(value_ + other.Value());
}

Double Double::operator-(const Double& other) const {
  return Double(value_ - other.Value());
}

Double Double::operator*(const Double& other) const {
  return Double(value_ * other.Value());
}

Double Double::operator/(const Double& other) const {
  return Double(value_ / other.Value());
}

Double& Double::operator+=(const Double& other) {
  value_ += other.Value();
  return *this;
}

Double& Double::operator-=(const Double& other) {
  value_ -= other.Value();
  return *this;
}

Double& Double::operator*=(const Double& other) {
  value_ *= other.Value();
  return *this;
}

Double& Double::operator/=(const Double& other) {
  value_ /= other.Value();
  return *this;
}

bool Double::operator>(const Double& other) const {
  return DefinitelyGreaterThan(value_, other.Value(), kEpsilon_);
}

bool Double::operator>=(const Double& other) const {
  return !((*this) < other);
}

bool Double::operator<(const Double& other) const {
  return DefinitelyLessThan(value_, other.Value(), kEpsilon_);
}

bool Double::operator<=(const Double& other) const {
  return !((*this) > other);
}

bool Double::operator==(const Double& other) const {
  return EssentiallyEqual(value_, other.Value(), kEpsilon_);
}

bool Double::ApproximatelyEqual(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::EssentiallyEqual(double a, double b, double epsilon) {
  return std::fabs(a - b) <= std::fmin(std::fabs(a), std::fabs(b)) * epsilon;
}
/**更加精确的比较方法
 * 常用方法是直接比较 a 和 b 的差的绝对值与 epsilon 的大小确实是一种常见的浮点数比较方法，但这种方法在处理极大或极小的浮点数时可能会出现问题。
 * 当 a 和 b 的值非常大时，由于浮点数的精度限制，它们之间的差可能会被舍入到一个比实际值大得多的值。这时，即使 a 和 b 实际上非常接近，
 * 直接比较它们的差和 epsilon 也可能会认为它们不相等。
 * 同样，当 a 和 b 的值非常小（接近于机器 epsilon）时，它们的差可能会被舍入到 0。这时，即使 a 和 b 实际上有较大的差距，
 * 直接比较它们的差和 epsilon 也可能会错误地认为它们相等。
 * 因此，这里的方法通过将 epsilon 与 a 和 b 的绝对值相乘，将 epsilon 缩放到一个与 a 和 b 的大小相当的范围，
 * 然后再进行比较。这样可以避免上述问题，使得比较结果更加精确。
*/
bool Double::DefinitelyGreaterThan(double a, double b, double epsilon) {
  return (a - b) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

bool Double::DefinitelyLessThan(double a, double b, double epsilon) {
  return (b - a) > std::fmax(std::fabs(a), std::fabs(b)) * epsilon;
}

}  // namespace planning
}  // namespace apollo
