/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/planning_base/common/util/print_debug_info.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/planning_base/gflags/planning_gflags.h"
namespace apollo {
namespace planning {

void PrintPoints::set_id(std::string id) { id_ = id; }
void PrintPoints::AddPoint(double x, double y) { points.emplace_back(x, y); }

void PrintPoints::PrintToLog() {
  if (!FLAGS_enable_print_curve) {
    return;
  }
  std::stringstream ssm;
  ssm << "print_" << id_ << ":";
  for (size_t i = 0; i < points.size(); i++) {
    ssm << std::fixed << "(" << points[i].first << ", " << points[i].second
        << ");";
  }
  AINFO << ssm.str();
}

void PrintCurves::AddPoint(std::string key, double x, double y) {
  if (!FLAGS_enable_print_curve) {
    return;
  }
  if (curve_map_.count(key) == 0) {
    curve_map_[key] = PrintPoints(key);
  }
  curve_map_[key].AddPoint(x, y);
}

void PrintCurves::AddPoint(std::string key,
                           const apollo::common::math::Vec2d& point) {
  if (!FLAGS_enable_print_curve) {
    return;
  }
  if (curve_map_.count(key) == 0) {
    curve_map_[key] = PrintPoints(key);
  }
  curve_map_[key].AddPoint(point.x(), point.y());
}

void PrintCurves::AddPoint(std::string key,
                           const std::vector<common::math::Vec2d>& points) {
  if (!FLAGS_enable_print_curve) {
    return;
  }
  for (const auto& point : points) {
    AddPoint(key, point);
  }
}
void PrintCurves::PrintToLog() {
  if (!FLAGS_enable_print_curve) {
    return;
  }
  for (auto iter = curve_map_.begin(); iter != curve_map_.end(); iter++) {
    iter->second.PrintToLog();
  }
}

}  // namespace planning
}  // namespace apollo
