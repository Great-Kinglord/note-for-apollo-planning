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
 * @file reference_line.cc
 **/

#include "modules/planning/reference_line/reference_line.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "boost/math/tools/minima.hpp"

#include "modules/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using MapPath = hdmap::Path;
using apollo::common::SLPoint;

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),
      map_path_(MapPath(std::vector<hdmap::MapPathPoint>(
          reference_points.begin(), reference_points.end()))) {}

ReferenceLine::ReferenceLine(const MapPath& hdmap_path)
    : map_path_(hdmap_path) {
  for (const auto& point : hdmap_path.path_points()) {
    DCHECK(!point.lane_waypoints().empty());
    const auto& lane_waypoint = point.lane_waypoints()[0];
    reference_points_.emplace_back(
        hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0,
        0.0, 0.0);
  }
}

ReferenceLine::ReferenceLine(
    const std::vector<ReferencePoint>& reference_points,
    const std::vector<hdmap::LaneSegment>& lane_segments,
    const double max_approximation_error)
    : reference_points_(reference_points),
      map_path_(MapPath(std::vector<hdmap::MapPathPoint>(
                            reference_points.begin(), reference_points.end()),
                        lane_segments, max_approximation_error)) {}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
  const auto& accumulated_s = map_path_.accumulated_s();
  if (s < accumulated_s.front()) {
    AWARN << "The requested s is before the start point of the reference "
             "line; reference line starts at "
          << accumulated_s.front() << ", requested " << s << ".";
    ReferencePoint ref_point(map_path_.GetSmoothPoint(s), 0.0, 0.0, 0.0, 0.0);
    if (ref_point.lane_waypoints().empty()) {
      ref_point.add_lane_waypoints(reference_points_.front().lane_waypoints());
    }
    return ref_point;
  }
  if (s > accumulated_s.back()) {
    AWARN << "The requested s exceeds the reference line; reference line "
             "ends at "
          << accumulated_s.back() << "requested " << s << " .";
    ReferencePoint ref_point(map_path_.GetSmoothPoint(s), 0.0, 0.0, 0.0, 0.0);
    if (ref_point.lane_waypoints().empty()) {
      ref_point.add_lane_waypoints(reference_points_.back().lane_waypoints());
    }
    return ref_point;
  }

  auto it_lower =
      std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
  if (it_lower == accumulated_s.begin()) {
    return reference_points_.front();
  } else {
    auto index = std::distance(accumulated_s.begin(), it_lower);
    const auto& p0 = reference_points_[index - 1];
    const auto& p1 = reference_points_[index];

    const double s0 = accumulated_s[index - 1];
    const double s1 = accumulated_s[index];

    return Interpolate(p0, s0, p1, s1, s);
  }
}

double ReferenceLine::FindMinDistancePoint(const ReferencePoint& p0,
                                           const double s0,
                                           const ReferencePoint& p1,
                                           const double s1, const double x,
                                           const double y) {
  auto func_dist_square = [&p0, &p1, &s0, &s1, &x, &y](const double s) {
    auto p = Interpolate(p0, s0, p1, s1, s);
    double dx = p.x() - x;
    double dy = p.y() - y;
    return dx * dx + dy * dy;
  };

  return ::boost::math::tools::brent_find_minima(func_dist_square, s0, s1, 8)
      .first;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double x,
                                                const double y) const {
  CHECK_GE(reference_points_.size(), 0);

  auto func_distance_square = [](const ReferencePoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double d_min = func_distance_square(reference_points_.front(), x, y);
  double index_min = 0;

  for (uint32_t i = 1; i < reference_points_.size(); ++i) {
    double d_temp = func_distance_square(reference_points_[i], x, y);
    if (d_temp < d_min) {
      d_min = d_temp;
      index_min = i;
    }
  }

  uint32_t index_start = (index_min == 0 ? index_min : index_min - 1);
  uint32_t index_end =
      (index_min + 1 == reference_points_.size() ? index_min : index_min + 1);

  if (index_start == index_end) {
    return reference_points_[index_start];
  }

  double s0 = map_path_.accumulated_s()[index_start];
  double s1 = map_path_.accumulated_s()[index_end];

  double s = ReferenceLine::FindMinDistancePoint(
      reference_points_[index_start], s0, reference_points_[index_end], s1, x,
      y);

  return Interpolate(reference_points_[index_start], s0,
                     reference_points_[index_end], s1, s);
}

bool ReferenceLine::SLToXY(const SLPoint& sl_point,
                           common::math::Vec2d* const xy_point) const {
  CHECK_NOTNULL(xy_point);
  if (map_path_.num_points() < 2) {
    AERROR << "The reference line has too few points.";
    return false;
  }

  const auto matched_point = GetReferencePoint(sl_point.s());
  const auto angle = common::math::Angle16::from_rad(matched_point.heading());
  xy_point->set_x(matched_point.x() - common::math::sin(angle) * sl_point.l());
  xy_point->set_y(matched_point.y() + common::math::cos(angle) * sl_point.l());
  return true;
}

bool ReferenceLine::XYToSL(const common::math::Vec2d& xy_point,
                           SLPoint* const sl_point) const {
  DCHECK_NOTNULL(sl_point);
  double s = 0;
  double l = 0;
  if (!map_path_.GetProjection(xy_point, &s, &l)) {
    AERROR << "Can't get nearest point from path.";
    return false;
  }

  sl_point->set_s(s);
  sl_point->set_l(l);
  return true;
}

ReferencePoint ReferenceLine::Interpolate(const ReferencePoint& p0,
                                          const double s0,
                                          const ReferencePoint& p1,
                                          const double s1, const double s) {
  if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
    return p0;
  }
  DCHECK_LE(s0 - 1.0e-6, s) << " s: " << s << " is less than s0 :" << s0;
  DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << "is larger than s1: " << s1;

  CHECK(!p0.lane_waypoints().empty());
  CHECK(!p1.lane_waypoints().empty());
  const double x = common::math::lerp(p0.x(), s0, p1.x(), s1, s);
  const double y = common::math::lerp(p0.y(), s0, p1.y(), s1, s);
  const double heading =
      common::math::slerp(p0.heading(), s0, p1.heading(), s1, s);
  const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
  const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);
  const auto& p0_waypoint = p0.lane_waypoints()[0];
  std::vector<hdmap::LaneWaypoint> waypoints;
  double upper_bound = 0.0;
  double lower_bound = 0.0;
  if ((s - s0) + p0_waypoint.s <= p0_waypoint.lane->total_length()) {
    const double lane_s = p0_waypoint.s + s - s0;
    waypoints.emplace_back(p0_waypoint.lane, lane_s);
    p0_waypoint.lane->GetWidth(lane_s, &upper_bound, &lower_bound);
  }
  const auto& p1_waypoint = p1.lane_waypoints()[0];
  if (p1_waypoint.lane->id().id() != p0_waypoint.lane->id().id() &&
      p1_waypoint.s - (s1 - s) >= 0) {
    const double lane_s = p1_waypoint.s - (s1 - s);
    waypoints.emplace_back(p1_waypoint.lane, lane_s);
    p1_waypoint.lane->GetWidth(lane_s, &upper_bound, &lower_bound);
  }
  if (waypoints.empty()) {
    const double lane_s = p0_waypoint.s;
    waypoints.emplace_back(p0_waypoint.lane, lane_s);
    p0_waypoint.lane->GetWidth(lane_s, &upper_bound, &lower_bound);
  }
  return ReferencePoint(hdmap::MapPathPoint({x, y}, heading, waypoints), kappa,
                        dkappa, lower_bound, upper_bound);
}

const std::vector<ReferencePoint>& ReferenceLine::reference_points() const {
  return reference_points_;
}

const MapPath& ReferenceLine::map_path() const { return map_path_; }

bool ReferenceLine::GetLaneWidth(const double s, double* const left_width,
                                 double* const right_width) const {
  return map_path_.GetWidth(s, left_width, right_width);
}

bool ReferenceLine::IsOnRoad(const SLPoint& sl_point) const {
  if (sl_point.s() <= 0 || sl_point.s() > map_path_.length()) {
    return false;
  }
  double left_width = 0.0;
  double right_width = 0.0;

  if (!GetLaneWidth(sl_point.s(), &left_width, &right_width)) {
    return false;
  }

  if (sl_point.l() <= -right_width || sl_point.l() >= left_width) {
    return false;
  }

  return true;
}

bool ReferenceLine::GetSLBoundary(const common::math::Box2d& box,
                                  SLBoundary* const sl_boundary) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());
  std::vector<common::math::Vec2d> corners;
  box.GetAllCorners(&corners);
  for (const auto& point : corners) {
    SLPoint sl_point;
    if (!XYToSL(point, &sl_point)) {
      AERROR << "failed to get projection for point: " << point.DebugString()
             << " on reference line.";
      return false;
    }
    start_s = std::fmin(start_s, sl_point.s());
    end_s = std::fmax(end_s, sl_point.s());
    start_l = std::fmin(start_l, sl_point.l());
    end_l = std::fmax(end_l, sl_point.l());
  }
  sl_boundary->set_start_s(start_s);
  sl_boundary->set_end_s(end_s);
  sl_boundary->set_start_l(start_l);
  sl_boundary->set_end_l(end_l);
  return true;
}

bool ReferenceLine::HasOverlap(const common::math::Box2d& box) const {
  SLBoundary sl_boundary;
  if (!GetSLBoundary(box, &sl_boundary)) {
    AERROR << "Failed to get sl boundary for box " << box.DebugString();
    return false;
  }
  if (sl_boundary.end_s() < 0 || sl_boundary.start_s() > Length()) {
    return false;
  }
  if (sl_boundary.start_l() * sl_boundary.end_l() < 0) {
    return false;
  }

  double left_width = 0.0;
  double right_width = 0.0;
  const double mid_s = (sl_boundary.start_s() + sl_boundary.end_s()) / 2.0;
  if (mid_s < 0 || mid_s > Length()) {
    ADEBUG << "ref_s out of range:" << mid_s;
    return false;
  }
  if (!map_path_.GetWidth(mid_s, &left_width, &right_width)) {
    AERROR << "failed to get width at s = " << mid_s;
    return false;
  }
  if (sl_boundary.start_l() > 0) {
    return sl_boundary.start_l() < left_width;
  } else {
    return sl_boundary.end_l() > -right_width;
  }
}

std::string ReferenceLine::DebugString() const {
  const auto limit =
      std::min(reference_points_.size(),
               static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));
  return apollo::common::util::StrCat(
      "point num:", reference_points_.size(),
      apollo::common::util::PrintDebugStringIter(
          reference_points_.begin(), reference_points_.begin() + limit, ""));
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const {
  const auto& map_path_point = GetReferencePoint(s);
  double speed_limit = FLAGS_planning_upper_speed_limit;
  for (const auto& lane_waypoint : map_path_point.lane_waypoints()) {
    if (lane_waypoint.lane == nullptr) {
      AWARN << "lane_waypoint.lane is nullptr";
      continue;
    }
    speed_limit =
        std::fmin(lane_waypoint.lane->lane().speed_limit(), speed_limit);
  }
  return speed_limit;
}

}  // namespace planning
}  // namespace apollo
