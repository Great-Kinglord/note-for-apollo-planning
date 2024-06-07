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
 * @file qp_spline_st_speed_optimizer.cc
 **/

#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/file.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_graph.h"
#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::planning_internal::STGraphDebug;

QpSplineStSpeedOptimizer::QpSplineStSpeedOptimizer()
    : SpeedOptimizer("QpSplineStSpeedOptimizer") {}

bool QpSplineStSpeedOptimizer::Init(const PlanningConfig& config) {
  qp_spline_st_speed_config_ =
      config.em_planner_config().qp_spline_st_speed_config();
  st_boundary_config_ = qp_spline_st_speed_config_.st_boundary_config();
  is_init_ = true;
  return true;
}
/*
  * @brief 思想按时间采样N段，每一段速度轨迹采用多项式拟合，整体需要满足设定的各项约束条件，求取代价最小的多项式参数
  * @param adc_sl_boundary 从reference_line_info中获取
  * @param path_data 从reference_line_info中获取，就是mutable_path_data，横向规划完的结果
  * @param init_point 从frame中获取
  * @param reference_line 从reference_line_info中获取
  * @param path_decision 从reference_line_info中获取
  * 我的理解就是DP开辟凸空间，每个障碍物确认了决策标签，所以现在才进行QP规划
*/

Status QpSplineStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                         const PathData& path_data,
                                         const TrajectoryPoint& init_point,
                                         const ReferenceLine& reference_line,
                                         PathDecision* const path_decision,
                                         SpeedData* const speed_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process.";
    return Status(ErrorCode::PLANNING_ERROR, "Not init.");
  }

  if (path_data.discretized_path().NumOfPoints() == 0) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ///建立st图
  StBoundaryMapper boundary_mapper(
      reference_line_info_->pnc_map(), adc_sl_boundary, st_boundary_config_,
      reference_line, path_data, qp_spline_st_speed_config_.total_path_length(),
      qp_spline_st_speed_config_.total_time());///< 默认长度80m，时间8s

  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    DCHECK(path_obstacle->HasLongitudinalDecision());
  }
  ///! 第一步获取边界        
  std::vector<StBoundary> boundaries;
  if (boundary_mapper.GetGraphBoundary(*path_decision, &boundaries).code() ==
      ErrorCode::PLANNING_ERROR) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Mapping obstacle for qp st speed optimizer failed!");
  }

  for (const auto& boundary : boundaries) {
    ADEBUG << "QPST mapped boundary: " << boundary.DebugString() << std::endl;
    DCHECK(boundary.boundary_type() != StBoundary::BoundaryType::UNKNOWN);
  }

  SpeedLimit speed_limits;
  if (boundary_mapper.GetSpeedLimits(&speed_limits) != Status::OK()) {///< 不同s处对应的速度限制,SL曲线这里是定了线型的
    return Status(ErrorCode::PLANNING_ERROR,
                  "GetSpeedLimits for dp st speed optimizer failed!");
  }

  // step 2 perform graph search
  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();
  QpSplineStGraph st_graph(qp_spline_st_speed_config_, veh_param);

  StGraphData st_graph_data(boundaries, init_point, speed_limits,
                            path_data.discretized_path().Length());

  STGraphDebug* st_graph_debug =
      frame_->DebugLogger()->mutable_planning_data()->add_st_graph();

  std::pair<double, double> accel_bound = {
      qp_spline_st_speed_config_.preferred_min_deceleration(),///< -1.8
      qp_spline_st_speed_config_.preferred_max_acceleration()};///< 1.2
  st_graph.SetDebugLogger(st_graph_debug);
  auto ret = st_graph.Search(st_graph_data, speed_data, accel_bound);
  ///进行了两次尝试
  if (ret != Status::OK()) {
    AERROR << "Failed to solve with ideal acceleration conditions. Use "
              "secondary choice instead.";

    accel_bound.first = qp_spline_st_speed_config_.min_deceleration(); ///< -4.5
    accel_bound.second = qp_spline_st_speed_config_.max_acceleration(); ///< 2.0
    ret = st_graph.Search(st_graph_data, speed_data, accel_bound);
    /// 放宽加速度的上下限范围
    if (ret != Status::OK()) {
      std::string msg = common::util::StrCat(
          Name(), ":Failed to search graph with quadratic programming!");
      RecordSTGraphDebug(boundaries, speed_limits, *speed_data, st_graph_debug);
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  // record debug info
  RecordSTGraphDebug(boundaries, speed_limits, *speed_data, st_graph_debug);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
