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
 * @file dp_st_speed_optimizer.cc
 **/

#include "modules/planning/tasks/dp_st_speed/dp_st_speed_optimizer.h"

#include <algorithm>
#include <vector>

#include "modules/planning/proto/planning_internal.pb.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"
#include "modules/planning/tasks/st_graph/st_graph_data.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::adapter::AdapterManager;
using apollo::localization::LocalizationEstimate;
using apollo::planning_internal::STGraphDebug;

DpStSpeedOptimizer::DpStSpeedOptimizer()
    : SpeedOptimizer("DpStSpeedOptimizer") {}

bool DpStSpeedOptimizer::Init(const PlanningConfig& config) {
  dp_st_speed_config_ = config.em_planner_config().dp_st_speed_config();
  st_boundary_config_ = dp_st_speed_config_.st_boundary_config();
  is_init_ = true;
  return true;
}

/// @brief 主函数
/// @param adc_sl_boundary 从reference_line_info中获取
/// @param path_data 从reference_line_info中获取，就是mutable_path_data，横向规划完的结果
/// @param init_point 从frame中获取
/// @param reference_line 从reference_line_info中获取
/// @param path_decision 从reference_line_info中获取
/// @param speed_data 从reference_line_info中获取，意思就是最终也是放到参考线中
/// @return 
Status DpStSpeedOptimizer::Process(const SLBoundary& adc_sl_boundary,
                                   const PathData& path_data,
                                   const common::TrajectoryPoint& init_point,
                                   const ReferenceLine& reference_line,
                                   PathDecision* const path_decision,
                                   SpeedData* const speed_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before process DpStSpeedOptimizer.";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  /// 判断有没有路径生成
  if (path_data.discretized_path().NumOfPoints() == 0) {
    std::string msg("Empty path data");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ///应该就是栅格地图，pnc_map就是routing的结果，adc_sl_boundary就是自车的SL的边界
  StBoundaryMapper boundary_mapper(
      reference_line_info_->pnc_map(), adc_sl_boundary, st_boundary_config_,
      reference_line, path_data, dp_st_speed_config_.total_path_length(),
      dp_st_speed_config_.total_time()); ///? total_path_length默认0.1，total_time默认3.0

  /// step 1 get boundaries 第一步获取边界，考虑了障碍物
  std::vector<StBoundary> boundaries;
  if (boundary_mapper.GetGraphBoundary(*path_decision, &boundaries).code() ==
      ErrorCode::PLANNING_ERROR) {
    const std::string msg =
        "Mapping obstacle for dp st speed optimizer failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // step 2 开始DP去搜索了
  SpeedLimit speed_limit;
  if (!boundary_mapper.GetSpeedLimits(&speed_limit).ok()) {
    const std::string msg =
        "Getting speed limits for dp st speed optimizer failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  const double path_length = path_data.discretized_path().Length();
  StGraphData st_graph_data(boundaries, init_point, speed_limit, path_length);

  DpStGraph st_graph(reference_line, st_graph_data, dp_st_speed_config_,
                     path_data, adc_sl_boundary);
  auto* debug = frame_->DebugLogger();
  STGraphDebug* st_graph_debug = debug->mutable_planning_data()->add_st_graph();
  ///开始DP搜索了
  if (!st_graph.Search(path_decision, speed_data).ok()) {
    const std::string msg(Name() +
                          ":Failed to search graph with dynamic programming.");
    AERROR << msg;
    RecordSTGraphDebug(boundaries, speed_limit, *speed_data, st_graph_debug);
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  RecordSTGraphDebug(boundaries, speed_limit, *speed_data, st_graph_debug);

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
