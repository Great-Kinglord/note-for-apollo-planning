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
 * @file
 **/

#include "modules/planning/tasks/traffic_decider/traffic_decider.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/tasks/traffic_decider/back_side_vehicles.h"

namespace apollo {
namespace planning {
using common::Status;
using common::VehicleConfigHelper;
/// 在对后向来车的处理决策中,会对path_decision中的每一个障碍物进行遍历,在遍历时,如果是后向车辆,则选择忽略,
/// 即通过调用函数PathDecision::AddLongitudinalDecision()和函数PathDecision::AddLateralDecision()为障碍物打上"ignore”的标签
TrafficDecider::TrafficDecider() : Task("TrafficDecider") {}
/// @brief 注册交通规则-后向来车
void TrafficDecider::RegisterRules() {
  rule_factory_.Register("BackSideVehicles", []() -> TrafficRule * {
    return new BackSideVehicles();///< 返回对象的指针
  });
}

bool TrafficDecider::Init(const PlanningConfig &config) {
  RegisterRules();
  rule_configs_ = config.rule_config(); ///< rule_config = 4
  is_init_ = true;
  return true;
}

Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
 ///当前只有一个交通规则-后向来车
  for (const auto rule_config : rule_configs_) {
    auto rule = rule_factory_.CreateObject(rule_config.name());
    if (!rule) {
      AERROR << "Could not find rule " << rule_config.DebugString();
      continue;
    }
    rule->ApplyRule(reference_line_info);
    ADEBUG << "Applied rule " << rule_config.name();
  }
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
