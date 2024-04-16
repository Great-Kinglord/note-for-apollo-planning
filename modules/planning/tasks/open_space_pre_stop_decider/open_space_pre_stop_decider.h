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

/**
 * @file
 **/

#pragma once

#include <memory>
#include <string>

#include "modules/planning/tasks/open_space_pre_stop_decider/proto/open_space_pre_stop_decider.pb.h"
#include "cyber/common/macros.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_interface_base/task_base/common/decider.h"

namespace apollo {
namespace planning {

class OpenSpacePreStopDecider : public Decider {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  bool CheckParkingSpotPreStop(Frame* const frame,
                               ReferenceLineInfo* const reference_line_info,
                               double* target_s);

  bool CheckPullOverPreStop(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info,
                            double* target_s);

  void SetParkingSpotStopFence(const double target_s, Frame* const frame,
                               ReferenceLineInfo* const reference_line_info);

  void SetPullOverStopFence(const double target_s, Frame* const frame,
                            ReferenceLineInfo* const reference_line_info);

 private:
  OpenSpacePreStopDeciderConfig config_;
  static constexpr const char* OPEN_SPACE_STOP_ID = "OPEN_SPACE_PRE_STOP";
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::OpenSpacePreStopDecider,
                                     Task)

}  // namespace planning
}  // namespace apollo
