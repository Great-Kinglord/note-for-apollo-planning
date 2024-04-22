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

#ifndef MODULES_PLANNING_PLANNER_EM_DECIDER_H_
#define MODULES_PLANNING_PLANNER_EM_DECIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/proto/decision.pb.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

class Decider {
 public:
  static DecisionResult MakeDecision(
      const ReferenceLineInfo &reference_line_info);

 private:
  DecisionResult &InternalMakeDecision(
      const ReferenceLineInfo &reference_line_info);
  int MakeMainStopDecision(const ReferenceLineInfo &reference_line_info);
  void MakeEStopDecision(const PathDecision &path_decision);
  void SetObjectDecisions(const PathDecision &path_decision);

 private:
  DecisionResult decision_result_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_EM_DECIDER_H_
