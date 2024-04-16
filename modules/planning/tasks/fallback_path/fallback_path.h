/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include <vector>
#include "modules/planning/tasks/fallback_path/proto/fallback_path.pb.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"

namespace apollo {
namespace planning {

class FallbackPath : public PathGeneration {
 public:
  bool Init(const std::string& config_dir, const std::string& name,
            const std::shared_ptr<DependencyInjector>& injector) override;

 private:
  apollo::common::Status Process(
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

  /**
   * @brief Calculate all path boundaries
   * @param boundary is calculated path boundaries
   */
  bool DecidePathBounds(std::vector<PathBoundary>* boundary);

  /**
   * @brief Optimize paths for each path boundary
   * @param path_boundaries is input path boundaries
   * @param candidate_path_data is output paths
   */
  bool OptimizePath(const std::vector<PathBoundary>& path_boundaries,
                    std::vector<PathData>* candidate_path_data);

  /**
   * @brief Assess the feasibility of each path and select the best one
   * @param candidate_path_data is input paths
   * @param final_path is output the best path
   */
  bool AssessPath(std::vector<PathData>* candidate_path_data,
                  PathData* final_path);

  FallbackPathConfig config_;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::FallbackPath, Task)
}  // namespace planning
}  // namespace apollo
