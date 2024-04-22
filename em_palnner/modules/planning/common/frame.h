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
 * @file frame.h
 **/

#ifndef MODULES_PLANNING_COMMON_FRAME_H_
#define MODULES_PLANNING_COMMON_FRAME_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "modules/common/proto/geometry.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/planning/proto/reference_line_smoother_config.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "modules/common/status/status.h"
#include "modules/map/pnc_map/pnc_map.h"
#include "modules/planning/common/indexed_queue.h"
#include "modules/planning/common/obstacle.h"
#include "modules/planning/common/reference_line_info.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"

namespace apollo {
namespace planning {

class Frame {
 public:
  explicit Frame(const uint32_t sequence_num);

  // functions called out of optimizers
  void SetRoutingResponse(const routing::RoutingResponse &routing);
  void SetPrediction(const prediction::PredictionObstacles &prediction);
  void SetPlanningStartPoint(const common::TrajectoryPoint &start_point);
  void SetVehicleInitPose(const localization::Pose &pose);
  const common::TrajectoryPoint &PlanningStartPoint() const;
  common::Status Init(const PlanningConfig &config,
                      const double current_time_stamp);

  static void SetMap(hdmap::PncMap *pnc_map);

  uint32_t SequenceNum() const;

  std::string DebugString() const;

  ADCTrajectory *MutableADCTrajectory();

  planning_internal::Debug *DebugLogger();

  const PublishableTrajectory &ComputedTrajectory() const;

  const routing::RoutingResponse &routing_response() const;

  void RecordInputDebug();

  std::vector<ReferenceLineInfo> &reference_line_info();

  bool AddObstacle(std::unique_ptr<Obstacle> obstacle);

  const ReferenceLineInfo *FindDriveReferenceLineInfo();
  const ReferenceLineInfo *DriveReferenceLinfInfo() const;

 private:
  /**
   * @brief This is the function that can create one reference lines
   * from routing result.
   * In current implementation, only one reference line will be returned.
   * But this is insufficient when multiple driving options exist.
   *
   * TODO create multiple reference_lines from this function.
   */
  std::vector<ReferenceLine> CreateReferenceLineFromRouting(
      const common::PointENU &position,
      const routing::RoutingResponse &routing);

  /**
   * @brief create obstacles from prediction input.
   * @param prediction the received prediction result.
   */
  void CreatePredictionObstacles(
      const prediction::PredictionObstacles &prediction);

  bool InitReferenceLineInfo(const std::vector<ReferenceLine> &reference_lines);

  void AlignPredictionTime(const double trajectory_header_time);

  /**
   * Check if there is collision with obstacles
   */
  bool CheckCollision();

  const Obstacle *AddStaticVirtualObstacle(const std::string &id,
                                           const common::math::Box2d &box);

  const Obstacle *CreateDestinationObstacle();

 private:
  common::TrajectoryPoint planning_start_point_;

  std::vector<ReferenceLineInfo> reference_line_info_;

  /**
   * the reference line info that the vehicle finally choose to drive on.
   **/
  const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

  routing::RoutingResponse routing_response_;
  prediction::PredictionObstacles prediction_;

  std::mutex obstacles_mutex_;
  IndexedObstacles obstacles_;

  uint32_t sequence_num_ = 0;
  localization::Pose init_pose_;
  static const hdmap::PncMap *pnc_map_;
  ReferenceLineSmootherConfig smoother_config_;

  ADCTrajectory trajectory_pb_;  // planning output pb

  std::string collision_obstacle_id_;
};

class FrameHistory : public IndexedQueue<uint32_t, Frame> {
 private:
  DECLARE_SINGLETON(FrameHistory);
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_COMMON_FRAME_H_
