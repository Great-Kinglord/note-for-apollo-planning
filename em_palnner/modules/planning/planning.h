
#ifndef MODULES_PLANNING_PLANNING_H_
#define MODULES_PLANNING_PLANNING_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/proto/planning_config.pb.h"

#include "modules/common/apollo_app.h"
#include "modules/common/status/status.h"
#include "modules/common/util/factory.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/planner/planner.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Localization
 *
 * @brief Localization module main class. It processes GPS and IMU as input,
 * to generate localization info.
 */
class Planning : public apollo::common::ApolloApp {
 public:

  std::string Name() const override;///<const修饰函数，表示函数不会修改类的成员变量

  apollo::common::Status Init() override; ///<ApolloApp类中有纯虚函数，这里进行了重写

  apollo::common::Status Start() override;

  void Stop() override;

  /**
   * @brief Plan the trajectory given current vehicle state
   * @param is_on_auto_mode whether the current system is on auto-driving mode
   */
  common::Status Plan(
      const double current_time_stamp,
      const std::vector<common::TrajectoryPoint>& stitching_trajectory);

  void RunOnce();

  common::Status InitFrame(const uint32_t sequence_num, const double time_stamp,
                           const common::TrajectoryPoint& init_adc_point);

 private:
  // Watch dog timer
  void OnTimer(const ros::TimerEvent&);

  void PublishPlanningPb(ADCTrajectory* trajectory_pb);

  void PublishPlanningPb(ADCTrajectory* trajectory_pb, double timestamp);

  void RegisterPlanners();

  apollo::common::util::Factory<PlanningConfig::PlannerType, Planner>
      planner_factory_;

  PlanningConfig config_;

  std::unique_ptr<hdmap::PncMap> pnc_map_;

  std::unique_ptr<Frame> frame_;

  std::unique_ptr<Planner> planner_;

  PublishableTrajectory last_publishable_trajectory_;

  ros::Timer timer_;
};

}  // namespace planning
}  // namespace apollo

#endif 
