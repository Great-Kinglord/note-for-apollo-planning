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
 * @file dp_road_graph.h
 **/

#include "modules/planning/tasks/dp_poly_path/dp_road_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "modules/planning/math/double.h"
#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLine &reference_line,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_(reference_line),
      speed_data_(speed_data) {}

bool DPRoadGraph::FindPathTunnel(
    const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    PathData *const path_data) {
  CHECK_NOTNULL(path_data);
  init_point_ = init_point;
  /// 把车辆起始的位置转换为SL坐标系
  if (!reference_line_.XYToSL(
          {init_point_.path_point().x(), init_point_.path_point().y()},
          &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }
  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  ///上面步骤完成了寻找最小cost的path
  std::vector<common::FrenetFramePoint> frenet_path;
  double accumulated_s = init_sl_point_.s();
  const double path_resolution = config_.path_resolution();///< 0.1
  ///每一段曲线，以0.1去采样
  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];
    ///！要记得五次多项式，不是根据实际s_pre，和s之间的曲线，而是平移到0到s-s_pre上，对于l是没有影响的
    const double path_length = cur_node.sl_point.s() - prev_node.sl_point.s();///<这两点的s距离
    double current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (Double::Compare(current_s, path_length) < 0.0) {///<从小于这段长度去采样
      const double l = curve.Evaluate(0, current_s);
      const double dl = curve.Evaluate(1, current_s);
      const double ddl = curve.Evaluate(2, current_s);
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);///!这里就能对应上了
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += path_resolution;
    }
    accumulated_s += path_length;
  }
  FrenetFramePath tunnel(frenet_path);///<frenet_path中已经包含了上面找出的路径
  path_data->SetReferenceLine(&reference_line_);///<把参考线放进来
  path_data->SetFrenetPath(tunnel);///<把tunnel放进来
  return true;
}

/// @brief  获取当前参考线下最优的前进路线
/// @param obstacles 
/// @param min_cost_path 
/// @return 
bool DPRoadGraph::GenerateMinCostPath(
    const std::vector<const PathObstacle *> &obstacles,
    std::vector<DPRoadGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);

  std::vector<std::vector<common::SLPoint>> path_waypoints;
  if (!SamplePathWaypoints(init_point_, &path_waypoints)) { ///< 采样路径点
    AERROR << "Fail to sample path waypoints!";
    return false;
  }
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<common::SLPoint>{init_sl_point_});///< 插入起始点

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();

  TrajectoryCost trajectory_cost(config_, reference_line_, obstacles,
                                 vehicle_config.vehicle_param(), speed_data_);
  ///为DP做准备
  std::vector<std::vector<DPRoadGraphNode>> graph_nodes(path_waypoints.size());///< 二维vector，最外层的size为9应该
  graph_nodes[0].emplace_back(init_sl_point_, nullptr, 0.0); ///< 第一个点，也就是车辆的起始点,第一层的vector是一维
  /// 应该是从1到8，0已经在上面搞完了
  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes[level - 1];///<前一个快，就是一个s对应多个l
    const auto &level_points = path_waypoints[level]; ///< 当前块，上一个s对应多个l
    ///遍历当前块的每一个点，应该是循环9次
    for (const auto &cur_point : level_points) {///< 遍历当前块的每一个点，也就是每一个l
      graph_nodes[level].emplace_back(cur_point, nullptr); ///< 这里的nullptr是一个指针
      auto &cur_node = graph_nodes[level].back();
      ///除了init，上一层中的9维，上一层和当前层一一连接
      for (const auto &prev_dp_node : prev_dp_nodes) {
        const auto &prev_sl_point = prev_dp_node.sl_point;
        ///两个点之间使用五次多项式去连接，认为一阶导数和二阶导数都是0
        QuinticPolynomialCurve1d curve(prev_sl_point.l(), 0.0, 0.0,
                                       cur_point.l(), 0.0, 0.0,
                                       cur_point.s() - prev_sl_point.s());
        const double cost =
            trajectory_cost.Calculate(curve, prev_sl_point.s(), cur_point.s()) +
            prev_dp_node.min_cost;///< 上一次mincost加上当前的cost
        cur_node.UpdateCost(&prev_dp_node, curve, cost);
      }
    }
  }

  ///找最优的路径，cost最小，可以指向前一个节点的指针一层层找回去
  DPRoadGraphNode fake_head;
  ///最后一层中遍历所有的节点，找到最小的cost的节点信息给到fake_head
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                         cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
  ///只要指针不为空就一直往前找
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);///< ->操作符用于通过指针访问对象的成员
  }
  ///使用std::reverse函数将min_cost_path中的元素顺序反转
  std::reverse(min_cost_path->begin(), min_cost_path->end());
  return true;
}
/// @brief 进行路径的采样
/// @param init_point 
/// @param points 
/// @return 
bool DPRoadGraph::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK(points != nullptr);

  common::math::Vec2d init_cartesian_point(init_point.path_point().x(),
                                           init_point.path_point().y()); ///< 车辆起始位置
  common::SLPoint init_sl_point;
  if (!reference_line_.XYToSL(init_cartesian_point, &init_sl_point)) {
    AERROR << "Failed to get sl point from point "
           << init_cartesian_point.DebugString();
    return false;
  }

  const double reference_line_length =
      reference_line_.map_path().accumulated_s().back(); ///< 参考线长度

  double level_distance =
      std::fmax(config_.step_length_min(),
                std::fmin(init_point.v(), config_.step_length_max())); ///< 8.0 min(spd,15) 不低于车辆1s中的行驶距离

  double accumulated_s = init_sl_point.s();
  for (std::size_t i = 0;
       i < config_.sample_level() && accumulated_s < reference_line_length;
       ++i) {
      /// sample_level默认为8
    std::vector<common::SLPoint> level_points;
    accumulated_s += level_distance;
    double s = std::fmin(accumulated_s, reference_line_length);

    int32_t num =
        static_cast<int32_t>(config_.sample_points_num_each_level() / 2); ///< 9/2=4

    for (int32_t j = -num; j < num + 1; ++j) {/// -4, -3, -2, -1, 0, 1, 2, 3, 4
      double l = config_.lateral_sample_offset() * j; ///< 0.5*j   -2~2
      auto sl = common::util::MakeSLPoint(s, l);
      if (reference_line_.IsOnRoad(sl)) {
        level_points.push_back(sl);
      }
    }
    if (!level_points.empty()) {
      points->push_back(level_points);
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
