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
 */

#ifndef MODULES_ADAPTERS_ADAPTER_MANAGER_H_
#define MODULES_ADAPTERS_ADAPTER_MANAGER_H_

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "modules/common/adapters/adapter.h"
#include "modules/common/adapters/message_adapters.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/log.h"
#include "modules/common/macro.h"

#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::common::adapter
 * @brief apollo::common::adapter
 */
namespace apollo {
namespace common {
namespace adapter {

/// Macro to prepare all the necessary adapter functions when adding a
/// new input/output. For example when you want to listen to
/// car_status message for your module, you can do
/// REGISTER_ADAPTER(CarStatus) write an adapter class called
/// CarStatusAdapter, and call EnableCarStatus(`car_status_topic`,
/// true, `callback`(if there's one)) in AdapterManager.
#define REGISTER_ADAPTER(name)                                                 \
 public:                                                                       \
  static void Enable##name(const std::string &topic_name,                      \
                           AdapterConfig::Mode mode,                           \
                           int message_history_limit) {                        \
    CHECK(message_history_limit > 0)                                           \
        << "Message history limit must be greater than 0";                     \
    instance()->InternalEnable##name(topic_name, mode, message_history_limit); \
  }                                                                            \
  static name##Adapter *Get##name() {                                          \
    return instance()->InternalGet##name();                                    \
  }                                                                            \
  static bool Feed##name##File(const std::string &proto_file) {                \
    if (!instance()->name##_) {                                                \
      AERROR << "Initialize adapter before feeding protobuf";                  \
      return false;                                                            \
    }                                                                          \
    return Get##name()->FeedFile(proto_file);                                  \
  }                                                                            \
  static void Publish##name(const name##Adapter::DataType &data) {             \
    instance()->InternalPublish##name(data);                                   \
  }                                                                            \
  template <typename T>                                                        \
  static void Fill##name##Header(const std::string &module_name, T *data) {    \
    static_assert(std::is_same<name##Adapter::DataType, T>::value,             \
                  "Data type must be the same with adapter's type!");          \
    instance()->name##_->FillHeader(module_name, data);                        \
  }                                                                            \
  static void Add##name##Callback(name##Adapter::Callback callback) {          \
    CHECK(instance()->name##_)                                                 \
        << "Initialize adapter before setting callback";                       \
    instance()->name##_->AddCallback(callback);                                \
  }                                                                            \
  template <class T>                                                           \
  static void Add##name##Callback(                                             \
      void (T::*fp)(const name##Adapter::DataType &data), T *obj) {            \
    Add##name##Callback(std::bind(fp, obj, std::placeholders::_1));            \
  }                                                                            \
                                                                               \
 private:                                                                      \
  std::unique_ptr<name##Adapter> name##_;                                      \
  ros::Publisher name##publisher_;                                             \
  ros::Subscriber name##subscriber_;                                           \
                                                                               \
  void InternalEnable##name(const std::string &topic_name,                     \
                            AdapterConfig::Mode mode,                          \
                            int message_history_limit) {                       \
    name##_.reset(                                                             \
        new name##Adapter(#name, topic_name, message_history_limit));          \
    if (mode != AdapterConfig::PUBLISH_ONLY && node_handle_) {                 \
      name##subscriber_ =                                                      \
          node_handle_->subscribe(topic_name, message_history_limit,           \
                                  &name##Adapter::OnReceive, name##_.get());   \
    }                                                                          \
    if (mode != AdapterConfig::RECEIVE_ONLY && node_handle_) {                 \
      name##publisher_ = node_handle_->advertise<name##Adapter::DataType>(     \
          topic_name, message_history_limit);                                  \
    }                                                                          \
                                                                               \
    observers_.push_back([this]() { name##_->Observe(); });                    \
  }                                                                            \
  name##Adapter *InternalGet##name() { return name##_.get(); }                 \
  void InternalPublish##name(const name##Adapter::DataType &data) {            \
    /* Only publish ROS msg if node handle is initialized. */                  \
    if (node_handle_) {                                                        \
      name##publisher_.publish(data);                                          \
    }                                                                          \
    name##_->SetLatestPublished(data);                                         \
  }

/**
 * @class AdapterManager
 *
 * @brief this class hosts all the specific adapters and manages them.
 * It provides APIs for the users to initialize, access and interact
 * with the adapters that they are interested in.
 *
 * \par
 * Each (potentially) useful adapter needs to be registered here with
 * the macro REGISTER_ADAPTER.
 *
 * \par
 * The AdapterManager is a singleton.
 */
class AdapterManager {
 public:
  /**
   * @brief Initialize the /class AdapterManager singleton with the
   * provided configuration. The configuration is specified by the
   * file path.
   * @param adapter_config_filename the path to the proto file that
   * contains the adapter manager configuration.
   */
  static void Init(const std::string &adapter_config_filename);

  /**
   * @brief Initialize the /class AdapterManager singleton with the
   * provided configuration.
   * @param configs the adapter manager configuration proto.
   */
  static void Init(const AdapterManagerConfig &configs);

  /**
   * @brief check if the AdapterManager is initialized
   */
  static bool Initialized();

  static void Observe();

  /**
   * @brief create a timer which will call a callback at the specified
   * rate. It takes a class member function, and a bare pointer to the
   * object to call the method on.
   */
  template <class T>
  static ros::Timer CreateTimer(ros::Duration period,
                                void (T::*callback)(const ros::TimerEvent &),
                                T *obj, bool oneshot = false,
                                bool autostart = true) {
    CHECK(instance()->node_handle_)
        << "ROS node is only available in ROS mode, "
           "check your adapter config file!";
    return instance()->node_handle_->createTimer(period, callback, obj, oneshot,
                                                 autostart);
  }

 private:
  /// The node handler of ROS, owned by the /class AdapterManager
  /// singleton.
  std::unique_ptr<ros::NodeHandle> node_handle_;

  /// Observe() callbacks that will be used to to call the Observe()
  /// of enabled adapters.
  std::vector<std::function<void()>> observers_;

  bool initialized_ = false;

  /// The following code registered all the adapters of interest.
  REGISTER_ADAPTER(Chassis);
  REGISTER_ADAPTER(ChassisDetail);
  REGISTER_ADAPTER(ControlCommand);
  REGISTER_ADAPTER(Gps);
  REGISTER_ADAPTER(Imu);
  REGISTER_ADAPTER(Camera);
  REGISTER_ADAPTER(Localization);
  REGISTER_ADAPTER(Monitor);
  REGISTER_ADAPTER(Pad);
  REGISTER_ADAPTER(PerceptionObstacles);
  REGISTER_ADAPTER(Planning);
  REGISTER_ADAPTER(PointCloud);
  REGISTER_ADAPTER(Prediction);
  REGISTER_ADAPTER(TrafficLightDetection);
  REGISTER_ADAPTER(RoutingRequest);
  REGISTER_ADAPTER(RoutingResponse);
  REGISTER_ADAPTER(RelativeOdometry);
  REGISTER_ADAPTER(InsStat);
  REGISTER_ADAPTER(HMICommand);

  DECLARE_SINGLETON(AdapterManager);
};

}  // namespace adapter
}  // namespace common
}  // namespace apollo

#endif
