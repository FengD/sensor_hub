#pragma once

#include <list>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <map>
#include "common/common.h"
#include "common/error_code.h"


#ifndef WITH_ROS2
#include "module_diagnose/proto/module_diagnose.pb.h"
#else
#include "sensor_msg/msg/module_status.hpp"
#include "sensor_msg/msg/module_status_item.hpp"
#include "sensor_msg/msg/level.hpp"
#endif

namespace crdc {
namespace airi {

#ifndef WITH_ROS2
using apollo::cyber::Node;
#else
using rclcpp::Node;
using sensor_msg::msg::ModuleStatus;
using sensor_msg::msg::Level;
using sensor_msg::msg::ModuleStatusItem;
#endif

enum ModuleType {
  SENSOR_CAMERA = 0,
  SENSOR_INS = 1,
  SENSOR_LIDAR = 2,
  SENSOR_RADAR = 3
};

struct DiagnoseInput {
  ModuleType type;
  uint32_t position_id;
  DeviceStatus error;
#ifndef WITH_ROS2
  Level level;
#else
  uint8_t level;
#endif
  std::string custom_desc;
  std::string context;
  DiagnoseInput() : position_id(0), custom_desc(""), context("") { }
  DiagnoseInput(const ModuleType& type,
                const uint32_t& position_id,
                const DeviceStatus& error,
#ifndef WITH_ROS2
                const Level& level,
#else
                const uint8_t& level,
#endif
                const std::string& custom_desc = "",
                const std::string& context = "") : type(type),
                                                   position_id(position_id),
                                                   error(error),
                                                   level(level),
                                                   custom_desc(custom_desc),
                                                   context(context) { }
};

class ModuleDiagnose : public crdc::airi::common::Thread {
 public:
  void diagnose_list(std::vector<DiagnoseInput>& inputs);
  void set_ready(const bool& ready = true);
  bool init(const std::string& frame_id, const std::string& topic);
  void set_period_usec(const uint64_t& usec);
  void stop();

 protected:
  void report();
  std::shared_ptr<Node> node_;
#ifndef WITH_ROS2
  std::shared_ptr<apollo::cyber::Writer<ModuleStatus>> writer_;
#else
  std::shared_ptr<rclcpp::Publisher<ModuleStatus>> writer_;
#endif

 private:
  ModuleDiagnose();
  ~ModuleDiagnose();
  friend class  common::Singleton<ModuleDiagnose>;
  void diagnose_single(const DiagnoseInput& input);
  void run() override;
  std::string topic_;
  std::string frame_id_;
  std::unordered_map<uint32_t, ModuleStatusItem> items_map_;
  std::mutex lock_;
  uint32_t seq_ = 0;
  uint64_t period_usec_;
  bool ready_ = false;
  bool stop_ = true;
  std::shared_ptr<ModuleStatus> status_;
  std::map<int, std::string> sensor_error_key_;
  std::map<int, std::string> drivers_error_code2name_map_;
};

}  // namespace airi
}  // namespace crdc
