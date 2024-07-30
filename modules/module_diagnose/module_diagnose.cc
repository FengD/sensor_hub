#include "module_diagnose/module_diagnose.h"

#include <gflags/gflags.h>

#include <utility>

namespace crdc {
namespace airi {

ModuleDiagnose::ModuleDiagnose(): crdc::airi::common::Thread(true) {
  sensor_error_key_ = {{-50, "LIDAR_PACKET_LOSS"},
                       {-34, "CAMERA_IMAGE_BLUR"},
                       {-33, "CAMERA_IMAGE_DARK"},
                       {-32, "CAMERA_STRONG_BACKLIGHT"},
                       {-31, "CAMERA_WINDOW_DIRTY_BLOCKED"},
                       {-30, "CAMERA_DATA_GREEN"},
                       {-28, "FAILED_GET_CYBER_MESSAGE"},
                       {-27, "NON_BLOCKERR"},
                       {-26, "UNKOWN_HOST_ERROR"},
                       {-25, "CONNECT_ERROR"},
                       {-24, "ERR_BIND"},
                       {-23, "ERR_INTERFACE"},
                       {-22, "ERR_IO"},
                       {-17, "MSG_SEND_ERROR"},
                       {-16, "MSG_RECEIVE_ERROR"},
                       {-15, "DEVICE_ERROR"},
                       {-14, "TIMEOUT_ERROR"},
                       {-13, "POLL_ERROR"},
                       {-12, "SOCKET_SETOPT_ERROR"},
                       {-11, "SOCKET_BIND_ERROR"},
                       {-10, "SOCKET_OPEN_ERROR"},
                       {-9, "IP_EMPTY_ERROR"},
                       {-8, "GET_RAW_PACKET_ERROR"},
                       {-7, "POLL_INPUT_ERROR"},
                       {-6, "POLL_TIMEOUT"},
                       {-5, "POLL_FAILED_ERROR"},
                       {-4, "NON_BLOCK_ERROR"},
                       {-3, "BIND_SOCKET_ERROR"},
                       {-2, "CREATE_SOCKET_ERROR"},
                       {-1, "CAMERA_TIMEOUT"},
                       {0, "NO_ERROR"},
                       {1, "SUCCESS"}};
  drivers_error_code2name_map_ = sensor_error_key_;
}
ModuleDiagnose::~ModuleDiagnose() {}

void ModuleDiagnose::set_ready(const bool& ready) {
  std::lock_guard<std::mutex> lock(lock_);
  ready_ = ready;
}

bool ModuleDiagnose::init(const std::string& frame_id, const std::string& topic) {
  frame_id_ = frame_id;
  topic_ = topic;
#ifndef WITH_ROS2
  node_ = apollo::cyber::CreateNode(topic_);
#else
  node_ = std::make_shared<rclcpp::Node>(topic_);
#endif
  CHECK(node_);
#ifndef WITH_ROS2
  writer_ = node_->CreateWriter<ModuleStatus>(topic_);
#else
  writer_ = node_->create_publisher<ModuleStatus>(topic_, rclcpp::QoS(10));
#endif
  CHECK(writer_);
  status_ = std::make_shared<ModuleStatus>();
  stop_ = false;
  period_usec_ = 100000;
  return true;
}

void ModuleDiagnose::set_period_usec(const uint64_t& usec) {
  period_usec_ = usec;
}

void ModuleDiagnose::diagnose_single(const DiagnoseInput& input) {
  ModuleStatusItem item;
  uint32_t item_key = static_cast<uint32_t>(input.type) * 1000
                      + input.position_id * 100
                      + static_cast<uint32_t>(input.error);
  uint32_t error_key = static_cast<uint32_t>(input.error);
  if (sensor_error_key_.find(error_key) != sensor_error_key_.end()) {
#ifndef WITH_ROS2
    item.set_key(sensor_error_key_[error_key]);
    item.set_code(item_key);
#else
    item.set__key(sensor_error_key_[error_key]);
    item.set__code(item_key);
#endif
  } else {
#ifndef WITH_ROS2
    item.set_key("");
    item.set_code(0);
#else
    item.set__key("");
    item.set__code(0);
#endif
  }
#ifndef WITH_ROS2
  item.set_level(input.level);
#else
  item.set__level(input.level);
#endif
  if (input.custom_desc.empty()) {
    if (input.level != Level::NOERR) {
#ifndef WITH_ROS2
      item.set_desc(drivers_error_code2name_map_[error_key]);
#else
      item.set__desc(drivers_error_code2name_map_[error_key]);
#endif
    } else {
#ifndef WITH_ROS2
      item.set_desc("OK");
#else
      item.set__desc("OK");
#endif
    }
  } else {
#ifndef WITH_ROS2
    item.set_desc(input.custom_desc);
#else
    item.set__desc(input.custom_desc);
#endif
  }
  if (!input.context.empty()) {
#ifndef WITH_ROS2
    item.set_context(input.context);
#else
    item.set__context(input.context);
#endif
  }
  items_map_[item_key] = item;
}

void ModuleDiagnose::diagnose_list(std::vector<DiagnoseInput>& inputs) {
  std::lock_guard<std::mutex> lock(lock_);
  for (const auto& input : inputs) {
    diagnose_single(input);
  }
  inputs.clear();
}

void ModuleDiagnose::report() {
#ifndef WITH_ROS2
  status_->clear_items();
  status_->clear_header();
  auto header = status_->mutable_header();
  header->set_frame_id(frame_id_);
  header->set_seq(++seq_);
  auto now = get_now_microsecond();
  header->mutable_stamp()->set_secs(now / 1000000);
  header->mutable_stamp()->set_nsecs((now % 1000000) * 1000);
#else
  status_->items.clear();
  status_->header.set__frame_id(frame_id_)
                 .set__seq(++seq_);
  auto now = get_now_microsecond();
  status_->header.stamp.set__secs(now / 1000000)
                       .set__nsecs((now % 1000000) * 1000);
#endif
  std::list<ModuleStatusItem> items;
  {
    std::lock_guard<std::mutex> lock(lock_);
    for (const auto& item : items_map_) {
      items.emplace_back(item.second);
    }
    items_map_.clear();
  }
#ifndef WITH_ROS2
  *(status_->mutable_items()) = {items.begin(), items.end()};
#else
  status_->items.assign(items.begin(), items.end());
#endif
  auto max_level = Level::NOERR;
#ifndef WITH_ROS2
  for (auto item : status_->items()) {
    max_level = static_cast<int>(item.level()) > static_cast<int>(max_level) ?
                item.level() : max_level;
  }
  status_->set_ready(ready_);
#else
  for (auto item : status_->items) {
    max_level = static_cast<int>(item.level) > static_cast<int>(max_level) ?
                item.level : max_level;
  }
  status_->set__ready(ready_);
#endif

  if (!ready_) {
#ifndef WITH_ROS2
    status_->set_level(Level::DEADLY);
#else
    status_->set__level(Level::DEADLY);
#endif
  } else {
#ifndef WITH_ROS2
    status_->set_level(max_level);
#else
    status_->set__level(max_level);

#endif
  }
#ifndef WITH_ROS2
  writer_->Write(status_);
#else
  writer_->publish(*status_);
#endif
}

void ModuleDiagnose::run() {
  while (!stop_) {
    std::this_thread::sleep_for(std::chrono::microseconds(period_usec_));
    report();
  }
}

void ModuleDiagnose::stop() {
  stop_ = true;
}

}  // namespace airi
}  // namespace crdc
