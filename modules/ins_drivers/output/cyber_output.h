// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: cyber_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "cyber/sensor_proto/pose.pb.h"
#include "cyber/sensor_proto/ins.pb.h"
#include "cyber/sensor_proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

class InsCyberOutput {
 public:
  InsCyberOutput() = default;
  virtual ~InsCyberOutput() = default;

  /**
   * @brief Init the cyber output
   * @param the name of the node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[INS_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[INS_OUTPUT] Node name: " << node_name;
    node_ = apollo::cyber::CreateNode(node_name, "crdc");
    CHECK(node_);
    ins_data_writer_ptr_.reset(new ChannelWriter<Ins>(node_));
    packets_writer_ptr_.reset(new ChannelWriter<Packets>(node_));
    return true;
  }

  void subscribe_pose(const std::string& topic,
                      const std::function<void(const std::shared_ptr<Pose>&)>& reader_func) {
    auto pose_listener = node_->CreateReader<Pose>(topic, reader_func);
  }

  /**
   * @brief Send the ins data message by topic
   * @param topic name
   * @param proto ins data ptr
   * @return status
   */
  bool write_ins_data(const std::string& topic, const std::shared_ptr<Ins>& proto_ins_data) {
    proto_ins_data->mutable_header()->set_module_name("InsDriver");
    proto_ins_data->mutable_header()->set_timestamp_sec(
                static_cast<double>(get_now_microsecond()) / 1000000);
    return ins_data_writer_ptr_->write(topic, proto_ins_data);
  }

  /**
   * @brief Send the packets message by topic
   * @param topic name
   * @param proto packets ptr
   * @return status
   */
  bool write_packet(const std::string& topic, const std::shared_ptr<Packets>& proto_packets) {
    return packets_writer_ptr_->write(topic, proto_packets);
  }

 private:
  friend class common::Singleton<InsCyberOutput>;

  template <typename MessageT>
  class ChannelWriter {
   public:
    explicit ChannelWriter(std::shared_ptr<apollo::cyber::Node>& node) : node_(node) {}
    bool write(const std::string& topic, const std::shared_ptr<MessageT>& msg_ptr) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (writer_.find(topic) == writer_.end()) {
        writer_[topic] = node_->CreateWriter<MessageT>(topic);
        CHECK(writer_[topic]) << topic << "Writer is null";
      }
      return writer_[topic]->Write(msg_ptr);
    }

   private:
    std::unordered_map<std::string, std::shared_ptr<apollo::cyber::Writer<MessageT>>> writer_;
    std::shared_ptr<apollo::cyber::Node> node_;
    std::mutex mutex_;
  };
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<ChannelWriter<Ins>> ins_data_writer_ptr_;
  std::shared_ptr<ChannelWriter<Packets>> packets_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc
