// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: cyber_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "cyber/sensor_proto/image.pb.h"

namespace crdc {
namespace airi {

class CameraCyberOutput {
 public:
  CameraCyberOutput() = default;
  virtual ~CameraCyberOutput() = default;

  /**
   * @brief Init the cyber output
   * @param the name of the node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[CAMERA_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[CAMERA_OUTPUT] Node name: " << node_name;
    node_ = apollo::cyber::CreateNode(node_name, "crdc");
    CHECK(node_);
    image_writer_ptr_.reset(new ChannelWriter<Image>(node_));
    return true;
  }

  /**
   * @brief Send the image message by topic
   * @param topic name
   * @param proto image ptr
   * @return status
   */
  bool write_image(const std::string& topic, const std::shared_ptr<Image>& proto_image) {
    return image_writer_ptr_->write(topic, proto_image);                
  }

  std::shared_ptr<apollo::cyber::Node> get_node() {
    return node_;
  }

 private:
  friend class common::Singleton<CameraCyberOutput>;

  template <typename MessageT>
  class ChannelWriter {
   public:
    explicit ChannelWriter(std::shared_ptr<apollo::cyber::Node>& node) : node_(node){}
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
  std::shared_ptr<ChannelWriter<Image>> image_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc