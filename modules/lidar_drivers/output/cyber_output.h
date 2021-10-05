// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: cyber_output

#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "common/common.h"
#include "cyber/sensor_proto/pose.pb.h"
#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/sensor_proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

class LidarCyberOutput {
 public:
  LidarCyberOutput() = default;
  virtual ~LidarCyberOutput() = default;

  /**
   * @brief Init the cyber output
   * @param the name of the node
   * @return status
   */
  bool init(const std::string& name) {
    std::string node_name = name;
    auto machine_subcode = std::getenv("MACHINE_SUBCODE");
    if (!machine_subcode) {
      LOG(WARNING) << "[LIDAR_OUTPUT] MACHINE_SUBCODE is not set.";
    } else {
      node_name = name + std::string("_") + machine_subcode;
    }

    LOG(INFO) << "[LIDAR_OUTPUT] Node name: " << node_name;
    node_ = apollo::cyber::CreateNode(node_name, "crdc");
    CHECK(node_);
    cloud_writer_ptr_.reset(new ChannelWriter<PointCloud>(node_));
    packets_writer_ptr_.reset(new ChannelWriter<Packets>(node_));
    fusion_clouds_writer_ptr_ = node_->CreateWriter<PointClouds>("MERGE_CLOUD");
    return true;
  }

  void subscribe_pose(const std::string& topic,
                      const std::function<void(const std::shared_ptr<Pose>&)>& reader_func) {
    auto pose_listener = node_->CreateReader<Pose>(topic, reader_func);
  }

  /**
   * @brief Send the cloud message by topic
   * @param topic name
   * @param proto cloud ptr
   * @return status
   */
  bool write_cloud(const std::string& topic, const std::shared_ptr<PointCloud>& proto_cloud) {
    return cloud_writer_ptr_->write(topic, proto_cloud);
  }

  /**
   * @brief Send the merged cloud
   * @param the cloud list
   * @return status
   */
  bool write_fusion_clouds(const std::shared_ptr<PointClouds>& clouds) {
    return fusion_clouds_writer_ptr_->Write(clouds);
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
  friend class common::Singleton<LidarCyberOutput>;

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
  std::shared_ptr<ChannelWriter<PointCloud>> cloud_writer_ptr_;
  std::shared_ptr<ChannelWriter<Packets>> packets_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<PointClouds>> fusion_clouds_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc
