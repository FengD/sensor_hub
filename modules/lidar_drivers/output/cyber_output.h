// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: cyber_output

#pragma once

#include <string>
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

    cloud_writer_ptr_ = node_->CreateWriter<PointCloud>("lidar/cloud");
    packets_writer_ptr_ = node_->CreateWriter<Packets>("lidar/packets");
    fusion_clouds_writer_ptr_ = node_->CreateWriter<PointClouds>("lidar/clouds");
    return true;
  }

  void subscribe_pose(const std::string& topic,
                      const std::function<void(const std::shared_ptr<Pose>&)>& reader_func) {
    auto pose_listener = node_->CreateReader<Pose>(topic, reader_func);
  }
  bool write_cloud(const std::shared_ptr<PointCloud>& cloud) {
    return cloud_writer_ptr_->Write(cloud);                
  }

  bool write_fusion_clouds(const std::shared_ptr<PointClouds>& clouds) {
    return fusion_clouds_writer_ptr_->Write(clouds);
  }

  bool write_packet(const std::shared_ptr<Packets>& packets) {
    return packets_writer_ptr_->Write(packets);
  }

 private:
  friend class common::Singleton<LidarCyberOutput>;
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<apollo::cyber::Writer<PointCloud>> cloud_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<Packets>> packets_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<PointClouds>> fusion_clouds_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc