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

class LidarCyberMessage {
 public:
  LidarCyberMessage() = default;
  virtual ~LidarCyberMessage() = default;
  bool init(const std::string& name) {
    apollo::cyber::Init(name.c_str());
    lidar_node_ptr_ = apollo::cyber::CreateNode(name);
    cloud_writer_ptr_ = lidar_node_ptr_->CreateWriter<PointCloud>("test");
    packets_writer_ptr_ = lidar_node_ptr_->CreateWriter<Packets>("test");
    fusion_clouds_writer_ptr_ = lidar_node_ptr_->CreateWriter<PointClouds>("test");
    apollo::cyber::WaitForShutdown();
    return true;
  }

  void subscribe_pose(const std::string& topic,
                      const std::function<void(const std::shared_ptr<Pose>&)>& reader_func) {
    auto pose_listener = lidar_node_ptr_->CreateReader<Pose>(topic, reader_func);
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
  friend class common::Singleton<LidarCyberMessage>;
  std::shared_ptr<apollo::cyber::Node> lidar_node_ptr_;
  std::shared_ptr<apollo::cyber::Writer<PointCloud>> cloud_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<Packets>> packets_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<PointClouds>> fusion_clouds_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc