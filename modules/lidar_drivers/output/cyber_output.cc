// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: cyber_output

#pragma once

#include "lidar_drivers/output/cyber_output.h"

namespace crdc {
namespace airi {

bool LidarCyberOutput::init(const std::string& name) {
  apollo::cyber::Init(name.c_str());
  lidar_node_ptr_ = apollo::cyber::CreateNode(name);
  cloud_writer_ptr_ = lidar_node_ptr_->CreateWriter<PointCloud>("test");
  packets_writer_ptr_ = lidar_node_ptr_->CreateWriter<Packets>("test");
  fusion_clouds_writer_ptr_ = lidar_node_ptr_->CreateWriter<PointClouds>("test");
  apollo::cyber::WaitForShutdown();
  return true;
}

void LidarCyberOutput::subscribe_pose(const std::string& topic,
                    const std::function<void(const std::shared_ptr<Pose>&)>& reader_func) {
  auto pose_listener = lidar_node_ptr_->CreateReader<Pose>(topic, reader_func);
}

bool LidarCyberOutput::write_cloud(const std::shared_ptr<PointCloud>& cloud) {
  return cloud_writer_ptr_->Write(cloud);                
}

bool LidarCyberOutput::write_fusion_clouds(const std::shared_ptr<PointClouds>& clouds) {
  return fusion_clouds_writer_ptr_->Write(clouds);
}

bool LidarCyberOutput::write_packet(const std::shared_ptr<Packets>& packets) {
  return packets_writer_ptr_->Write(packets);
}

}  // namespace airi
}  // namespace crdc