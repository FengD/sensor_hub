// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: cyber_output

#pragma once

#include <string>
#include "cyber/cyber.h"
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
  bool init(const std::string& name);

  void subscribe_pose(const std::string& topic,
                      const std::function<void(const std::shared_ptr<Pose>&)>& reader_func);

  bool write_cloud(const std::shared_ptr<PointCloud>& cloud);

  bool write_fusion_clouds(const std::shared_ptr<PointClouds>& clouds);

  bool write_packet(const std::shared_ptr<Packets>& packets);

 private:
  friend class Singleton<LidarCyberOutput>;
  std::shared_ptr<apollo::cyber::Node> lidar_node_ptr_;
  std::shared_ptr<apollo::cyber::Writer<PointCloud>> cloud_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<Packets>> packets_writer_ptr_;
  std::shared_ptr<apollo::cyber::Writer<PointClouds>> fusion_clouds_writer_ptr_;
};

}  // namespace airi
}  // namespace crdc