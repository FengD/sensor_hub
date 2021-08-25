// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: output

#pragma once

#include <string>
#include "common/common.h"
#include "common/proto/pose.pb.h"
#include "common/proto/lidar.pb.h"
#include "common/proto/eth_packet.pb.h"

namespace crdc {
namespace airi {

class LidarOutput {
 public:
  LidarOutput() = default;
  virtual ~LidarOutput() = default;
  bool init(const std::string name);
  void subscribe_pose(const std::function<void(const std::shared_ptr<Pose>&)>& reader_func);
  bool write_cloud(const std::string& topic,
                   const std::shared_ptr<PointCloud>& cloud);
  bool write_fusion_clouds(const std::string& topic,
                           const std::shared_ptr<PointClouds>& clouds);
  bool write_packet(const std::string& topic,
                    const std::shared_ptr<Packets>& clouds);
};

}  // namespace airi
}  // namespace crdc