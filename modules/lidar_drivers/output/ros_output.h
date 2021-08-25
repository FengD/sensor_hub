// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ros_output

#pragma once

#include "lidar_drivers/output/output.h"

namespace crdc {
namespace airi {

class RosOutput : public LidarOutput {
 public:
  RosOutput() = default;
  virtual ~RosOutput() = default;
  bool init(const std::string name) {
    return true;
  }

  void subscribe_pose(const std::function<void(const std::shared_ptr<Pose>&)>& reader_func) {

  }

  bool write_cloud(const std::string& topic,
                   const std::shared_ptr<PointCloud>& cloud) {
    return true;                
  }

  bool write_fusion_clouds(const std::string& topic,
                           const std::shared_ptr<PointClouds>& clouds) {
    return true;
  }

  bool write_packet(const std::string& topic,
                    const std::shared_ptr<Packets>& clouds) {
    return true;
  }

 private:
  friend class Singleton<RosOutput>;
};

}  // namespace airi
}  // namespace crdc