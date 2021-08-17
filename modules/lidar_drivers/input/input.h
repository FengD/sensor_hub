#pragma once

#include <memory>
#include <string>

#include "common/common.h"
#include "lidar/proto/lidar_config.pb.h"
#include "sensor_msgs/proto/lidar_data.pb.h"

namespace crdc {
namespace airi {

class LidarInput {

 public:
  LidarInput() = default;
  virtual ~LidarInput() = default;
  virtual bool init(const C& config) {
    return false;
  }

  virtual int get_data() {
    return 
  }

  virtual std::string name() const = 0;

  bool 

 protected:
  C config_;
  int 

};

REGISTER_COMPONENT(LidarInput);

}  // airi
}  // crdc