#pragma once

#include "cyber/cyber.h"

#include "proto/transform_component_config.pb.h"
#include "cyber/lgsvl_proto/detection2darray.pb.h"
#include "cyber/sensor_proto/detection2darray.pb.h"

namespace crdc {
namespace airi {

class Transform2DGT {
 public:
  Transform2DGT() = default;
  virtual ~Transform2DGT() = default;

  bool init(const TransformComponentConfig& config);

  bool transform_2dgt_to_crdc(const std::shared_ptr<apollo::common::Detection2DArray> &apollo_2dgt,
        std::shared_ptr<crdc::airi::Detection2DArray> &crdc_2dgt_);

 private:
  std::shared_ptr<crdc::airi::Detection2DArray> crdc_2dgt_;
  std::string input_topic_;
  std::string output_topic_;
  std::shared_ptr<apollo::cyber::Writer<crdc::airi::Detection2DArray>> talker_;
};

}  // namespace airi
}  // namespace crdc
