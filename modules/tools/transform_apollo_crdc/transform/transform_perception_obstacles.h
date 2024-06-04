#pragma once

#include <unordered_map>
#include "cyber/cyber.h"
#include "cyber/sensor_proto/perception_obstacle.pb.h"
#include "cyber/lgsvl_proto/perceotion_obstacle.pb.h"
#include "cyber/lgsvl_proto/gps.pb.h"
#include "cyber/lgsvl_proto/geometry.pb.h"
#include "proto/transform_component_config.pb.h"

namespace crdc {
namespace airi {

class TransformPercptionObstacles {
 public:
  TransformPercptionObstacles() = default;
  virtual ~TransformPercptionObstacles() = default;

  bool init(const TransformComponentConfig& config);

  bool transform_perception_obstacle_to_crdc(const std::shared_ptr<apollo::perception::PerceptionObstacles> &apollo_perception_obstacles,
        std::shared_ptr<crdc::airi::PerceptionObstacles> &crdc_perception_obstacles_);

  void transform_header(const std::shared_ptr<apollo::perception::PerceptionObstacles> &apollo_perception_obstacles,
        std::shared_ptr<crdc::airi::PerceptionObstacles> &crdc_perception_obstacles_);

  void transform_perception_obstacle(apollo::perception::PerceptionObstacle* apollo_perception_obstacle,
          crdc::airi::PerceptionObstacle* crdc_perception_obstacle);

  crdc::airi::PerceptionObstacle_Type int_to_obstacle(int object_type_int);
 private:
  std::shared_ptr<crdc::airi::PerceptionObstacles> crdc_perception_obstacles_;
  std::string input_topic_;
  std::string output_topic_;
  std::shared_ptr<apollo::cyber::Writer<crdc::airi::PerceptionObstacles>> talker_;
  std::unordered_map<int, apollo::common::PointENU> gps_timestamp_position_;
  std::unordered_map<int, double> gps_timestamp_heading_;   // heading default easting 90
};

}  // namespace airi
}  // namespace crdc