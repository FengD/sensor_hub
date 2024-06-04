#include <functional>
#include "transform/transform_perception_obstacles.h"
#include "cyber/cyber.h"
#include "math.h"

namespace crdc {
namespace airi {

void TransformPercptionObstacles::transform_header(const std::shared_ptr<apollo::perception::PerceptionObstacles>
      &apollo_perception_obstacles,
      std::shared_ptr<crdc::airi::PerceptionObstacles> &crdc_perception_obstacles_) {
  crdc_perception_obstacles_->mutable_header()->set_timestamp_sec(apollo_perception_obstacles->mutable_header()->timestamp_sec());
  crdc_perception_obstacles_->mutable_header()->set_sequence_num(apollo_perception_obstacles->mutable_header()->sequence_num());
  crdc_perception_obstacles_->mutable_header()->set_lidar_timestamp(apollo_perception_obstacles->mutable_header()->lidar_timestamp());
  crdc_perception_obstacles_->mutable_header()->set_camera_timestamp(apollo_perception_obstacles->mutable_header()->camera_timestamp());
  crdc_perception_obstacles_->mutable_header()->set_radar_timestamp(apollo_perception_obstacles->mutable_header()->radar_timestamp());
  crdc_perception_obstacles_->mutable_header()->set_version(apollo_perception_obstacles->mutable_header()->version());
  crdc_perception_obstacles_->mutable_header()->set_frame_id(apollo_perception_obstacles->mutable_header()->frame_id());
  crdc_perception_obstacles_->mutable_header()->set_module_name("LGSVL3DGroundTruth");
}

crdc::airi::PerceptionObstacle_Type TransformPercptionObstacles::int_to_obstacle(int object_type_int) {
  crdc::airi::PerceptionObstacle_Type object_type_obstacle;
  switch (object_type_int) {
    case 0:
      object_type_obstacle = crdc::airi::PerceptionObstacle::UNKNOWN;
    break;
    case 1:
      object_type_obstacle = crdc::airi::PerceptionObstacle::UNKNOWN_MOVABLE;
    break;
    case 2:
      object_type_obstacle = crdc::airi::PerceptionObstacle::UNKNOWN_UNMOVABLE;
    break;
    case 3:
      object_type_obstacle = crdc::airi::PerceptionObstacle::PEDESTRIAN;
    break;
    case 4:
      object_type_obstacle = crdc::airi::PerceptionObstacle::BICYCLE;
    break;
    case 5:
      object_type_obstacle = crdc::airi::PerceptionObstacle::VEHICLE;
    break;
    default:
      object_type_obstacle = crdc::airi::PerceptionObstacle::UNKNOWN;
    break;
  }
  return object_type_obstacle;
}

void TransformPercptionObstacles::transform_perception_obstacle(apollo::perception::PerceptionObstacle* apollo_perception_obstacle,
          crdc::airi::PerceptionObstacle* crdc_perception_obstacle) {
  int timestamp = static_cast<int>(crdc_perception_obstacles_->mutable_header()->timestamp_sec()* 100) ;
  if (gps_timestamp_position_.find(timestamp) != gps_timestamp_position_.end()) {
    double gps_heading = gps_timestamp_heading_[timestamp] - 90;  // 90: default easting
    double gps_theta = gps_heading * M_PI / 180;

    crdc_perception_obstacle->set_id(apollo_perception_obstacle->id());
    crdc_perception_obstacle->set_theta(apollo_perception_obstacle->theta() - gps_theta);

    // Rotate according to ego heading
    // x' = x * cos(angle) + y * sin(angle), y' = y * cos(angle) - x * sin(angle)
    crdc_perception_obstacle->mutable_position()->set_x(
          (-apollo_perception_obstacle->mutable_position()->x() + gps_timestamp_position_[timestamp].x()) *
            cos(gps_heading * M_PI / 180.0) +
          (-apollo_perception_obstacle->mutable_position()->y() + gps_timestamp_position_[timestamp].y()) *
            sin(gps_heading * M_PI / 180.0)
    );
    crdc_perception_obstacle->mutable_position()->set_y(
          (-apollo_perception_obstacle->mutable_position()->y() + gps_timestamp_position_[timestamp].y()) *
            cos(gps_heading * M_PI / 180.0) -
          (-apollo_perception_obstacle->mutable_position()->x() + gps_timestamp_position_[timestamp].x()) *
            sin(gps_heading * M_PI / 180.0)
    );
    crdc_perception_obstacle->mutable_position()->set_z(apollo_perception_obstacle->mutable_position()->z()
          - gps_timestamp_position_[timestamp].z()
    );

    crdc_perception_obstacle->mutable_velocity()->set_x(apollo_perception_obstacle->mutable_velocity()->x());
    crdc_perception_obstacle->mutable_velocity()->set_y(apollo_perception_obstacle->mutable_velocity()->y());

    crdc_perception_obstacle->set_length(apollo_perception_obstacle->length());
    crdc_perception_obstacle->set_width(apollo_perception_obstacle->width());
    crdc_perception_obstacle->set_height(apollo_perception_obstacle->height());

    crdc::airi::PerceptionObstacle_Type object_type = int_to_obstacle(static_cast<int>(apollo_perception_obstacle->type()));
    crdc_perception_obstacle->set_type(object_type);
  }
}

// class TransformPercptionObstacles
bool TransformPercptionObstacles::transform_perception_obstacle_to_crdc(const std::shared_ptr<apollo::perception::PerceptionObstacles> &apollo_perception_obstacles,
        std::shared_ptr<crdc::airi::PerceptionObstacles> &crdc_perception_obstacles_) {
  crdc_perception_obstacles_->Clear();
  transform_header(apollo_perception_obstacles, crdc_perception_obstacles_);

  // get obstacles(i) from apollo and set it to crdc
  for (int i = 0; i < apollo_perception_obstacles->perception_obstacle().size(); ++i) {
    auto crdc_perception_obstacle = crdc_perception_obstacles_->add_perception_obstacle();
    transform_perception_obstacle(apollo_perception_obstacles->mutable_perception_obstacle(i), crdc_perception_obstacle);
  }
  talker_->Write(crdc_perception_obstacles_);
}


bool TransformPercptionObstacles::init(const TransformComponentConfig& config) {
  crdc_perception_obstacles_ = std::make_shared<crdc::airi::PerceptionObstacles>();
  input_topic_ = config.input_topic();
  output_topic_ = config.output_topic();
  std::string gps_topic = config.gps_topic();
  if (gps_topic.empty()) {
    LOG(FATAL) << "need gps topic to parse ego heading";
  }
  LOG(INFO) << "subscribe perception obstacles message from topic: " << input_topic_;

  // create listener
  auto listener_node = apollo::cyber::CreateNode("perception_obstacles_reader");
  auto listener = listener_node->CreateReader<apollo::perception::PerceptionObstacles>(
        input_topic_, [this](const std::shared_ptr<apollo::perception::PerceptionObstacles>&apollo_perception_obstacles){
            transform_perception_obstacle_to_crdc(apollo_perception_obstacles, crdc_perception_obstacles_);
    });
  auto listener_gps = listener_node->CreateReader<apollo::localization::Gps>(
        gps_topic, [this](const std::shared_ptr<apollo::localization::Gps>&apollo_gps){
          gps_timestamp_position_[static_cast<int>(apollo_gps->mutable_header()->timestamp_sec() * 100)] =
                apollo_gps->mutable_localization()->position();
          gps_timestamp_heading_[static_cast<int>(apollo_gps->mutable_header()->timestamp_sec() * 100)] =
                apollo_gps->mutable_localization()->heading();
  });
  // create talker
  auto talker_node = apollo::cyber::CreateNode("perception_obstacles_talker");
  talker_ = talker_node->CreateWriter<crdc::airi::PerceptionObstacles>(output_topic_);
  apollo::cyber::WaitForShutdown();

  return true;
}

}  // namespace airi
}  // namespace crdc