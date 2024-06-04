#include "transform/transform_2d_gt.h"

namespace crdc {
namespace airi {

bool Transform2DGT::transform_2dgt_to_crdc(const std::shared_ptr<apollo::common::Detection2DArray> &apollo_2dgt,
        std::shared_ptr<crdc::airi::Detection2DArray> &crdc_2dgt_) {
  // header
  crdc_2dgt_->mutable_header()->set_timestamp_sec(apollo_2dgt->mutable_header()->timestamp_sec());
  crdc_2dgt_->mutable_header()->set_sequence_num(apollo_2dgt->mutable_header()->sequence_num());
  crdc_2dgt_->mutable_header()->set_lidar_timestamp(apollo_2dgt->mutable_header()->lidar_timestamp());
  crdc_2dgt_->mutable_header()->set_camera_timestamp(apollo_2dgt->mutable_header()->camera_timestamp());
  crdc_2dgt_->mutable_header()->set_radar_timestamp(apollo_2dgt->mutable_header()->radar_timestamp());
  crdc_2dgt_->mutable_header()->set_version(apollo_2dgt->mutable_header()->version());
  crdc_2dgt_->mutable_header()->set_frame_id(apollo_2dgt->mutable_header()->frame_id());
  crdc_2dgt_->mutable_header()->set_module_name("LGSVL2DGT");

  for (int i = 0; i < apollo_2dgt->detections().size(); ++i) {
    auto detection = crdc_2dgt_->add_detections();
    detection->mutable_bbox()->set_x(apollo_2dgt->mutable_detections(i)->mutable_bbox()->x());
    detection->mutable_bbox()->set_y(apollo_2dgt->mutable_detections(i)->mutable_bbox()->y());
    detection->mutable_bbox()->set_width(apollo_2dgt->mutable_detections(i)->mutable_bbox()->width());
    detection->mutable_bbox()->set_height(apollo_2dgt->mutable_detections(i)->mutable_bbox()->height());
    detection->set_label(apollo_2dgt->mutable_detections(i)->label());
    detection->set_id(apollo_2dgt->mutable_detections(i)->id());
  }
  LOG(INFO)  << "publish 2d ground truth message to topic: " << output_topic_;
  talker_->Write(crdc_2dgt_);
}

bool Transform2DGT::init(const TransformComponentConfig& config) {
  crdc_2dgt_ = std::make_shared<crdc::airi::Detection2DArray>();
  input_topic_ = config.input_topic();
  output_topic_ = config.output_topic();
  LOG(INFO) << "subscribe 2d ground truth message from topic: " << input_topic_;

  // create listener
  auto listener_node = apollo::cyber::CreateNode("2DGT_reader");
  auto listener = listener_node->CreateReader<apollo::common::Detection2DArray>(
        input_topic_, [this](const std::shared_ptr<apollo::common::Detection2DArray>&apollo_2dgt){
            transform_2dgt_to_crdc(apollo_2dgt, crdc_2dgt_);
    });
  // create talker
  auto talker_node = apollo::cyber::CreateNode("2DGT_talker");
  talker_ = talker_node->CreateWriter<crdc::airi::Detection2DArray>(output_topic_);
  apollo::cyber::WaitForShutdown();

  return true;
}

}  // namespace airi
}  // namespace crdc