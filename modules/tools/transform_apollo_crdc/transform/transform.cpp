#include <functional>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include "cyber/cyber.h"
#include "transform/transform.h"
#include "lidar_drivers/parser/parser.h"

namespace crdc {
namespace airi {

// class TransformImage
crdc::airi::Header TransformImage::construct_header(const apollo::drivers::CompressedImage &apollo_msg) {
  crdc::airi::Header crdc_header;
  crdc_header.set_timestamp_sec(apollo_msg.header().timestamp_sec());
  crdc_header.set_sequence_num(apollo_msg.header().sequence_num());
  crdc_header.set_lidar_timestamp(apollo_msg.header().lidar_timestamp());
  crdc_header.set_camera_timestamp(apollo_msg.header().camera_timestamp());
  crdc_header.set_radar_timestamp(apollo_msg.header().radar_timestamp());
  crdc_header.set_version(apollo_msg.header().version());
  crdc_header.set_frame_id(apollo_msg.header().frame_id());
  crdc_header.set_module_name("LGSVLCamera");
  return crdc_header;
}

bool TransformImage::transform_img_to_crdc(const apollo::drivers::CompressedImage &apollo_compressed_img,
        crdc::airi::Image2 &crdc_img2_) {
  crdc_img2_.mutable_header()->CopyFrom(construct_header(apollo_compressed_img));
  std::vector<uint8_t> compressed_buffer(apollo_compressed_img.data().begin(), apollo_compressed_img.data().end());
  cv::Mat image = cv::imdecode(compressed_buffer, CV_LOAD_IMAGE_UNCHANGED);
  cv::cvtColor(image, image, CV_BGR2RGB);
#ifdef DRAW_ON_MAT
  int timestamp = static_cast<int>(apollo_compressed_img.header().timestamp_sec() * 100);
  if (detection_timestamp_2darray.find(timestamp) !=
        detection_timestamp_2darray.end()) {
    int x, y, w, h;
    std::string label;
    for (int i = 0; i < detection_timestamp_2darray[timestamp]->detections().size(); ++i) {
      auto detection = detection_timestamp_2darray[timestamp]->mutable_detections(i);
      x = detection->mutable_bbox()->x();
      y = detection->mutable_bbox()->y();
      w = detection->mutable_bbox()->width();
      h = detection->mutable_bbox()->height();
      label = detection->label();
      cv::Point point_top_left(x - w / 2, y - h / 2), point_bottom_right(x + w / 2, y + h / 2);
      cv::rectangle(image, point_top_left, point_bottom_right, cv::Scalar(255, 255, 255), 2, 1, 0);
      cv::putText(image, label, cv::Point(x - w / 2 + 50, y - h / 2 + 150),
              cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2.0);
    }
  }
#endif
  crdc_img2_.set_height(image.rows);
  crdc_img2_.set_width(image.cols);
  crdc_img2_.set_type(std::to_string(crdc::airi::RGB8));
  crdc_img2_.set_compression(crdc::airi::Image2_Compression_RAW);
  crdc_img2_.set_step(3 * image.cols);
  crdc_img2_.set_data(image.data, image.rows * image.cols * image.channels());
}

void TransformImage::read_image_from_topic(
        const std::shared_ptr<apollo::drivers::CompressedImage>& apollo_compressed_img) {
  transform_img_to_crdc(*apollo_compressed_img, crdc_img2_);
  std::shared_ptr<crdc::airi::Image2> crdc_pub_img2 = std::make_shared<crdc::airi::Image2>(crdc_img2_);
  LOG(INFO)  << "publish image message to topic: " << output_topic_;
  talker_->Write(crdc_pub_img2);
}

bool TransformImage::init(const TransformComponentConfig& config) {
  input_topic_ = config.input_topic();
  output_topic_ = config.output_topic();
#ifdef DRAW_ON_MAT
  std::string detection_2d_gt_topic = config.detection_2d_gt_topic();
  if (detection_2d_gt_topic.empty()) {
    LOG(WARNING) << "[CAMERA] without 2d ground truth message";
  }
#endif
  auto talker_node = apollo::cyber::CreateNode("image_talker");
  talker_ = talker_node->CreateWriter<crdc::airi::Image2>(output_topic_);
  LOG(INFO) << "subscribe image message from topic: " << input_topic_;
  auto listener_node = apollo::cyber::CreateNode("image_reader");
  auto listener = listener_node->CreateReader<apollo::drivers::CompressedImage>(
            input_topic_, std::bind(&TransformImage::read_image_from_topic, this, std::placeholders::_1));
#ifdef DRAW_ON_MAT
  auto listener_2d_gt = listener_node->CreateReader<apollo::common::Detection2DArray>(
            detection_2d_gt_topic, [this](const std::shared_ptr<apollo::common::Detection2DArray> &apollo_2d_gt) {
              detection_timestamp_2darray[static_cast<int>(apollo_2d_gt->mutable_header()->timestamp_sec() * 100)] =
                  apollo_2d_gt;
  });
#endif
  apollo::cyber::WaitForShutdown();

  return true;
}

// class TransformPointCloud
bool TransformPointCloud::transform_pcd_to_crdc(const std::shared_ptr<apollo::drivers::PointCloud> &apollo_pcd,
        std::shared_ptr<crdc::airi::PointCloud2> &crdc_pcd2_) {
  // header
  crdc_pcd2_->mutable_header()->set_timestamp_sec(apollo_pcd->mutable_header()->timestamp_sec());
  crdc_pcd2_->mutable_header()->set_sequence_num(apollo_pcd->mutable_header()->sequence_num());
  crdc_pcd2_->mutable_header()->set_lidar_timestamp(apollo_pcd->mutable_header()->lidar_timestamp());
  crdc_pcd2_->mutable_header()->set_camera_timestamp(apollo_pcd->mutable_header()->camera_timestamp());
  crdc_pcd2_->mutable_header()->set_radar_timestamp(apollo_pcd->mutable_header()->radar_timestamp());
  crdc_pcd2_->mutable_header()->set_version(apollo_pcd->mutable_header()->version());
  crdc_pcd2_->mutable_header()->set_frame_id(apollo_pcd->mutable_header()->frame_id());
  crdc_pcd2_->mutable_header()->set_module_name("LGSVLLidar");

  crdc_pcd2_->set_height(apollo_pcd->height());
  crdc_pcd2_->set_width(apollo_pcd->width());
  crdc_pcd2_->set_point_step(40);
  crdc_pcd2_->set_row_step(40 * apollo_pcd->width());
  crdc_pcd2_->set_is_dense(apollo_pcd->is_dense());
  crdc_pcd2_->set_is_bigendian(false);
  // field
  crdc_pcd2_->clear_fields();
  crdc::airi::PointField* f;
  ADD_FIELD(crdc_pcd2_, "sec", PointField::UINT32, 0, 1);
  ADD_FIELD(crdc_pcd2_, "usec", PointField::UINT32, 4, 1);
  ADD_FIELD(crdc_pcd2_, "x", PointField::FLOAT32, 8, 1);
  ADD_FIELD(crdc_pcd2_, "y", PointField::FLOAT32, 12, 1);
  ADD_FIELD(crdc_pcd2_, "z", PointField::FLOAT32, 16, 1);
  ADD_FIELD(crdc_pcd2_, "intensity", PointField::FLOAT32, 20, 1);
  ADD_FIELD(crdc_pcd2_, "azimuth", PointField::FLOAT32, 24, 1);
  ADD_FIELD(crdc_pcd2_, "elevation", PointField::FLOAT32, 28, 1);
  ADD_FIELD(crdc_pcd2_, "ring", PointField::FLOAT32, 32, 1);
  ADD_FIELD(crdc_pcd2_, "distance", PointField::FLOAT32, 36, 1);
  // data
  crdc_pcd2_->mutable_data()->resize(PointFieldBytes * apollo_pcd->point().size());
  char *data = new char [PointFieldBytes * apollo_pcd->point().size()];
  int dis = 0;
  for (uint32_t w = 0; w < apollo_pcd->point().size(); ++w) {
    float point[] = {apollo_pcd->point(w).x(), apollo_pcd->point(w).y(),
            apollo_pcd->point(w).z()};
    memcpy(&data[dis + 8], point, sizeof(point));
    dis +=  crdc_pcd2_->point_step();
  }
  crdc_pcd2_->set_data(data, PointFieldBytes * apollo_pcd->point().size());
  delete [] data;
  LOG(INFO)  << "publish point cloud message to topic: " << output_topic_;
  talker_->Write(crdc_pcd2_);
}


bool TransformPointCloud::init(const TransformComponentConfig& config) {
  crdc_pcd2_ = std::make_shared<crdc::airi::PointCloud2>();
  input_topic_ = config.input_topic();
  output_topic_ = config.output_topic();
  LOG(INFO) << "subscribe point cloud message from topic: " << input_topic_;

  // create listener
  auto listener_node = apollo::cyber::CreateNode("lidar_reader");
  auto listener = listener_node->CreateReader<apollo::drivers::PointCloud>(
        input_topic_, [this](const std::shared_ptr<apollo::drivers::PointCloud>&apollo_pcd){
            transform_pcd_to_crdc(apollo_pcd, crdc_pcd2_);
    });
  // create talker
  auto talker_node = apollo::cyber::CreateNode("lidar_talker");
  talker_ = talker_node->CreateWriter<crdc::airi::PointCloud2>(output_topic_);
  apollo::cyber::WaitForShutdown();

  return true;
}

}  // namespace airi
}  // namespace crdc