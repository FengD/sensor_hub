#pragma once

#include "cyber/cyber.h"
#include "proto/transform_component_config.pb.h"
#include "cyber/sensor_proto/lidar.pb.h"
#include "cyber/sensor_proto/image.pb.h"
#include "cyber/lgsvl_proto/pointcloud.pb.h"
#include "cyber/lgsvl_proto/sensor_image.pb.h"
#include "cyber/lgsvl_proto/detection2darray.pb.h"

namespace crdc {
namespace airi {

#define DRAW_ON_MAT 0
#define PointFieldBytes 40

class TransformImage {
 public:
  TransformImage() = default;
  virtual ~TransformImage() = default;

  bool init(const TransformComponentConfig& config);

  bool transform_img_to_crdc(const apollo::drivers::CompressedImage &apollo_compressed_img,
        crdc::airi::Image2 &crdc_img2_);

  void read_image_from_topic(const std::shared_ptr<apollo::drivers::CompressedImage>& apollo_compressed_img);

  crdc::airi::Header construct_header(const apollo::drivers::CompressedImage &apollo_msg);

 private:
  crdc::airi::Image2 crdc_img2_;
  std::string input_topic_;
  std::string output_topic_;
  std::shared_ptr<apollo::cyber::Writer<crdc::airi::Image2>> talker_;
#ifdef DRAW_ON_MAT
  std::unordered_map<int, std::shared_ptr<apollo::common::Detection2DArray>> detection_timestamp_2darray;
#endif
};

class TransformPointCloud {
 public:
  TransformPointCloud() = default;
  virtual ~TransformPointCloud() = default;

  bool init(const TransformComponentConfig& config);

  bool transform_pcd_to_crdc(const std::shared_ptr<apollo::drivers::PointCloud> &apollo_pcd,
        std::shared_ptr<crdc::airi::PointCloud2> &crdc_pcd2_);

 private:
  std::shared_ptr<crdc::airi::PointCloud2> crdc_pcd2_;
  std::string input_topic_;
  std::string output_topic_;
  std::shared_ptr<apollo::cyber::Writer<crdc::airi::PointCloud2>> talker_;
};

}  // namespace airi
}  // namespace crdc