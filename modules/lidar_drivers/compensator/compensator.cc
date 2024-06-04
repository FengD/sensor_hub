// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING, Zilou Cao
// Description: compensator

#include <algorithm>
#include "lidar_drivers/compensator/compensator.h"
#ifdef WITH_TDA4
#include "common/util.h"
#endif

namespace crdc {
namespace airi {

bool Compensator::init(const LidarComponent& config) {
  config_ = config;
  xytransfrom_ = true;
  auto vehicle_id = std::getenv("VIN");
  if (!vehicle_id) {
    LOG(FATAL) << "VIN is not set. use the default_vin";
  } else {
    LOG(INFO) << "[Compensator] Current VIN: "<< std::string(std::getenv("VIN"));
  }
  int lidar_num = config_.component_config_size();
  for (int i = 0; i < lidar_num; ++i) {
    calibration_path_.push_back(std::string(std::getenv("CRDC_WS")) + "/../"
         + std::string("vehicle_configuration/") + vehicle_id + "/"
         + config_.component_config(i).frame_id() + ".prototxt");
    lidar_transforms_[config_.component_config(i).frame_id()] =
      calibration_transform_matrix(calibration_path_.at(i));
  }
  return true;
}

bool Compensator::init(const LidarComponentConfig& config) {
  if (config.has_xytransfrom()) {
    xytransfrom_ = config.xytransfrom();
  } else {
    xytransfrom_ = true;
  }
#ifdef WITH_TDA4
  std::string vehicle_number = "600";
  std::string vehicle_type = "hav";
#ifdef PROJ_V200_J6P10
  vehicle_number = "810";
  vehicle_type = "j6p";
#endif
  std::string cali_dir_ = std::string("/data/data/vehicle_configuration/") + vehicle_type + '/'
                      + vehicle_number + '/';
  if (access(cali_dir_.c_str(), 0) == -1) {
    // Create a path waiting for the parameter file to be written
    LOG(ERROR) << "[Compensator] failed to read lidar calibration folder";
    if (system(("mkdir -p " + cali_dir_).c_str())) {
      LOG(WARNING) << "[Compensator] Failed to create dir " << cali_dir_;
    }
  }
  cali_path_ = cali_dir_ + config.frame_id() + ".prototxt";
#else
  auto vehicle_id = std::getenv("VIN");
  if (!vehicle_id) {
    LOG(FATAL) << "VIN is not set. use the default_vin";
  } else {
    LOG(INFO) << "[Compensator] Current VIN: "<< std::string(std::getenv("VIN"));
  }
  cali_path_ = std::string(std::getenv("CRDC_WS")) + "/../"
             + std::string("vehicle_configuration/") + vehicle_id + "/"
             + config.frame_id() + ".prototxt";
#endif
  lidar_transforms_[config.frame_id()] = calibration_transform_matrix(cali_path_);
  return true;
}

Eigen::Matrix4f Compensator::calibration_transform_matrix(
                      const std::string& config_file_path) {
  CalibrationConfig lidar_config;
  if (!crdc::airi::util::get_proto_from_file(config_file_path, &lidar_config)) {
    LOG(FATAL) << "[Compensator] failed to read lidar calibration proto config: "
               << config_file_path;
  }
  TfValue tf_value;
  if (xytransfrom_) {
    tf_value.x_ = lidar_config.x();
    tf_value.y_ = lidar_config.y();
  } else {
    tf_value.x_ = 0;
    tf_value.y_ = 0;
  }
  tf_value.z_ = lidar_config.z();
  tf_value.roll_ = lidar_config.roll();
  tf_value.pitch_ = lidar_config.pitch();
  tf_value.yaw_ = lidar_config.yaw();
  Eigen::Translation3f tl(tf_value.x_, tf_value.y_, tf_value.z_);
  Eigen::AngleAxisf rot_x(tf_value.roll_, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y(tf_value.pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z(tf_value.yaw_, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f rot_matrix = (tl * rot_z * rot_y * rot_x).matrix();
  return rot_matrix;
}

std::shared_ptr<PointCloud2> Compensator::cloud_transform(
              const std::shared_ptr<LidarPointCloud>& cloud) {
#ifdef WITH_ROS2
  auto& cloud_msg = cloud->cloud_msg_;
  std::string frame_id = cloud_msg->header.frame_id;
  Eigen::Matrix4f transform = lidar_transforms_[cloud_msg->header.frame_id];
  int cloud_num = cloud_msg->height * cloud_msg->width;
  if ("RADAR" == frame_id.substr(0, frame_id.find_first_of("_")) ||
        "MRR4D" == frame_id.substr(0, frame_id.find_first_of("_"))) {
    RadarPoint* radar_point;
    radar_point = reinterpret_cast<RadarPoint*>(cloud_msg->data.data());
    transforms(transform, radar_point, cloud_num);
  } else {
    struct LidarPoint* lidar_point;
    lidar_point = (struct LidarPoint*)cloud_msg->data.data();
    transforms(transform, lidar_point, cloud_num);
  }
  return cloud_msg;
#else
  struct LidarPoint* lidar_point;
  auto& proto_cloud = cloud->proto_cloud_;
  Eigen::Matrix4f transform = lidar_transforms_[proto_cloud->mutable_header()->frame_id()];
  lidar_point = (struct LidarPoint*)proto_cloud->mutable_data()->data();
  int cloud_num = proto_cloud->height() * proto_cloud->width();
  transforms(transform, lidar_point, cloud_num);
  return proto_cloud;
#endif
}

template<typename T>
void Compensator::transforms(Eigen::Matrix4f& transform, T points, int cloud_num) {
  for (int j = 0; j < cloud_num; ++j) {
    Eigen::Matrix<float, 3, 1> pt(points[j].x_, points[j].y_,
                                  points[j].z_);
    points[j].x_ = static_cast<float>(
        transform(0, 0) * pt.coeffRef(0) + transform(0, 1) * pt.coeffRef(1) +
        transform(0, 2) * pt.coeffRef(2) + transform(0, 3));
    points[j].y_ = static_cast<float>(
        transform(1, 0) * pt.coeffRef(0) + transform(1, 1) * pt.coeffRef(1) +
        transform(1, 2) * pt.coeffRef(2) + transform(1, 3));
    points[j].z_ = static_cast<float>(
        transform(2, 0) * pt.coeffRef(0) + transform(2, 1) * pt.coeffRef(1) +
        transform(2, 2) * pt.coeffRef(2) + transform(2, 3));
  }
}

bool Compensator::motion_compensation(const std::vector<std::shared_ptr<LidarPointCloud>>& clouds,
              std::shared_ptr<PointClouds2>& clouds_compensated,
              std::shared_ptr<CompensationInfo>& compensation_info) {
  for (size_t i = 0; i < clouds.size(); i++) {
#ifdef WITH_ROS2
    auto cloud_msg = cloud_transform(clouds[i]);
    clouds_compensated->header.stamp =
                cloud_msg->header.stamp;
    compensation_info->end_utime_ = cloud_msg->header.stamp.sec * 1000000000
                                    + cloud_msg->header.stamp.nanosec;
    clouds_compensated->clouds[i].set__header(cloud_msg->header);
    clouds_compensated->clouds[i].fields = cloud_msg->fields;
    clouds_compensated->clouds[i].data = cloud_msg->data;
    clouds_compensated->clouds[i].height = cloud_msg->height;
    clouds_compensated->clouds[i].width = cloud_msg->width;
    clouds_compensated->clouds[i].point_step = cloud_msg->point_step;
    clouds_compensated->clouds[i].row_step = cloud_msg->row_step;
    clouds_compensated->clouds[i].is_dense = cloud_msg->is_dense;
    clouds_compensated->clouds[i].is_bigendian = cloud_msg->is_bigendian;
#else
    auto proto_cloud = cloud_transform(clouds[i]);
    clouds_compensated->mutable_header()->set_lidar_timestamp(
                                   proto_cloud->mutable_header()->lidar_timestamp());
    compensation_info->end_utime_ = proto_cloud->mutable_header()->lidar_timestamp();
    *clouds_compensated->mutable_clouds(i)->mutable_header() = proto_cloud->header();
    *clouds_compensated->mutable_clouds(i)->mutable_fields() = proto_cloud->fields();
    *clouds_compensated->mutable_clouds(i)->mutable_data() = proto_cloud->data();
    clouds_compensated->mutable_clouds(i)->set_height(proto_cloud->height());
    clouds_compensated->mutable_clouds(i)->set_width(proto_cloud->width());
    clouds_compensated->mutable_clouds(i)->set_point_step(proto_cloud->point_step());
    clouds_compensated->mutable_clouds(i)->set_row_step(proto_cloud->row_step());
    clouds_compensated->mutable_clouds(i)->set_is_dense(proto_cloud->is_dense());
    clouds_compensated->mutable_clouds(i)->set_is_bigendian(proto_cloud->is_bigendian());
#endif
  }
  return true;
}

}  // namespace airi
}  // namespace crdc
