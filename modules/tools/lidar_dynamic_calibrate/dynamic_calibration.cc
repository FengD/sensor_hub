#include <google/protobuf/text_format.h>
#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include "tools/lidar_dynamic_calibrate/dynamic_calibration.h"
#include "tools/lidar_dynamic_calibrate/af_dynamic_calibrate.h"
#ifdef WITH_TDA4
#include "common/util.h"
#endif

namespace crdc {
namespace airi {

DynamicCalibration::DynamicCalibration() {
  reset_param();
}

void DynamicCalibration::init(const std::string &frame_id) {
  node_ = common::Singleton<AFNode>::get()->get_node();
  thread_af_spin_ = std::thread(&DynamicCalibration::af_spin, this, node_);
  if (thread_af_spin_.joinable())
    thread_af_spin_.detach();
  common::Singleton<DynamicCalibrateOutput>::get()->init("dynamic_calibrate");
#ifdef WITH_TDA4
  std::string vehicle_number = "600";
  std::string vehicle_type = "hav";
#ifdef PROJ_V200_J6P10
  vehicle_number = "810";
  vehicle_type = "j6p";
#endif
  cali_path_ =
      std::string("/data/data/vehicle_configuration/") + vehicle_type + '/' + vehicle_number + '/';
  if (access(cali_path_.c_str(), 0) == -1) {
    // Create a path waiting for the parameter file to be written
    LOG(ERROR) << "[Calibrate] failed to read lidar calibration folder";
    system(("mkdir -p " + cali_path_).c_str());
  }
  cali_path_ = cali_path_ + frame_id + ".prototxt";
#else
  auto vehicle_id = std::getenv("VIN");
  if (!vehicle_id) {
    LOG(FATAL) << "VIN is not set. use the default_vin";
  } else {
    LOG(INFO) << "[Calibrate] Current VIN: " << std::string(std::getenv("VIN"));
  }
  cali_path_ = std::string(std::getenv("CRDC_WS")) + "/../" +
               std::string("vehicle_configuration/") + vehicle_id + "/" + frame_id + ".prototxt";
#endif

  std::string dynamic_path =
      std::string(std::getenv("CRDC_WS")) +
      "/params/drivers/lidar/dynamic_calibrate/lidar_dynamic_calibrate.prototxt";
  if (!crdc::airi::util::get_proto_from_file(dynamic_path, &calibrate_config_)) {
    LOG(FATAL) << "[Calibrate] failed to read lidar dynamic calibrate proto config: "
               << dynamic_path;
  }

  range_ = calibrate_config_.adjustment_range();
}

void DynamicCalibration::af_spin(std::shared_ptr<rclcpp::Node> node) {
  rclcpp::spin(node);
}

void DynamicCalibration::write_to_prototxt(CalibrationConfig &lidar_config) {
  if (!crdc::airi::util::set_proto_to_ascii_file(lidar_config_, cali_path_)) {
    LOG(FATAL) << "[Calibrate] failed to write calibration file: " << cali_path_;
  }
}

std::string DynamicCalibration::get_save_path() {
  return cali_path_;
}

int DynamicCalibration::get_process_state() {
  return process_state_;
}

void DynamicCalibration::receive_boundary_marker(Marker boundary_msg) {
  if (process_state_ == 1 || process_state_ == 2) {
    auto boundary_k = (boundary_msg.points[1].y - boundary_msg.points[0].y) / 20;
    auto boundary_b = (boundary_msg.points[1].y + boundary_msg.points[0].y) / 2;
    if (boundary_b == 100) {
      stop_frame_nums_++;
      LOG(ERROR) << "Not in a valid calibration scenario !!!";
    } else if (fabs(boundary_k) > range_) {
      stop_frame_nums_++;
      LOG(ERROR) << "The calibration vehicle did not travel in a straight line !!!";
    } else {
      current_frame_num_++;
      if (current_frame_num_ > calibrate_config_.start_frame_nums()) {
        count_num_++;
        stop_frame_nums_ = 0;
        process_state_ = 2;
        y_current_ = atan(boundary_k);
        b_current_ = boundary_b;
        b_total_ += b_current_;
        b_average_ = b_total_ / count_num_;
        // Filter out points with deviation more than 0.3 meters between b_average_ and b_current_
        if (fabs(b_average_ - b_current_) > calibrate_config_.b_offset_threshold() &&
            count_num_ >= 100) {
          count_num_--;
          b_total_ -= b_current_;
          b_average_ = b_total_ / count_num_;
        } else {
          yaw_array.push_back(y_current_);
          b_array.push_back(b_current_);
        }
      }
    }

    if (stop_frame_nums_ >= calibrate_config_.stop_frame_nums()) {
      if (count_num_ > calibrate_config_.min_total_nums()) {
        // Calculate the average value and fluctuation range of yaw_array, b_array
        int frame_num = yaw_array.size() - 100;
        double mean_yaw =
            std::accumulate(yaw_array.begin(), yaw_array.end() - 100, 0.0) / frame_num;
        double mean_b = std::accumulate(b_array.begin(), b_array.end() - 100, 0.0) / frame_num;
        int count_yaw_deviation = 0;
        int count_b_deviation = 0;
        for (size_t i = 0; i < frame_num; ++i) {
          if (fabs(yaw_array[i] - mean_yaw) > 0.01) {
            count_yaw_deviation++;
          }
          if (fabs(b_array[i] - mean_b) > 0.2) {
            count_b_deviation++;
          }
        }
        double deviation_yaw_rate = static_cast<double>(count_yaw_deviation) / frame_num;
        double deviation_b_rate = static_cast<double>(count_b_deviation) / frame_num;
        if (deviation_yaw_rate > calibrate_config_.yaw_deviation_rate_threshold() ||
            deviation_b_rate > calibrate_config_.b_deviation_rate_threshold()) {
          LOG(ERROR) << "High deviation between values and mean, calibration failed!!!";
          process_state_ = 4;
          return;
        }

        // Statistics 80% of data range
        std::sort(yaw_array.begin(), yaw_array.end());
        size_t lower_index = static_cast<size_t>(frame_num * 0.10);
        size_t upper_index = static_cast<size_t>(frame_num * 0.90);
        y_average_ =
            std::accumulate(yaw_array.begin() + lower_index, yaw_array.begin() + upper_index, 0.0) /
            frame_num;
        newyawangle_ = (oldyawangle_ - y_average_) * 1e7;
        lidar_config_.set_yaw(newyawangle_ / 1e7);
        if (!flag_)
          write_to_prototxt(lidar_config_);
#ifdef WITH_TDA4
        std::string vehicle_number = get_vehicle_number();
        current_time_str_ = calib_utils_->get_current_time();
        if (get_product_name() == "HH03-3") {
          file_save_prefix_ = vehicle_number + "-003-LidarFL-" + current_time_str_;
          result_save_path_ = "/extdata/calibrationfl/";
        } else {
          file_save_prefix_ = vehicle_number + "-005-LidarRR-" + current_time_str_;
          result_save_path_ = "/extdata/calibrationrr/";
        }
        calib_utils_->check_or_create_file(result_save_path_);
        system(("cp -r " + cali_path_ + " " + result_save_path_ + file_save_prefix_ + ".prototxt")
                   .c_str());
        get_the_calibrated_cloud(cloud_->cloud_msg_, -y_average_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
        pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        parse_point_to_pcl_cloud(cloud_->cloud_msg_, pcl_cloud);
        convert_pcl_to_image(pcl_cloud);
        std::string image_base64_stream = data_converter_->convert_image_to_base64(rotatedImage);
        std::string base64_file_path = result_save_path_ + file_save_prefix_ + ".txt";
        calib_utils_->write_base64_stream_file(base64_file_path, image_base64_stream);
        calib_utils_->pack_files(result_save_path_);
#endif
        flag_ = true;
        process_state_ = 3;
      } else {
        process_state_ = 4;
      }
    }
  }
}

void DynamicCalibration::reset_param() {
  flag_ = false;
  y_current_ = 0;
  y_average_ = 0;
  b_current_ = 0;
  b_average_ = 0;
  b_total_ = 0;
  oldyawangle_ = 0;
  newyawangle_ = 0;
  count_num_ = 0;
  current_frame_num_ = 0;
  stop_frame_nums_ = 0;
  process_state_ = 1;
}


void DynamicCalibration::get_the_calibrated_cloud(std::shared_ptr<PointCloud2> &cloud_msg,
                                                  float y_correction) {
  struct LidarPoint *lidar_point;
  Eigen::Transform<float, 3, Eigen::Affine> transform;
  transform = Eigen::AngleAxisf(y_correction, Eigen::Vector3f::UnitZ());
  cloud_msg = cloud_->cloud_msg_;
  lidar_point = (struct LidarPoint *)cloud_msg->data.data();
  int cloud_num = cloud_msg->height * cloud_msg->width;
  for (int j = 0; j < cloud_num; ++j) {
    Eigen::Matrix<float, 3, 1> pt(lidar_point[j].x_, lidar_point[j].y_, lidar_point[j].z_);
    Eigen::Vector3f rotated_point = transform * pt;
    lidar_point[j].x_ = rotated_point.x();
    lidar_point[j].y_ = rotated_point.y();
    lidar_point[j].z_ = rotated_point.z();
  }
}

void DynamicCalibration::parse_point_to_pcl_cloud(std::shared_ptr<PointCloud2> &pcd2_msg,
                                                  pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {
  cloud->clear();
  for (uint32_t h = 0; h < pcd2_msg->height; ++h) {
    auto data = pcd2_msg->data.data() + pcd2_msg->row_step * h;
    for (uint32_t w = 0; w < pcd2_msg->width; ++w) {
      auto p = data + 8;
      memcpy(&point_, p, sizeof(pcl::PointXYZI));
      cloud->push_back(point_);
      data += pcd2_msg->point_step;
    }
  }
}

void DynamicCalibration::convert_pcl_to_image(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  const int width = 960;
  const int height = 1280;
  pcl::PointXYZI minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);

  double min_x = minPt.x;
  double max_x = maxPt.x;
  double min_y = minPt.y;
  double max_y = maxPt.y;
  double min_z = minPt.z;
  double max_z = maxPt.z;
  // Create RGB image objects
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  // Traverse every point in the point cloud
  for (const auto &point : cloud->points) {
    // vertical view
    int y = static_cast<int>((point.y - min_y) / (max_y - min_y) * width);
    int x = static_cast<int>((point.x - min_x) / (max_x - min_x) * height);
    // Check if the checkpoint is within the image range
    if (y >= 0 && y < width && x >= 0 && x < height) {
      // Set the color information of points
      cv::Vec3b &color1 = image.at<cv::Vec3b>(x, y);
      color1[0] = 0;
      color1[1] = 0;
      color1[2] = 255;
    }
  }
  cv::rotate(image, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
}

void DynamicCalibration::process(std::shared_ptr<LidarPointCloud> cloud) {
  if (!crdc::airi::util::get_proto_from_file(cali_path_, &lidar_config_)) {
    LOG(FATAL) << "[Calibrate] failed to read lidar calibration proto config: " << cali_path_;
  }
  oldyawangle_ = lidar_config_.yaw();
  cloud_ = cloud;
  node_ = common::Singleton<AFNode>::get()->get_node();
  subscription_ = node_->create_subscription<Marker>(
      "boundary_visu", 10,
      std::bind(&DynamicCalibration::receive_boundary_marker, this, std::placeholders::_1));
}

}  // namespace airi
}  // namespace crdc
