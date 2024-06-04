#pragma once
#include <pcl/common/common.h>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <opencv2/opencv.hpp>
#ifndef WITH_ROS2
#include "cyber/sensor_proto/lidar.pb.h"
#include "lidar_drivers/output/cyber_output.h"
#else
#include "lidar_drivers/output/af_output.h"
#endif
#ifdef WITH_TDA4
#include "tools/service_server/data_encode_decode.h"
#include "tools/service_server/calib_utils.h"
#endif
#include "tools/lidars_calibrate/calibrate/include/multi_lidar_calibrator.h"
#include "tools/lidars_calibrate/calibrate/proto/car_info_config.pb.h"
#include "tools/lidars_calibrate/calibrate/proto/cloud_transform_config.pb.h"
#include "common/util.h"
#include "tools/lidars_calibrate/calibrate/include/utils.hpp"

#ifdef WITH_ROS2
using PointClouds2 = sensor_msg::msg::PointClouds2;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
#else
#define ADD_FIELD(cloud, name, datatype, offset, count) \
  f = cloud->add_fields();                              \
  f->set_name(name);                                    \
  f->set_datatype(datatype);                            \
  f->set_offset(offset);                                \
  f->set_count(count);
#endif

namespace crdc {
namespace airi {

struct CaliPoint {
  float x_ = 0;
  float y_ = 0;
  float z_ = 0;
  CaliPoint() {
  }
};

class LidarsCalibration {
 public:
  LidarsCalibration();
  ~LidarsCalibration();

  bool init(const std::string &config_path);

#ifdef WITH_TDA4
  int process(pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr fr_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr rl_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr rr_cloud, std::string &path);
  std::string get_save_path() {
    std::string proto_save_path;
    if (get_product_name() == "HH03-3") {
      proto_save_path = save_path_ + "LIDAR_FL.prototxt";
    } else {
      proto_save_path = save_path_ + "LIDAR_RR.prototxt";
    }
    return proto_save_path;
  }

  bool calib_finshed_ = false;
#else
  int process(pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr fr_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr rl_cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr rr_cloud);
#endif

  std::string name() const {
    return "LidarsCalibration";
  }

  bool cali_flag_;  // if calibrate left and right lidar

 private:
#ifdef WITH_ROS2
  void pcltoros2(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, struct CaliPoint *lidar_point_,
                 std::shared_ptr<PointCloud2> &proto_cloud_);

  void add_field(sensor_msgs::msg::PointField &f, const std::string &name, const uint8_t datatype,
                 const int offset, const int count);
#else
  void pcltocyber(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_, struct CaliPoint *lidar_point_,
                  std::shared_ptr<PointCloud2> &proto_cloud_);
#endif
#ifdef WITH_TDA4
  std::string write_calib_res_proto(double x, double y, double z, double roll, double pitch,
                                    double yaw);
  void convert_pcl_to_image(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  std::string current_time_str_, result_save_path_, file_save_prefix_;
  cv::Mat rotatedImage, rotatedImage2, rotatedImage3;
  std::shared_ptr<DataEncoderDecoder> data_converter_;
  std::shared_ptr<CalibrationUtils> calib_utils_;
#endif

  void construct_cloud(std::shared_ptr<PointCloud2> &proto_cloud_, const std::string framd_id);
  // transform_the_grid_center
  void Calculate_carbody_grid(const PointICloudPtr &input_cloud_ptr,
                              std::vector<float> &transform_params, const CarbodyRoi &carbody_roi,
                              const float &carbody_grid, float &grid_dist, float &grid_aver);
  int32_t preprocess(const PointICloudPtr &input_cloud_ptr, std::vector<float> &transform_params,
                     const CarbodyRoi &carbody_roi);
  void RANSACPlane(const pcl::PointCloud<PointI>::ConstPtr &input_cloud_ptr,
                   std::vector<int> &inliers);
  void Caculate_grid_x(const pcl::PointCloud<PointI>::ConstPtr &src_cloud,
                       std::vector<float> &transform_params, float &x_dist, float &x_aver,
                       const float &carbody_x);
  void Caculate_grid_y(const pcl::PointCloud<PointI>::ConstPtr &src_cloud,
                       std::vector<float> &transform_params, float &y_dist, float &y_aver,
                       const float &carbody_y);

  void get_diagonal_line_intersection(const float &x1, const float &x2, const float &x3,
                                      const float &x4, const float &y1, const float &y2,
                                      const float &y3, const float &y4, float &x, float &y);
  static float SumXFn(float sum, PointI p);
  static float SumYFn(float sum, PointI p);
  bool aligned;
  std::string vehicle_type_;
  std::string left_lidar_name_, right_lidar_name_;
  std::vector<std::vector<float>> t_;
  static constexpr int32_t NUM_POINTS = 50000;
  LidarsCalibrationConfig lidars_calibration_config_;
  MultiLidarCalibrator *multi_lidar_calibrator_;
  // left_tf_cloud info
  std::string tflcloud_channel_name_;
  std::shared_ptr<PointCloud2> proto_lcloud_;
  struct CaliPoint *lidar_lpoint_;
  // right_tf_cloud info
  std::string tfrcloud_channel_name_;
  std::shared_ptr<PointCloud2> proto_rcloud_;
  struct CaliPoint *lidar_rpoint_;
  std::string left_lidar2_name_, right_lidar2_name_;
  // left_tf_cloud2 info
  std::string trlcloud_channel_name_;
  std::shared_ptr<PointCloud2> proto_lcloud2_;
  struct CaliPoint *lidar_lpoint2_;
  // right_tf_cloud2 info
  std::string trrcloud_channel_name_;
  std::shared_ptr<PointCloud2> proto_rcloud2_;
  struct CaliPoint *lidar_rpoint2_;
  std::string config_fr_path, config_fl_path, config_rl_path, config_rr_path;

  PointICloudPtr preprocessed_pcloud_, filtered_pcloud_;
  float x_, y_;
  int version_num_;
  std::string save_path_;
};

}  // namespace airi
}  // namespace crdc
