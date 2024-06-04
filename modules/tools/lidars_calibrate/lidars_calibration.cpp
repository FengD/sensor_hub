#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <string>
#include "tools/lidars_calibrate/lidars_calibration.h"

#define TRANSFORM_CLOUD
namespace crdc {
namespace airi {

LidarsCalibration::LidarsCalibration() {
  multi_lidar_calibrator_ = new MultiLidarCalibrator();
  cali_flag_ = false;
  aligned = false;
}

LidarsCalibration::~LidarsCalibration() {
  delete multi_lidar_calibrator_;
}
#ifdef WITH_ROS2
void LidarsCalibration::add_field(sensor_msgs::msg::PointField &f, const std::string &name,
                                  const uint8_t datatype, const int offset, const int count) {
  f.name = name;
  f.datatype = datatype;
  f.offset = offset;
  f.count = count;
}
void LidarsCalibration::pcltoros2(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                  struct CaliPoint *lidar_point_,
                                  std::shared_ptr<PointCloud2> &proto_cloud_) {
  proto_cloud_->data.resize(cloud->size() * sizeof(CaliPoint));
  proto_cloud_->width = cloud->size();
  proto_cloud_->row_step = sizeof(CaliPoint) * proto_cloud_->width;
  for (int i = 0; i < cloud->size(); i++) {
    CaliPoint &pt = lidar_point_[i];
    pt.x_ = cloud->points[i].x;
    pt.y_ = cloud->points[i].y;
    pt.z_ = cloud->points[i].z;
  }
}
#else
void LidarsCalibration::pcltocyber(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_,
                                   struct CaliPoint *lidar_point_,
                                   std::shared_ptr<PointCloud2> &proto_cloud_) {
  proto_cloud_->mutable_data()->resize(cloud_->size() * sizeof(CaliPoint));
  proto_cloud_->set_width(cloud_->size());
  proto_cloud_->set_row_step(sizeof(CaliPoint) * proto_cloud_->width());
  for (int i = 0; i < cloud_->size(); i++) {
    CaliPoint &pt = lidar_point_[i];
    pt.x_ = cloud_->points[i].x;
    pt.y_ = cloud_->points[i].y;
    pt.z_ = cloud_->points[i].z;
  }
}
#endif

void LidarsCalibration::construct_cloud(std::shared_ptr<PointCloud2> &proto_cloud_,
                                        const std::string framd_id) {
#ifdef WITH_ROS2
  proto_cloud_->fields.clear();
  sensor_msgs::msg::PointField f;
  add_field(f, "x", sensor_msgs::msg::PointField::FLOAT32, 0, 1);
  proto_cloud_->fields.emplace_back(f);
  add_field(f, "y", sensor_msgs::msg::PointField::FLOAT32, 4, 1);
  proto_cloud_->fields.emplace_back(f);
  add_field(f, "z", sensor_msgs::msg::PointField::FLOAT32, 8, 1);
  proto_cloud_->fields.emplace_back(f);
  proto_cloud_->data.resize(NUM_POINTS * sizeof(CaliPoint));
  proto_cloud_->point_step = sizeof(CaliPoint);
  proto_cloud_->height = 1;
  proto_cloud_->width = NUM_POINTS;
  proto_cloud_->is_dense = true;
  proto_cloud_->row_step = proto_cloud_->point_step * proto_cloud_->width;
  proto_cloud_->is_bigendian = false;
  proto_cloud_->header.frame_id = vehicle_type_;
#else
  proto_cloud_->clear_fields();
  PointField *f;
  ADD_FIELD(proto_cloud_, "x", PointField::FLOAT32, 0, 1);
  ADD_FIELD(proto_cloud_, "y", PointField::FLOAT32, 4, 1);
  ADD_FIELD(proto_cloud_, "z", PointField::FLOAT32, 8, 1);
  proto_cloud_->mutable_data()->resize(NUM_POINTS * sizeof(CaliPoint));
  proto_cloud_->set_point_step(sizeof(CaliPoint));
  proto_cloud_->set_height(1);
  proto_cloud_->set_width(NUM_POINTS);
  proto_cloud_->set_is_dense(true);
  proto_cloud_->mutable_header()->set_frame_id(framd_id);
  proto_cloud_->set_row_step(sizeof(CaliPoint) * proto_cloud_->width());
  proto_cloud_->set_is_bigendian(false);
#endif
}

bool LidarsCalibration::init(const std::string &config_path) {
  // calibrate
  if (!crdc::airi::util::get_proto_from_file(config_path, &lidars_calibration_config_)) {
    LOG(FATAL) << "[LIDARSCALIBRATION] failed to read lidarscalibration config proto.";
    return false;
  }
  multi_lidar_calibrator_->setParameters(lidars_calibration_config_);
  aligned = lidars_calibration_config_.aligned();  // if cloud registration is completed
  vehicle_type_ = lidars_calibration_config_.vehicle_type();
  left_lidar_name_ = lidars_calibration_config_.lidarlist(0).name();
  right_lidar_name_ = lidars_calibration_config_.lidarlist(1).name();
  tflcloud_channel_name_ = lidars_calibration_config_.lidarlist(0).output_topic();
  tfrcloud_channel_name_ = lidars_calibration_config_.lidarlist(1).output_topic();
  if (0 == vehicle_type_.compare("hav")) {
    left_lidar2_name_ = lidars_calibration_config_.lidarlist(2).name();
    right_lidar2_name_ = lidars_calibration_config_.lidarlist(3).name();
    trlcloud_channel_name_ = lidars_calibration_config_.lidarlist(2).output_topic();
    trrcloud_channel_name_ = lidars_calibration_config_.lidarlist(3).output_topic();
    std::vector<std::vector<float>> t_temp(4, std::vector<float>(6, 0.0));
    t_.swap(t_temp);
  } else {
    std::vector<std::vector<float>> t_temp(2, std::vector<float>(6, 0.0));
    t_.swap(t_temp);
  }
  // tf left cloud output and right cloud output
  proto_lcloud_ = std::make_shared<PointCloud2>();
  proto_rcloud_ = std::make_shared<PointCloud2>();
  construct_cloud(proto_lcloud_, left_lidar_name_);
  construct_cloud(proto_rcloud_, right_lidar_name_);
#ifdef WITH_ROS2
  lidar_lpoint_ = (struct CaliPoint *)proto_lcloud_->data.data();
  lidar_rpoint_ = (struct CaliPoint *)proto_rcloud_->data.data();
#else
  lidar_lpoint_ = (struct CaliPoint *)proto_lcloud_->mutable_data()->data();
  lidar_rpoint_ = (struct CaliPoint *)proto_rcloud_->mutable_data()->data();
#endif
  if (0 == vehicle_type_.compare("hav")) {
    // tf left cloud2 output and  tf right cloud2 output
    proto_lcloud2_ = std::make_shared<PointCloud2>();
    proto_rcloud2_ = std::make_shared<PointCloud2>();
    construct_cloud(proto_lcloud2_, left_lidar2_name_);
    construct_cloud(proto_rcloud2_, right_lidar2_name_);
#ifdef WITH_ROS2
    lidar_lpoint2_ = (struct CaliPoint *)proto_lcloud2_->data.data();
    lidar_rpoint2_ = (struct CaliPoint *)proto_rcloud2_->data.data();
#else
    lidar_lpoint2_ = (struct CaliPoint *)proto_lcloud2_->mutable_data()->data();
    lidar_rpoint2_ = (struct CaliPoint *)proto_rcloud2_->mutable_data()->data();
#endif
  }
  preprocessed_pcloud_.reset(new PointICloud);
  filtered_pcloud_.reset(new PointICloud);
  save_path_ = "/data/data/vehicle_configuration/hav/600/";
  version_num_ = 10000;
  return true;
}

#ifdef WITH_TDA4
int LidarsCalibration::process(pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr fr_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr rl_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr rr_cloud, std::string &path) {
#else
int LidarsCalibration::process(pcl::PointCloud<pcl::PointXYZI>::Ptr fl_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr fr_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr rl_cloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr rr_cloud) {
#endif
  auto &left_cloud = fl_cloud;
  auto &right_cloud = fr_cloud;
  auto &left_cloud2 = rl_cloud;
  auto &right_cloud2 = rr_cloud;
  TransformConfig cal_fr_value, cal_fl_value, cal_rl_value, cal_rr_value;
#ifdef WITH_TDA4
  std::string calibrate_mode = "auto";
#else
  std::string calibrate_mode = std::string(std::getenv("CALI_MODEL"));
#endif
  if (0 == calibrate_mode.compare("manual")) {
#ifdef WITH_ROS2
    pcltoros2(left_cloud, lidar_lpoint_, proto_lcloud_);
    common::Singleton<LidarAFOutput>::get()->write_cloud(left_lidar_name_, proto_lcloud_);
    pcltoros2(right_cloud, lidar_rpoint_, proto_rcloud_);
    common::Singleton<LidarAFOutput>::get()->write_cloud(right_lidar_name_, proto_rcloud_);
    if (0 == vehicle_type_.compare("hav")) {
      pcltoros2(left_cloud2, lidar_lpoint2_, proto_lcloud2_);
      common::Singleton<LidarAFOutput>::get()->write_cloud(left_lidar2_name_, proto_lcloud2_);
      pcltoros2(right_cloud2, lidar_rpoint2_, proto_rcloud2_);
      common::Singleton<LidarAFOutput>::get()->write_cloud(right_lidar2_name_, proto_rcloud2_);
    }
#endif
    if (0 == vehicle_type_.compare("hav")) {
      config_fr_path = std::string(std::getenv("CRDC_WS")) + "/params/hav/transform_fr.prototxt";
      config_fl_path = std::string(std::getenv("CRDC_WS")) + "/params/hav/transform_fl.prototxt";
      config_rl_path = std::string(std::getenv("CRDC_WS")) + "/params/hav/transform_rl.prototxt";
      config_rr_path = std::string(std::getenv("CRDC_WS")) + "/params/hav/transform_rr.prototxt";

      crdc::airi::util::get_proto_from_file(config_fr_path, &cal_fr_value);
      crdc::airi::util::get_proto_from_file(config_fl_path, &cal_fl_value);
      crdc::airi::util::get_proto_from_file(config_rl_path, &cal_rl_value);
      crdc::airi::util::get_proto_from_file(config_rr_path, &cal_rr_value);

      std::vector<float> params_fr{cal_fr_value.x(),    cal_fr_value.y(),     cal_fr_value.z(),
                                   cal_fr_value.roll(), cal_fr_value.pitch(), cal_fr_value.yaw()},
          params_fl{cal_fl_value.x(),    cal_fl_value.y(),     cal_fl_value.z(),
                    cal_fl_value.roll(), cal_fl_value.pitch(), cal_fl_value.yaw()},
          params_rl{cal_rl_value.x(),    cal_rl_value.y(),     cal_rl_value.z(),
                    cal_rl_value.roll(), cal_rl_value.pitch(), cal_rl_value.yaw()},
          params_rr{cal_rr_value.x(),    cal_rr_value.y(),     cal_rr_value.z(),
                    cal_rr_value.roll(), cal_rr_value.pitch(), cal_rr_value.yaw()};

      for (int i = 0; i <= 5; i++) {
        t_[0][i] = params_fr[i];
        t_[1][i] = params_fl[i];
        t_[2][i] = params_rl[i];
        t_[3][i] = params_rr[i];
      }
    } else {
      if (0 == vehicle_type_.compare("j6p")) {
        config_fr_path =
            std::string(std::getenv("CRDC_WS")) + "/params/j6p/transform_right.prototxt";
        config_fl_path =
            std::string(std::getenv("CRDC_WS")) + "/params/j6p/transform_left.prototxt";
      } else {
        config_fr_path =
            std::string(std::getenv("CRDC_WS")) + "/params/hav3g/transform_rr.prototxt";
        config_fl_path =
            std::string(std::getenv("CRDC_WS")) + "/params/hav3g/transform_fl.prototxt";
      }

      crdc::airi::util::get_proto_from_file(config_fr_path, &cal_fr_value);
      crdc::airi::util::get_proto_from_file(config_fl_path, &cal_fl_value);
      std::vector<float> params_fr{cal_fr_value.x(),    cal_fr_value.y(),     cal_fr_value.z(),
                                   cal_fr_value.roll(), cal_fr_value.pitch(), cal_fr_value.yaw()},
          params_fl{cal_fl_value.x(),    cal_fl_value.y(),     cal_fl_value.z(),
                    cal_fl_value.roll(), cal_fl_value.pitch(), cal_fl_value.yaw()};

      for (int i = 0; i <= 5; i++) {
        t_[0][i] = params_fl[i];
        t_[1][i] = params_fr[i];
      }
    }
  } else {
    if (!cali_flag_ && !aligned) {
      multi_lidar_calibrator_->setInputCloud(left_cloud, left_lidar_name_);
      multi_lidar_calibrator_->setInputCloud(right_cloud, right_lidar_name_);
      if (0 == vehicle_type_.compare("hav")) {
        multi_lidar_calibrator_->setInputCloud(left_cloud2, left_lidar2_name_);
        multi_lidar_calibrator_->setInputCloud(right_cloud2, right_lidar2_name_);
      }
      multi_lidar_calibrator_->Process();
      // 获取各个激光雷达的标定参数
      t_ = multi_lidar_calibrator_->getT();
      if (0 == vehicle_type_.compare("hav")) {
        std::cout << "右前雷达:\n"
                  << t_[0][0] << ", " << t_[0][1] << ", " << t_[0][2] << ", " << t_[0][3] << ", "
                  << t_[0][4] << ", " << t_[0][5] << std::endl;
        std::cout << "左前雷达:\n"
                  << t_[1][0] << ", " << t_[1][1] << ", " << t_[1][2] << ", " << t_[1][3] << ", "
                  << t_[1][4] << ", " << t_[1][5] << std::endl;
        std::cout << "左后雷达:\n"
                  << t_[2][0] << ", " << t_[2][1] << ", " << t_[2][2] << ", " << t_[2][3] << ", "
                  << t_[2][4] << ", " << t_[2][5] << std::endl;
        std::cout << "右后雷达:\n"
                  << t_[3][0] << ", " << t_[3][1] << ", " << t_[3][2] << ", " << t_[3][3] << ", "
                  << t_[3][4] << ", " << t_[3][5] << std::endl;
      } else {  // 该模式下通过多激光标定工程得到标定参数
        // 以激光雷达连线中点为坐标原点
        if (0 == vehicle_type_.compare("j6p")) {
          t_[0][0] = -t_[1][0] / 2;
          t_[1][0] = t_[1][0] / 2;
          t_[0][1] = -t_[1][1] / 2;
          t_[1][1] = t_[1][1] / 2;
        }
        std::cout << "左雷达:\n"
                  << t_[0][0] << ", " << t_[0][1] << ", " << t_[0][2] << ", " << t_[0][3] << ", "
                  << t_[0][4] << ", " << t_[0][5] << std::endl;
        std::cout << "右雷达:\n"
                  << t_[1][0] << ", " << t_[1][1] << ", " << t_[1][2] << ", " << t_[1][3] << ", "
                  << t_[1][4] << ", " << t_[1][5] << std::endl;
      }
      if (multi_lidar_calibrator_->isAllFinished()) {
        cali_flag_ = true;
        if (0 == vehicle_type_.compare("hav")) {
          // 计算以右前激光雷达为原点时４个激光雷达对角线连线交点坐标（x,y）
          get_diagonal_line_intersection(t_[0][0], t_[1][0], t_[2][0], t_[3][0], t_[0][1], t_[1][1],
                                         t_[2][1], t_[3][1], x_, y_);
          t_[0][0] = t_[0][0] - x_;
          t_[1][0] = t_[1][0] - x_;
          t_[2][0] = t_[2][0] - x_;
          t_[3][0] = t_[3][0] - x_;
          t_[0][1] = t_[0][1] - y_;
          t_[1][1] = t_[1][1] - y_;
          t_[2][1] = t_[2][1] - y_;
          t_[3][1] = t_[3][1] - y_;
          // 输出坐标平移后的参数
          std::cout << "右前雷达:\n"
                    << t_[0][0] << ", " << t_[0][1] << ", " << t_[0][2] << ", " << t_[0][3] << ", "
                    << t_[0][4] << ", " << t_[0][5] << std::endl;
          std::cout << "左前雷达:\n"
                    << t_[1][0] << ", " << t_[1][1] << ", " << t_[1][2] << ", " << t_[1][3] << ", "
                    << t_[1][4] << ", " << t_[1][5] << std::endl;
          std::cout << "左后雷达:\n"
                    << t_[2][0] << ", " << t_[2][1] << ", " << t_[2][2] << ", " << t_[2][3] << ", "
                    << t_[2][4] << ", " << t_[2][5] << std::endl;
          std::cout << "右后雷达:\n"
                    << t_[3][0] << ", " << t_[3][1] << ", " << t_[3][2] << ", " << t_[3][3] << ", "
                    << t_[3][4] << ", " << t_[3][5] << std::endl;

          transformCloud<PointI>(*left_cloud, t_[0], *left_cloud);
          transformCloud<PointI>(*right_cloud, t_[1], *right_cloud);
          transformCloud<PointI>(*left_cloud2, t_[2], *left_cloud2);
          transformCloud<PointI>(*right_cloud2, t_[3], *right_cloud2);

          // 根据雷达扫描到的车身信息对x、ｙ精确标定
          CarInfoConfig fr_carbody_info, fl_carbody_info, rl_carbody_info, rr_carbody_info;
          std::string car_info_fr_path =
              std::string(std::getenv("CRDC_WS")) + "params/hav/car_info_fr.prototxt";
          std::string car_info_fl_path =
              std::string(std::getenv("CRDC_WS")) + "params/hav/car_info_fl.prototxt";
          std::string car_info_rl_path =
              std::string(std::getenv("CRDC_WS")) + "params/hav/car_info_rl.prototxt";
          std::string car_info_rr_path =
              std::string(std::getenv("CRDC_WS")) + "params/hav/car_info_rr.prototxt";
          crdc::airi::util::get_proto_from_file(car_info_fr_path, &fr_carbody_info);
          crdc::airi::util::get_proto_from_file(car_info_fl_path, &fl_carbody_info);
          crdc::airi::util::get_proto_from_file(car_info_rl_path, &rl_carbody_info);
          crdc::airi::util::get_proto_from_file(car_info_rr_path, &rr_carbody_info);

          float fr_x_dist, fr_y_dist, fl_x_dist, fl_y_dist, rl_x_dist, rl_y_dist, rr_x_dist,
              rr_y_dist, fr_x_aver, fr_y_aver, fl_x_aver, fl_y_aver, rl_x_aver, rl_y_aver,
              rr_x_aver, rr_y_aver;

          Calculate_carbody_grid(left_cloud, t_[0], fr_carbody_info.front_roi(),
                                 fr_carbody_info.car_lidar_distance().deta_x(), fr_x_dist,
                                 fr_x_aver);
          Calculate_carbody_grid(left_cloud, t_[0], fr_carbody_info.side_roi(),
                                 fr_carbody_info.car_lidar_distance().deta_y(), fr_y_dist,
                                 fr_y_aver);
          std::cout << "fr_x_dist = " << fr_x_dist << ",fr_x_aver = " << fr_x_aver << std::endl;
          std::cout << "fr_y_dist = " << fr_y_dist << ",fr_y_aver = " << fr_y_aver << std::endl;

          Calculate_carbody_grid(right_cloud, t_[1], fl_carbody_info.front_roi(),
                                 fl_carbody_info.car_lidar_distance().deta_x(), fl_x_dist,
                                 fl_x_aver);
          Calculate_carbody_grid(right_cloud, t_[1], fl_carbody_info.side_roi(),
                                 fl_carbody_info.car_lidar_distance().deta_y(), fl_y_dist,
                                 fl_y_aver);
          std::cout << "fl_x_dist = " << fl_x_dist << ",fl_x_aver = " << fl_x_aver << std::endl;
          std::cout << "fl_y_dist = " << fl_y_dist << ",fl_y_aver = " << fl_y_aver << std::endl;

          Calculate_carbody_grid(left_cloud2, t_[2], rl_carbody_info.front_roi(),
                                 rl_carbody_info.car_lidar_distance().deta_x(), rl_x_dist,
                                 rl_x_aver);
          Calculate_carbody_grid(left_cloud2, t_[2], rl_carbody_info.side_roi(),
                                 rl_carbody_info.car_lidar_distance().deta_y(), rl_y_dist,
                                 rl_y_aver);
          std::cout << "rl_x_dist = " << rl_x_dist << ",rl_x_aver = " << rl_x_aver << std::endl;
          std::cout << "rl_y_dist = " << rl_y_dist << ",rl_y_aver = " << rl_y_aver << std::endl;

          Calculate_carbody_grid(right_cloud2, t_[3], rr_carbody_info.front_roi(),
                                 rr_carbody_info.car_lidar_distance().deta_x(), rr_x_dist,
                                 rr_x_aver);
          Calculate_carbody_grid(right_cloud2, t_[3], rr_carbody_info.side_roi(),
                                 rr_carbody_info.car_lidar_distance().deta_y(), rr_y_dist,
                                 rr_y_aver);
          std::cout << "rr_x_dist = " << rr_x_dist << ",rr_x_aver = " << rr_x_aver << std::endl;
          std::cout << "rr_y_dist = " << rr_y_dist << ",rr_y_aver = " << rr_y_aver << std::endl;

          // 输出精确标定后的参数
          std::cout << "右前雷达:\n"
                    << t_[0][0] << ", " << t_[0][1] << ", " << t_[0][2] << ", " << t_[0][3] << ", "
                    << t_[0][4] << ", " << t_[0][5] << std::endl;
          std::cout << "左前雷达:\n"
                    << t_[1][0] << ", " << t_[1][1] << ", " << t_[1][2] << ", " << t_[1][3] << ", "
                    << t_[1][4] << ", " << t_[1][5] << std::endl;
          std::cout << "左后雷达:\n"
                    << t_[2][0] << ", " << t_[2][1] << ", " << t_[2][2] << ", " << t_[2][3] << ", "
                    << t_[2][4] << ", " << t_[2][5] << std::endl;
          std::cout << "右后雷达:\n"
                    << t_[3][0] << ", " << t_[3][1] << ", " << t_[3][2] << ", " << t_[3][3] << ", "
                    << t_[3][4] << ", " << t_[3][5] << std::endl;
        }
        if (0 == vehicle_type_.compare("hav3g")) {
          t_[0][0] = 7.71;
          t_[0][1] = 1.39;
          t_[0][5] = 0.71;
          t_[1][0] = -7.71;
          t_[1][1] = -1.39;
          t_[1][5] = 3.84;
#ifdef WITH_TDA4
          std::string vehicle_number = get_vehicle_number();
          current_time_str_ = calib_utils_->get_current_time();
          if (get_product_name() == "HH03-3") {
            file_save_prefix_ = vehicle_number + "-003-LidarFL-" + current_time_str_;
            result_save_path_ = "/extdata/calibrationfl/";
            calib_utils_->check_or_create_file(result_save_path_);
            path =
                write_calib_res_proto(t_[0][0], t_[0][1], t_[0][2], t_[0][3], t_[0][4], t_[0][5]);
            system(("cp -r " + path + " " + "/data/data/vehicle_configuration/hav/600/" +
                    "LIDAR_FL.prototxt")
                       .c_str());
          } else {
            file_save_prefix_ = vehicle_number + "-005-LidarRR-" + current_time_str_;
            result_save_path_ = "/extdata/calibrationrr/";
            calib_utils_->check_or_create_file(result_save_path_);
            path =
                write_calib_res_proto(t_[1][0], t_[1][1], t_[1][2], t_[1][3], t_[1][4], t_[1][5]);
            system(("cp -r " + path + " " + "/data/data/vehicle_configuration/hav/600/" +
                    "LIDAR_RR.prototxt")
                       .c_str());
          }
          std::string base64_file_path = result_save_path_ + file_save_prefix_ + ".txt";
          // 根据雷达扫描到的车身信息对x、ｙ精确标定
          transformCloud<PointI>(*left_cloud, t_[0], *left_cloud);
          transformCloud<PointI>(*right_cloud, t_[1], *right_cloud);
          pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud_transformed(
              new pcl::PointCloud<pcl::PointXYZI>);
          // Combine the point clouds
          pcl::copyPointCloud((*left_cloud) + (*right_cloud), *merged_cloud_transformed);
          convert_pcl_to_image(merged_cloud_transformed);

          if (rotatedImage.empty() || rotatedImage2.empty() || rotatedImage3.empty()) {
            std::cerr << "无法加载图像文件" << std::endl;
            return 1;
          }

          // 确保三张图像具有相同的尺寸
          if (rotatedImage.size() != rotatedImage2.size() ||
              rotatedImage.size() != rotatedImage3.size()) {
            std::cerr << "图像尺寸不匹配" << std::endl;
            return 1;
          }

          // 创建一个新的图像，大小为三张图像的宽度之和 x 任意一个图像的高度
          int newWidth = rotatedImage.cols * 3;
          int newHeight = rotatedImage.rows;
          cv::Mat mergedImage(newHeight, newWidth, rotatedImage.type());

          // 将三张图像复制到新图像中
          rotatedImage.copyTo(mergedImage(cv::Rect(0, 0, rotatedImage.cols, rotatedImage.rows)));
          rotatedImage2.copyTo(
              mergedImage(cv::Rect(rotatedImage.cols, 0, rotatedImage2.cols, rotatedImage2.rows)));
          rotatedImage3.copyTo(mergedImage(
              cv::Rect(rotatedImage.cols * 2, 0, rotatedImage3.cols, rotatedImage3.rows)));

          // // 保存合并后的图像
          // std::string png_path = result_save_path_ + file_save_prefix_ + ".png";
          // cv::imwrite(png_path, mergedImage);

          std::cout << "图像合并成功并保存为 merged_image.png" << std::endl;
          std::string image_base64_stream = data_converter_->convert_image_to_base64(mergedImage);
          calib_utils_->write_base64_stream_file(base64_file_path, image_base64_stream);
          calib_utils_->pack_files(result_save_path_);
          calib_finshed_ = true;
          return 0;
#endif

          CarInfoConfig fl_carbody_info, rr_carbody_info;
          std::string car_info_fl_path =
              std::string(std::getenv("CRDC_WS")) + "params/hav3g/car_info_fl.prototxt";
          std::string car_info_rr_path =
              std::string(std::getenv("CRDC_WS")) + "params/hav3g/car_info_rr.prototxt";
          crdc::airi::util::get_proto_from_file(car_info_fl_path, &fl_carbody_info);
          crdc::airi::util::get_proto_from_file(car_info_rr_path, &rr_carbody_info);

          float fl_x_dist, fl_y_dist, rr_x_dist, rr_y_dist, fl_x_aver, fl_y_aver, rr_x_aver,
              rr_y_aver;

          Calculate_carbody_grid(left_cloud, t_[0], fl_carbody_info.front_roi(),
                                 fl_carbody_info.car_lidar_distance().deta_x(), fl_x_dist,
                                 fl_x_aver);
          Calculate_carbody_grid(left_cloud, t_[0], fl_carbody_info.side_roi(),
                                 fl_carbody_info.car_lidar_distance().deta_y(), fl_y_dist,
                                 fl_y_aver);
          std::cout << "fl_x_dist = " << fl_x_dist << ",fl_x_aver = " << fl_x_aver << std::endl;
          std::cout << "fl_y_dist = " << fl_y_dist << ",fl_y_aver = " << fl_y_aver << std::endl;
          Calculate_carbody_grid(right_cloud, t_[1], rr_carbody_info.front_roi(),
                                 rr_carbody_info.car_lidar_distance().deta_x(), rr_x_dist,
                                 rr_x_aver);
          Calculate_carbody_grid(right_cloud, t_[1], rr_carbody_info.side_roi(),
                                 rr_carbody_info.car_lidar_distance().deta_y(), rr_y_dist,
                                 rr_y_aver);
          std::cout << "rr_x_dist = " << rr_x_dist << ",rr_x_aver = " << rr_x_aver << std::endl;
          std::cout << "rr_y_dist = " << rr_y_dist << ",rr_y_aver = " << rr_y_aver << std::endl;

          // 输出精确标定后的参数
          std::cout << "左前雷达:\n"
                    << t_[0][0] << ", " << t_[0][1] << ", " << t_[0][2] << ", " << t_[0][3] << ", "
                    << t_[0][4] << ", " << t_[0][5] << std::endl;
          std::cout << "右后雷达:\n"
                    << t_[1][0] << ", " << t_[1][1] << ", " << t_[1][2] << ", " << t_[1][3] << ", "
                    << t_[1][4] << ", " << t_[1][5] << std::endl;
        }
      }
    }
  }

  if (!aligned) {
    transformCloud<PointI>(*left_cloud, t_[0], *left_cloud);
    transformCloud<PointI>(*right_cloud, t_[1], *right_cloud);
    if (0 == vehicle_type_.compare("hav")) {
      transformCloud<PointI>(*left_cloud2, t_[2], *left_cloud2);
      transformCloud<PointI>(*right_cloud2, t_[3], *right_cloud2);
    }
  }
#ifndef WITH_TDA4
#ifdef WITH_ROS2
  pcltoros2(left_cloud, lidar_lpoint_, proto_lcloud_);
  common::Singleton<LidarAFOutput>::get()->write_cloud(tflcloud_channel_name_, proto_lcloud_);
  pcltoros2(right_cloud, lidar_rpoint_, proto_rcloud_);
  common::Singleton<LidarAFOutput>::get()->write_cloud(tfrcloud_channel_name_, proto_rcloud_);
  if (0 == vehicle_type_.compare("hav")) {
    pcltoros2(left_cloud2, lidar_lpoint2_, proto_lcloud2_);
    common::Singleton<LidarAFOutput>::get()->write_cloud(trlcloud_channel_name_, proto_lcloud2_);
    pcltoros2(right_cloud2, lidar_rpoint2_, proto_rcloud2_);
    common::Singleton<LidarAFOutput>::get()->write_cloud(trrcloud_channel_name_, proto_rcloud2_);
  }
#else
  pcltocyber(left_cloud, lidar_lpoint_, proto_lcloud_);
  common::Singleton<LidarCyberOutput>::get()->write_cloud(tflcloud_channel_name_, proto_lcloud_);

  pcltocyber(right_cloud, lidar_rpoint_, proto_rcloud_);
  common::Singleton<LidarCyberOutput>::get()->write_cloud(tfrcloud_channel_name_, proto_rcloud_);
  if (0 == vehicle_type_.compare("hav")) {
    pcltocyber(left_cloud2, lidar_lpoint2_, proto_lcloud2_);
    common::Singleton<LidarCyberOutput>::get()->write_cloud(trlcloud_channel_name_, proto_lcloud2_);

    pcltocyber(right_cloud2, lidar_rpoint2_, proto_rcloud2_);
    common::Singleton<LidarCyberOutput>::get()->write_cloud(trrcloud_channel_name_, proto_rcloud2_);
  }
#endif
#endif
  return 0;
}

#ifdef WITH_TDA4
void LidarsCalibration::convert_pcl_to_image(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
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
  // // 创建RGB图像对象
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image2(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat image3(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  // 遍历点云中的每个点
  for (const auto &point : cloud->points) {
    // 俯视图
    int y = static_cast<int>((point.y - min_y) / (max_y - min_y) * width);
    int x = static_cast<int>((point.x - min_x) / (max_x - min_x) * height);
    // 检查点是否在图像范围内
    if (y >= 0 && y < width && x >= 0 && x < height) {
      // 将点的颜色信息复制到图像上
      cv::Vec3b &color1 = image.at<cv::Vec3b>(x, y);
      color1[0] = 0;
      color1[1] = 0;
      color1[2] = 255;
    }

    // 侧视图
    int z = static_cast<int>((point.z - min_z) / (max_z - min_z) * width);
    // 检查点是否在图像范围内
    if (z >= 0 && z < width && x >= 0 && x < height) {
      // 将点的颜色信息复制到图像上
      cv::Vec3b &color2 = image2.at<cv::Vec3b>(x, z);
      color2[0] = 0;
      color2[1] = 0;
      color2[2] = 255;
    }

    // 后视图
    y = static_cast<int>((point.y - min_y) / (max_y - min_y) * height);
    // 检查点是否在图像范围内
    if (z >= 0 && z < width && y >= 0 && y < height) {
      // 将点的颜色信息复制到图像上
      cv::Vec3b &color3 = image3.at<cv::Vec3b>(y, z);
      color3[0] = 0;
      color3[1] = 0;
      color3[2] = 255;
    }
  }

  cv::rotate(image, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
  cv::rotate(image2, rotatedImage2, cv::ROTATE_90_COUNTERCLOCKWISE);
  cv::rotate(image3, rotatedImage3, cv::ROTATE_90_COUNTERCLOCKWISE);
}

std::string LidarsCalibration::write_calib_res_proto(double x, double y, double z, double roll,
                                                     double pitch, double yaw) {
  std::string proto_save_path = result_save_path_ + file_save_prefix_ + ".prototxt";
  std::ofstream file(proto_save_path, std::ofstream::trunc);
  file << "  x: " << x << std::endl;
  file << "  y: " << y << std::endl;
  file << "  z: " << z << std::endl;
  file << "  roll: " << roll << std::endl;
  file << "  pitch: " << pitch << std::endl;
  file << "  yaw: " << yaw << std::endl;

  file.close();
  return proto_save_path;
}
#endif
void LidarsCalibration::get_diagonal_line_intersection(const float &x1, const float &x2,
                                                       const float &x3, const float &x4,
                                                       const float &y1, const float &y2,
                                                       const float &y3, const float &y4, float &x,
                                                       float &y) {
  // 以４个激光雷达对角线连线交点为坐标原点
  x = -(-x2 * x3 * y1 + x3 * x4 * y1 + x1 * x4 * y2 - x3 * x4 * y2 + x1 * x2 * y3 - x1 * x4 * y3 -
        x1 * x2 * y4 + x2 * x3 * y4) /
      (x2 * y1 - x4 * y1 - x1 * y2 + x3 * y2 - x2 * y3 + x4 * y3 + x1 * y4 - x3 * y4);
  y = -(-(-x3 * y1 + x1 * y3) * (y2 - y4) + (y1 - y3) * (-x4 * y2 + x2 * y4)) /
      ((-x2 + x4) * (y1 - y3) - (-x1 + x3) * (y2 - y4));
}

void LidarsCalibration::Calculate_carbody_grid(const PointICloudPtr &input_cloud_ptr,
                                               std::vector<float> &transform_params,
                                               const CarbodyRoi &carbody_roi,
                                               const float &carbody_grid, float &grid_dist,
                                               float &grid_aver) {
  preprocessed_pcloud_->clear();
  filtered_pcloud_->clear();
  preprocess(input_cloud_ptr, transform_params, carbody_roi);
  std::vector<int> inliers;
  RANSACPlane(preprocessed_pcloud_, inliers);
  size_t preprocessed_pcloud_size = preprocessed_pcloud_->size();
  if (inliers.size() > 0.8 * preprocessed_pcloud_size) {
    pcl::copyPointCloud<PointI>(*preprocessed_pcloud_, inliers, *filtered_pcloud_);
    if (carbody_roi.ymax() - carbody_roi.ymin() > 0.5) {
      Caculate_grid_x(filtered_pcloud_, transform_params, grid_dist, grid_aver, carbody_grid);
    } else {
      Caculate_grid_y(filtered_pcloud_, transform_params, grid_dist, grid_aver, carbody_grid);
    }
  }
}

int32_t LidarsCalibration::preprocess(const PointICloudPtr &input_cloud_ptr,
                                      std::vector<float> &transform_params,
                                      const CarbodyRoi &carbody_roi) {
  if (input_cloud_ptr->points.empty())
    return -1;
  size_t input_cloud_size = input_cloud_ptr->points.size();
  for (size_t i = 0; i < input_cloud_size; ++i) {
    if (input_cloud_ptr->points[i].y > carbody_roi.ymax())
      continue;
    if (input_cloud_ptr->points[i].y < carbody_roi.ymin())
      continue;
    if (input_cloud_ptr->points[i].x < carbody_roi.xmin())
      continue;
    if (input_cloud_ptr->points[i].x > carbody_roi.xmax())
      continue;
    if (input_cloud_ptr->points[i].z < carbody_roi.zmin())
      continue;
    if (input_cloud_ptr->points[i].z > carbody_roi.zmax())
      continue;
    if (atan((fabs(input_cloud_ptr->points[i].y - transform_params[1])) /
             (fabs(input_cloud_ptr->points[i].x - transform_params[0]))) >= 10 * M_PI / 180 &&
        atan((fabs(input_cloud_ptr->points[i].y - transform_params[1])) /
             (fabs(input_cloud_ptr->points[i].x - transform_params[0]))) <= 80 * M_PI / 180)
      continue;
    preprocessed_pcloud_->push_back(input_cloud_ptr->points[i]);
  }
  return 0;
}

void LidarsCalibration::RANSACPlane(const pcl::PointCloud<PointI>::ConstPtr &input_cloud_ptr,
                                    std::vector<int> &inliers) {
  typename pcl::SampleConsensusModelPlane<PointI>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<PointI>(input_cloud_ptr));
  pcl::RandomSampleConsensus<PointI> ransac(model_p);
  ransac.setDistanceThreshold(.1);
  ransac.computeModel();
  ransac.getInliers(inliers);
}

void LidarsCalibration::Caculate_grid_x(const pcl::PointCloud<PointI>::ConstPtr &src_cloud,
                                        std::vector<float> &transform_params, float &x_dist,
                                        float &x_aver, const float &carbody_x) {
  if (src_cloud->points.empty())
    return;
  float sumX = 0.0;
  sumX = std::accumulate(src_cloud->begin(), src_cloud->end(), 0.0, SumXFn);
  x_aver = sumX / src_cloud->size();
  x_dist = transform_params[0] - x_aver;
  transform_params[0] = transform_params[0] + (carbody_x - x_dist);
}

void LidarsCalibration::Caculate_grid_y(const pcl::PointCloud<PointI>::ConstPtr &src_cloud,
                                        std::vector<float> &transform_params, float &y_dist,
                                        float &y_aver, const float &carbody_y) {
  if (src_cloud->points.empty())
    return;
  float sumY = 0.0;
  sumY = std::accumulate(src_cloud->begin(), src_cloud->end(), 0.0, SumYFn);
  y_aver = sumY / src_cloud->size();
  y_dist = transform_params[1] - y_aver;
  transform_params[1] = transform_params[1] + (carbody_y - y_dist);
}

float LidarsCalibration::SumXFn(float sum, PointI p) {
  return sum + p.x;
}

float LidarsCalibration::SumYFn(float sum, PointI p) {
  return sum + p.y;
}

}  // namespace airi
}  // namespace crdc
