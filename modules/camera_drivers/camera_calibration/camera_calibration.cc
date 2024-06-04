#include "camera_drivers/camera_calibration/camera_calibration.h"
namespace crdc {
namespace airi {

bool CameraCalibrate::init(const std::string &config_path, const int img_width,
                           const int img_height) {
  algorithm::CameraCalibrationConfig config;
  if (!crdc::airi::util::is_path_exists(config_path)) {
    LOG(FATAL) << "[CameraCalibrate] Path of " << config_path << " not exists.";
    return false;
  }
  if (!crdc::airi::util::get_proto_from_file(config_path, &config)) {
    LOG(FATAL) << "[CameraCalibrate] Failed to get proto from " << config_path << ".";
    return false;
  }
  data_frame_ = std::make_shared<PerceptionDataFrame<int>>();
  data_frame_->camera_frame = std::make_shared<algorithm::CameraData>();
  data_frame_->camera_frame->calib_meas_data =
      std::make_shared<algorithm::CameraCalibMeasurmentData>();
  data_frame_->camera_frame->calib_intrinsic_param =
      std::make_shared<algorithm::CameraIntrinsicParams>();
  extrinsic_config_ = config;
  extrinsic_calibrate_ = std::make_shared<algorithm::CameraExtrinsic>(config);
  if (!config.has_save_path()) {
    LOG(FATAL) << "Need to set save path in prototxt";
    return false;
  }
  data_converter_ = std::make_shared<DataEncoderDecoder>();
  calib_utils_ = std::make_shared<CalibrationUtils>();
  res_save_path_ = config.save_path();
  calib_utils_->check_or_create_file(res_save_path_);
  camera_position_ = config.camera_position();
  vehicle_number_ = get_vehicle_number();
  product_name_ = get_product_name();
  version_num_ = 10000;
  params_save_path_ = "/data/data/vehicle_configuration/hav/600/";
  image_width_ = img_width;
  image_height_ = img_height;
  return true;
}

void CameraCalibrate::get_calib_image_points(algorithm::CameraCalibrationConfig config,
                                             std::vector<cv::Point2f> &image_points) {
  int points_num = config.extrinsic_config().correspond_points_size();
  image_points.clear();
  for (int i = 0; i < points_num; ++i) {
    cv::Point2f point_2d =
        cv::Point2f(config.extrinsic_config().correspond_points(i).image_points().x(),
                    config.extrinsic_config().correspond_points(i).image_points().y());
    image_points.push_back(point_2d);
  }
}

void CameraCalibrate::read_camera_intrinsic_param(
    algorithm::CameraCalibrationConfig config,
    std::shared_ptr<CamIntrinsicParam> &camera_intrinsic_param) {
  int distortion_params_size = config.camera_params().distortion_params_size();
  camera_intrinsic_param->fx = config.camera_params().intrinsic().fx();
  camera_intrinsic_param->cx = config.camera_params().intrinsic().cx();
  camera_intrinsic_param->fy = config.camera_params().intrinsic().fy();
  camera_intrinsic_param->cy = config.camera_params().intrinsic().cy();
  camera_intrinsic_param->distortion_params.clear();
  for (int i = 0; i < distortion_params_size; ++i) {
    camera_intrinsic_param->distortion_params.push_back(
        config.camera_params().distortion_params(i));
  }
}

void CameraCalibrate::init_instrinsic_distortion_coeffs(
    std::shared_ptr<CamIntrinsicParam> &camera_intrinsic_param) {
  intrinsic_.setTo(0);
  intrinsic_.at<double>(0, 0) = camera_intrinsic_param->fx;
  intrinsic_.at<double>(0, 2) = camera_intrinsic_param->cx;
  intrinsic_.at<double>(1, 1) = camera_intrinsic_param->fy;
  intrinsic_.at<double>(1, 2) = camera_intrinsic_param->cy;
  intrinsic_.at<double>(2, 2) = 1;
  int distortion_params_size = camera_intrinsic_param->distortion_params.size();
  distortion_coeffs_ = cv::Mat(1, distortion_params_size, CV_64FC1);
  for (int i = 0; i < distortion_params_size; ++i) {
    distortion_coeffs_.at<double>(0, i) = camera_intrinsic_param->distortion_params[i];
  }
}

bool CameraCalibrate::convert_image(std::vector<std::shared_ptr<const CameraRawData>> &images_raw) {
  int images_num = images_raw.size();
  if (images_num < 5) return false;
  for (int i = 0; i < images_num; ++i) {
    cv::Mat image;
    int offset = 0;
    if (images_raw[i]->data_type.compare("Y") == 0) {
      cv::Mat y(image_height_, image_width_, CV_8UC1,
                reinterpret_cast<void *>(images_raw[i]->image_.data + offset));
      cv::cvtColor(y, image, cv::COLOR_GRAY2RGB);
    } else if (images_raw[i]->data_type.compare("NV12") == 0) {
      cv::Mat nv12(image_height_ * 3 / 2, image_width_, CV_8UC1,
                   reinterpret_cast<void *>(images_raw[i]->image_.data + offset));
      cv::cvtColor(nv12, image, cv::COLOR_YUV2BGR_NV12);
    } else if (images_raw[i]->data_type.compare("RGB8") == 0) {
      cv::Mat rgb(image_height_, image_width_, CV_8UC3,
                  reinterpret_cast<void *>(images_raw[i]->image_.data + offset));
      image = rgb.clone();
    } else {
      LOG(ERROR) << "[Camera Calibration] type: " << images_raw[i]->data_type << " not support.";
      return false;
    }
    if (!image.empty()) {
      data_frame_->camera_frame->frames.push_back(image);
    }
  }
  if (data_frame_->camera_frame->frames.size() < 5) return false;
  return true;
}

std::string CameraCalibrate::get_params_file_path() {
  std::string params_file_path;
  if (product_name_ == "HH03-4") {
    params_file_path = params_save_path_ + "FrontCameraParams.prototxt";
  } else if (product_name_ == "HH03-6") {
    params_file_path = params_save_path_ + "RearCameraParams.prototxt";
  }
  return params_file_path;
}

void CameraCalibrate::write_params_to_prototxt(std::string params_file_path, const cv::Mat &rvec,
                                               const cv::Mat &tvec) {
  std::ofstream output_file(params_file_path, std::ios::trunc);

  if (output_file.is_open()) {
    output_file << "camera_params {" << std::endl;
    output_file << "    intrinsic {" << std::endl;
    output_file << "        fx: " << intrinsic_.at<double>(0, 0) << std::endl;
    output_file << "        cx: " << intrinsic_.at<double>(0, 2) << std::endl;
    output_file << "        fy: " << intrinsic_.at<double>(1, 1) << std::endl;
    output_file << "        cy: " << intrinsic_.at<double>(1, 2) << std::endl;
    output_file << "    }" << std::endl;
    for (int i = 0; i < distortion_coeffs_.cols; ++i) {
      output_file << "    distortion_params: " << distortion_coeffs_.at<double>(0, i) << std::endl;
    }
    output_file << "    extrinsic {" << std::endl;
    output_file << "        rx: " << rvec.at<double>(0, 0) << std::endl;
    output_file << "        ry: " << rvec.at<double>(0, 1) << std::endl;
    output_file << "        rz: " << rvec.at<double>(0, 2) << std::endl;
    output_file << "        tx: " << tvec.at<double>(0, 0) << std::endl;
    output_file << "        ty: " << tvec.at<double>(0, 1) << std::endl;
    output_file << "        tz: " << tvec.at<double>(0, 2) << std::endl;
    output_file << "    }" << std::endl;
    output_file << "}" << std::endl;
    output_file.close();
  } else {
    LOG(INFO) << "Unable to open file " << params_file_path << " for writing.";
  }
}

std::string CameraCalibrate::write_calib_res_yaml(const std::string &luts_path, cv::Mat &rvec,
                                                  cv::Mat &tvec) {
  std::string yaml_save_path = res_save_path_ + file_save_prefix_ + ".yaml";
  std::ofstream file(yaml_save_path, std::ofstream::trunc);
  file << "luts_path: " << luts_path << std::endl;
  file << "extrinsics:" << std::endl;
  file << "  x: " << tvec.at<double>(0, 0) << std::endl;
  file << "  y: " << tvec.at<double>(0, 1) << std::endl;
  file << "  z: " << tvec.at<double>(0, 2) << std::endl;
  file << "  roll: " << rvec.at<double>(0, 0) << std::endl;
  file << "  pitch: " << rvec.at<double>(0, 1) << std::endl;
  file << "  yaw: " << rvec.at<double>(0, 2) << std::endl;
  file << "version: " << version_num_ << std::endl;
  file.close();
  return yaml_save_path;
}

bool CameraCalibrate::cal_world_coord(const cv::Mat &rvec, const cv::Mat &tvec) {
  std::string params_file_path = get_params_file_path();
  write_params_to_prototxt(params_file_path, rvec, tvec);
  cv::Mat m_a = (cv::Mat_<double>(2, 2));
  cv::Mat m_b = (cv::Mat_<double>(2, 1));
  cv::Mat c = (cv::Mat_<double>(2, 1));  // m_a * c = m_b
  cv::Mat rrvec;
  cv::Rodrigues(rvec, rrvec);
  std::vector<cv::Point2f> undistorted_corners;
  std::vector<cv::Point2f> image_points;
  for (int i = 0; i < image_height_; ++i) {
    for (int j = 0; j < image_width_; ++j) {
      image_points.push_back(cv::Point2f(j, i));
    }
  }

  cv::undistortPoints(image_points, undistorted_corners, intrinsic_, distortion_coeffs_,
                      cv::noArray(), intrinsic_);

  for (size_t i = 0; i < undistorted_corners.size(); ++i) {
    m_a.at<double>(0, 0) = intrinsic_.at<double>(0, 0) * rrvec.at<double>(0, 0) +
                           intrinsic_.at<double>(0, 2) * rrvec.at<double>(2, 0) -
                           rrvec.at<double>(2, 0) * undistorted_corners[i].x;
    m_a.at<double>(0, 1) = intrinsic_.at<double>(0, 0) * rrvec.at<double>(0, 1) +
                           intrinsic_.at<double>(0, 2) * rrvec.at<double>(2, 1) -
                           rrvec.at<double>(2, 1) * undistorted_corners[i].x;

    m_a.at<double>(1, 0) = intrinsic_.at<double>(1, 1) * rrvec.at<double>(1, 0) +
                           intrinsic_.at<double>(1, 2) * rrvec.at<double>(2, 0) -
                           rrvec.at<double>(2, 0) * undistorted_corners[i].y;
    m_a.at<double>(1, 1) = intrinsic_.at<double>(1, 1) * rrvec.at<double>(1, 1) +
                           intrinsic_.at<double>(1, 2) * rrvec.at<double>(2, 1) -
                           rrvec.at<double>(2, 1) * undistorted_corners[i].y;

    m_b.at<double>(0, 0) = tvec.at<double>(0, 2) * undistorted_corners[i].x -
                           intrinsic_.at<double>(0, 0) * tvec.at<double>(0, 0) -
                           intrinsic_.at<double>(0, 2) * tvec.at<double>(0, 2);
    m_b.at<double>(1, 0) = tvec.at<double>(0, 2) * undistorted_corners[i].y -
                           intrinsic_.at<double>(1, 1) * tvec.at<double>(0, 1) -
                           intrinsic_.at<double>(1, 2) * tvec.at<double>(0, 2);

    cv::solve(m_a, m_b, c, cv::DECOMP_SVD);
    vehicle_points_.push_back(cv::Point2f(c.at<double>(0, 0), c.at<double>(1, 0)));
  }
  return true;
}

void CameraCalibrate::generate_real_distance_lut() {
  float *vehicle_points =
      reinterpret_cast<float *>(malloc(vehicle_points_.size() * 2 * sizeof(float)));
  for (size_t i = 0; i < vehicle_points_.size(); ++i) {
    vehicle_points[2 * i] = round(vehicle_points_[i].x * 100);
    vehicle_points[2 * i + 1] = round(vehicle_points_[i].y * 100);
  }
  std::ofstream outlutbinfile(
      params_save_path_ + "TableCameraCoordination_" + camera_position_ + "_vision.bin",
      std::ios::binary);
  outlutbinfile.write(reinterpret_cast<char *>(vehicle_points),
                      sizeof(float) * vehicle_points_.size() * 2);
  outlutbinfile.close();
  free(vehicle_points);
  LOG(INFO) << "LUT GENERATION PORCESS IS DONE";
}

void CameraCalibrate::struct_deep_copy(
    const std::shared_ptr<CamIntrinsicParam> &cam_intrinsic_param,
    const std::shared_ptr<DbusSendData> &dbus_send_data,
    std::shared_ptr<algorithm::CameraIntrinsicParams> &data_frame_intrinsic,
    std::shared_ptr<algorithm::CameraCalibMeasurmentData> &data_frame_calib_data) {
  data_frame_intrinsic->fx = cam_intrinsic_param->fx;
  data_frame_intrinsic->fy = cam_intrinsic_param->fy;
  data_frame_intrinsic->cx = cam_intrinsic_param->cx;
  data_frame_intrinsic->cy = cam_intrinsic_param->cy;
  data_frame_intrinsic->distortion_params = cam_intrinsic_param->distortion_params;

  data_frame_calib_data->calib_mode = dbus_send_data->calib_mode;
  data_frame_calib_data->real_world_points_meas = dbus_send_data->real_world_points_meas;
  data_frame_calib_data->real_world_points_valid = dbus_send_data->real_world_points_valid;
  data_frame_calib_data->checkboard_dis = dbus_send_data->checkboard_dis;
  data_frame_calib_data->checkboard_height = dbus_send_data->checkboard_height;
}

int CameraCalibrate::process(std::vector<std::shared_ptr<const CameraRawData>> &images_raw,
                             std::string &path, std::shared_ptr<DbusSendData> &dbus_send_data,
                             std::shared_ptr<CamIntrinsicParam> &cam_intrinsic_param) {
  if (!convert_image(images_raw)) return 4;
  if ((cam_intrinsic_param->fx == 0) && (cam_intrinsic_param->fy == 0)) {
    cam_intrinsic_param->distortion_params.clear();
    read_camera_intrinsic_param(extrinsic_config_, cam_intrinsic_param);
  }
  calib_utils_->find_image_points(data_frame_->camera_frame->frames[0],
                                  data_frame_->camera_frame->image_points,
                                  dbus_send_data->area_low_threshold);
  if (data_frame_->camera_frame->image_points.size() !=
      dbus_send_data->real_world_points_meas.size() +
          dbus_send_data->real_world_points_valid.size()) {
    get_calib_image_points(extrinsic_config_, data_frame_->camera_frame->image_points);
  }
  current_time_str_ = calib_utils_->get_current_time();
  if (product_name_ == "HH03-4") {
    file_save_prefix_ = vehicle_number_ + "-004-CameraF-" + current_time_str_;
  } else if (product_name_ == "HH03-6") {
    file_save_prefix_ = vehicle_number_ + "-006-CameraR-" + current_time_str_;
  }
  std::string old_luts_path =
      res_save_path_ + "TableCameraCoordination_" + camera_position_ + "_vision.bin";
  std::string valid_luts_path =
      res_save_path_ + "TableCameraCoordination_" + camera_position_ + "_vision_valid.bin";
  std::string luts_path = res_save_path_ + file_save_prefix_ + ".bin";
  std::string src_base64_file_path = res_save_path_ + file_save_prefix_ + ".prototxt";
  std::string valid_base64_file_path = res_save_path_ + file_save_prefix_ + ".txt";
  calib_utils_->delete_file(old_luts_path);
  struct_deep_copy(cam_intrinsic_param, dbus_send_data,
                   data_frame_->camera_frame->calib_intrinsic_param,
                   data_frame_->camera_frame->calib_meas_data);
  int status = extrinsic_calibrate_->processing(data_frame_);
  data_frame_->camera_frame->extrinsic->translation_vector.at<double>(0, 1) -=
      static_cast<double>(data_frame_->camera_frame->calib_meas_data->real_world_points_meas[0].z);
  if (extrinsic_calibrate_->cal_world_coord(
          data_frame_->camera_frame->extrinsic->rotation_vector,
          data_frame_->camera_frame->extrinsic->translation_vector)) {
    extrinsic_calibrate_->generate_real_distance_lut(valid_luts_path);
  }
  if (!extrinsic_calibrate_->validate_luts(valid_luts_path) ||
      data_frame_->camera_frame->extrinsic->errors > 10) {
    status = 4;
  } else {
    status = 0;
  }
  data_frame_->camera_frame->image_points.clear();
  if (status != 0) {
    LOG(INFO) << "Camera Extrinsic Calibration Failed";
    path = "";
    return status;
  }
  data_frame_->camera_frame->extrinsic->translation_vector.at<double>(0, 1) +=
      static_cast<double>(data_frame_->camera_frame->calib_meas_data->real_world_points_meas[0].z);
  std::string copy_luts_command = "cp -r " + old_luts_path + " " + params_save_path_;
  std::system(copy_luts_command.c_str());
  std::filesystem::rename(old_luts_path, luts_path);
  std::string src_image_base64_stream =
      data_converter_->convert_image_to_base64(data_frame_->camera_frame->frames[0]);
  calib_utils_->write_base64_stream_file(src_base64_file_path, src_image_base64_stream);
  std::string valid_image_base64_stream = data_converter_->convert_image_to_base64(
      data_frame_->camera_frame->extrinsic->validate_image);
  calib_utils_->write_base64_stream_file(valid_base64_file_path, valid_image_base64_stream);
  path = write_calib_res_yaml(luts_path, data_frame_->camera_frame->extrinsic->rotation_vector,
                              data_frame_->camera_frame->extrinsic->translation_vector);
  std::string params_save_path = get_params_file_path();
  init_instrinsic_distortion_coeffs(cam_intrinsic_param);
  write_params_to_prototxt(params_save_path, data_frame_->camera_frame->extrinsic->rotation_vector,
                           data_frame_->camera_frame->extrinsic->translation_vector);
  calib_utils_->delete_file(valid_luts_path);
  calib_utils_->pack_files(res_save_path_);
  return status;
}

}  // namespace airi
}  // namespace crdc
