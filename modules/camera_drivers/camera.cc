// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera
// Contributor: shichong.wang

#include <omp.h>
#include "camera_drivers/camera.h"
#ifdef WITH_ROS2
#include "camera_drivers/output/af_output.h"
#else
#include "camera_drivers/output/cyber_output.h"
#endif

namespace sensor {
namespace hub {

Camera::Camera(const CameraComponentConfig& config) : common::Thread(true) {
    config_ = config;
    stop_ = false;
    flag_status_ = true;
    diagnose_executed_ = false;
    camera_name_ = config_.frame_id();
    sensor_position_id_ = config_.sensor_position_id();
    diagnose_begin_count_ = config_.diagnose_begin_count();
    diagnose_end_count_ = config_.diagnose_end_count();
    image_count_ = 0;
    image_seq_ = 0;
    encode_image_seq_ = 0;
    image_fps_index_ = 0;
    if (config_.has_encode_image_downsampling_each_image_frame()) {
        encode_image_downsampling_each_image_frame_ =
            config_.encode_image_downsampling_each_image_frame();
    } else {
        encode_image_downsampling_each_image_frame_ = 1;
    }
    std::string thread_name = camera_name_;
    if (thread_name.length() > MAX_THREAD_NAME_LENGTH) {
        thread_name = thread_name.substr(0, MAX_THREAD_NAME_LENGTH - 1);
    }
    set_thread_name(thread_name);

    if (config_.has_priority()) {
        set_priority(config_.priority());
    }

    std::string config_file_path = std::string(std::getenv("MAIN_WS"))
                                   + '/' + config_.config_file();
    if (!sensor::hub::util::is_path_exists(config_file_path)) {
        LOG(FATAL) << "[" << get_thread_name() << "] proto file not exits: "
                   << config_file_path;
        return;
    }

    if (!sensor::hub::util::get_proto_from_file(config_file_path,
                                              &camera_config_)) {
        LOG(FATAL) << "[" << get_thread_name() << "] failed to read camera proto config: "
                   << config_file_path;
        return;
    }
    if (config_.has_diagnose_config_file()) {
      std::string diagnose_config_file_path = std::string(std::getenv("MAIN_WS"))
                                    + '/' + config_.diagnose_config_file();
      if (!sensor::hub::util::is_path_exists(diagnose_config_file_path)) {
          LOG(FATAL) << "[" << get_thread_name() << "] proto file not exits: "
                    << diagnose_config_file_path;
          return;
      }

      if (!sensor::hub::util::get_proto_from_file(diagnose_config_file_path,
                                                &diagnose_config_)) {
          LOG(FATAL) << "[" << get_thread_name() << "] failed to read camera proto config: "
                    << diagnose_config_file_path;
          return;
      }
    }

    camera_diagnoser_ = std::make_shared<sensor::hub::CameraDiagnoser>(diagnose_config_);

    validate_calib_ = false;
    cam_intrinsic_param_ = std::make_shared<CamIntrinsicParam>();
#ifdef WITH_ROS2
    init_msg_image();
#ifdef WITH_TDA4
    if (config_.has_calibration_config()) {
      calibrate_process_ = std::make_shared<Calibrate>();
      std::string calib_config_file_path = std::string(std::getenv("MAIN_WS"))
                                   + '/' + config_.calibration_config();
      calibrate_process_->init(calib_config_file_path,
                               camera_config_.input_config().width(),
                               camera_config_.input_config().height());
      validate_calib_ = true;
    }
#endif
#else
    init_proto_image();
#endif
}
#ifdef WITH_ROS2
void Camera::init_msg_image() {
    image_msg_.header.frame_id = config_.frame_id();
    image_msg_.width = camera_config_.input_config().width();
    image_msg_.height = camera_config_.input_config().height();
    image_msg_.step = 3 * camera_config_.input_config().width();
    image_msg_.encoding = "RAW";
}
#else
void Camera::init_proto_image() {
    LOG(INFO) << "[" << get_thread_name() << "] create Camera Instance: "
              << config_.DebugString() << camera_config_.DebugString();
    proto_image_ = std::make_shared<Image2>();
    proto_image_->mutable_header()->set_frame_id(config_.frame_id());
    proto_image_->set_width(camera_config_.input_config().width());
    proto_image_->set_height(camera_config_.input_config().height());
    proto_image_->mutable_data()->reserve(camera_config_.input_config().width() *
                                          camera_config_.input_config().height() * 3);
    proto_image_->set_step(3 * camera_config_.input_config().width());
    proto_image_->set_compression(Image2_Compression_RAW);
}
#endif

bool Camera::init_encoder() {
    encoder_ = nullptr;
    if (camera_config_.has_encoder_config()) {
      camera_config_.mutable_encoder_config()->set_frame_id(config_.frame_id());
      encoder_ = EncoderFactory::get(camera_config_.encoder_config().name());
      if (!encoder_) {
          LOG(FATAL) << "[" << get_thread_name() << "] Failed to get camera encoder ptr: "
                << camera_config_.encoder_config().name();
        return false;
      }
      #ifdef WITH_TDA4
      if (camera_config_.encoder_config().name().compare("H264Encoder") == 0) {
        camera_config_.mutable_encoder_config()->
                    mutable_h264_encoder_config()->
                    set_width(camera_config_.input_config().width());
        camera_config_.mutable_encoder_config()->
                    mutable_h264_encoder_config()->
                    set_height(camera_config_.input_config().height());
      }
      #endif
      if (!encoder_->init(camera_config_.encoder_config())) {
          LOG(FATAL) << "[" << get_thread_name() << "] Failed to init camera encoder: "
                    << camera_config_.encoder_config().name();
        return false;
      }
    }

#ifdef WITH_ROS2
    encode_image_msg_.header = image_msg_.header;
    encode_mask_image_.header = image_msg_.header;
    if (camera_config_.encoder_config().name().compare("H264Encoder") == 0) {
        encode_image_msg_.format = "h264";
    } else {
        encode_image_msg_.format = "jpeg";
        encode_mask_image_.format = "jpeg";
    }
    before_encoder_rgb_ = cv::Mat(image_msg_.height, image_msg_.width,
            CV_8UC3);
#else
    proto_encode_image_ = std::make_shared<Image2>();
    proto_encode_image_->CopyFrom(*proto_image_);
    if (camera_config_.encoder_config().name().compare("H264Encoder") == 0) {
        proto_encode_image_->set_compression(Image2_Compression_H264);
    } else {
        proto_encode_image_->set_compression(Image2_Compression_JPEG);
    }
    before_encoder_rgb_ = cv::Mat(camera_config_.input_config().height(),
                                  camera_config_.input_config().width(), CV_8UC3);
#endif
    return true;
}

bool Camera::init_input() {
    input_ = nullptr;
    if (camera_config_.has_input_config()) {
        camera_config_.mutable_input_config()->set_frame_id(config_.frame_id());
        input_ = CameraInputFactory::get(camera_config_.input_config().name());
        if (!input_) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to get camera input ptr: "
                          << camera_config_.input_config().name();
            return false;
        }

        if (!input_->init(camera_config_.input_config())) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to init camera input: "
                      << camera_config_.input_config().name();
            return false;
        }
    }

    return true;
}

bool Camera::init_undistortion() {
    CameraSensorConfig sensor_config;
    if (config_.has_name()) {
        load_sensor_config(sensor_config);
        init_camera_config(sensor_config, camera_config_);
    }
    undistortion_ = nullptr;
    if (camera_config_.has_undistortion_config()) {
        camera_config_.mutable_undistortion_config()->set_frame_id(config_.frame_id());
        undistortion_ = UndistortionFactory::get(camera_config_.undistortion_config().name());
        if (!undistortion_) {
            LOG(FATAL) << "[" << get_thread_name() << "] failed to get undistortion ptr: "
                       << camera_config_.undistortion_config().name();
            return false;
        }

        if (!undistortion_->init(sensor_config)) {
            LOG(FATAL) << "[" << get_thread_name() << "] failed to init undistortion: "
                       << camera_config_.undistortion_config().name();
            return false;
        }
    }

    return true;
}

void Camera::load_sensor_config(CameraSensorConfig& sensor_config) {
    // todo load undistortion param
}

void Camera::init_camera_config(const CameraSensorConfig& sensor_config, CameraConfig& config) {
    // todo no need if no need to undistortion
    // config.mutable_input_config()->set_width(sensor_config.img_width_);
    // config.mutable_input_config()->set_height(sensor_config.img_height_);
    // config.mutable_input_config()->set_offset_x(sensor_config.offset_x_);
    // config.mutable_input_config()->set_offset_y(sensor_config.offset_y_);
    // config.mutable_input_config()->set_sensor_width(sensor_config.sensor_width_);
    // config.mutable_input_config()->set_sensor_height(sensor_config.sensor_height_);
}

void Camera::stop() {
    stop_ = true;
    input_->stop();
}

void Camera::dill_raw_image(const std::shared_ptr<const CameraRawData>& raw_data,
        const uint32_t& offset, const uint32_t& one_image_size) {
#ifdef WITH_ROS2
    image_msg_.header.stamp.sec = raw_data->utime_ / 1000000;
    int usec = raw_data->utime_ % 1000000;
    image_msg_.header.stamp.nanosec = usec * 1000;
    image_msg_.encoding = raw_data->data_type;
    image_msg_.data.resize(one_image_size);
    std::memcpy(image_msg_.data.data(), raw_data->image_.data + offset, one_image_size);
#else
    proto_image_->mutable_header()->set_camera_timestamp(raw_data->utime_);
    proto_image_->mutable_header()->set_sequence_num(image_seq_++);
    proto_image_->set_exposuretime(raw_data->exposure_time_);
    proto_image_->set_type(raw_data->data_type);
    proto_image_->set_data(raw_data->image_.data + offset, one_image_size);
#endif
}

bool Camera::dill_encode_image(const std::shared_ptr<const CameraRawData>& raw_data,
        const uint32_t& offset) {
    unsigned char* compress_buffer;
    int32_t compress_buffer_size = 0;
    if (raw_data->data_type.compare("Y") == 0) {
        cv::Mat y(camera_config_.input_config().height(),
                    camera_config_.input_config().width(), CV_8UC1,
                    reinterpret_cast<void*>(raw_data->image_.data + offset));
        cv::cvtColor(y, before_encoder_rgb_, cv::COLOR_GRAY2RGB);
        compress_buffer_size = encoder_->encode(before_encoder_rgb_, &compress_buffer);
    } else if (raw_data->data_type.compare("NV12") == 0) {
        cv::Mat nv12(camera_config_.input_config().height() * 3 / 2,
                    camera_config_.input_config().width(),
                    CV_8UC1, reinterpret_cast<void*>(raw_data->image_.data + offset));
        if (camera_config_.encoder_config().name().compare("H264Encoder") == 0) {
            #ifdef WITH_TDA4
            compress_buffer_size = encoder_->encode(nv12, &compress_buffer);
            #endif
        } else {
            cv::cvtColor(nv12, before_encoder_rgb_, cv::COLOR_YUV2BGR_NV12);
            compress_buffer_size = encoder_->encode(before_encoder_rgb_, &compress_buffer);
        }
    } else if (raw_data->data_type.compare("RGB8") == 0) {
        cv::Mat rgb(camera_config_.input_config().height(),
                    camera_config_.input_config().width(),
                    CV_8UC3, reinterpret_cast<void*>(raw_data->image_.data + offset));
        compress_buffer_size = encoder_->encode(rgb, &compress_buffer);
    } else {
        LOG(ERROR) << "[" << get_thread_name() << "] type: "
                << raw_data->data_type << " not support.";
        return false;
    }
    if (compress_buffer_size == 0) {
        LOG(ERROR) << "[" << get_thread_name() << "] encode failed.";
        return false;
    }
#ifdef WITH_ROS2
    encode_image_msg_.header.stamp.sec = raw_data->utime_ / 1000000;
    int usec = raw_data->utime_ % 1000000;
    encode_image_msg_.header.stamp.nanosec = usec * 1000;
    encode_image_msg_.data.resize(compress_buffer_size);
    std::memcpy(encode_image_msg_.data.data(), compress_buffer, compress_buffer_size);
#else
    proto_encode_image_->mutable_header()->set_camera_timestamp(raw_data->utime_);
    proto_encode_image_->mutable_header()->set_sequence_num(encode_image_seq_++);
    proto_encode_image_->set_exposuretime(raw_data->exposure_time_);
    proto_encode_image_->set_data(compress_buffer, compress_buffer_size);
    proto_encode_image_->set_type(raw_data->data_type);
#endif
    return true;
}

void Camera::dill_mask_image(const std::shared_ptr<const CameraRawData>& raw_data,
        const uint32_t& offset, const uint32_t& one_image_size) {
#ifdef WITH_ROS2
    encode_mask_image_.header.stamp.sec = raw_data->utime_ / 1000000;
    int usec = raw_data->utime_ % 1000000;
    encode_mask_image_.header.stamp.nanosec = usec * 1000;
    encode_mask_image_.data.resize(one_image_size);
    std::memcpy(encode_mask_image_.data.data(), raw_data->image_.data + offset, one_image_size);
#endif
}

#ifdef WITH_ROS2
void Camera::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t level,
                                const std::string& custom_desc, const std::string& context) {
    auto diagnose_input = DiagnoseInput(ModuleType::SENSOR_CAMERA, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}
#else
void Camera::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const Level& level,
                                const std::string& custom_desc, const std::string& context) {
    auto diagnose_input = DiagnoseInput(ModuleType::SENSOR_CAMERA, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}
#endif

void Camera::diagnose_images(
    const std::shared_ptr<const CameraRawData>& raw_data, const uint32_t& offset,
    const uint32_t& height, const uint32_t& width, const int32_t& ret) {
  if (image_count_ == (diagnose_end_count_ - 1)) {
    diagnose_executed_ = true;
  }
  cv::Mat raw_data_nv12(static_cast<int>(height * 1.5), static_cast<int>(width), CV_8UC1,
    reinterpret_cast<void*>(raw_data->image_.data + static_cast<int>(offset)));
  cv::Mat raw_data_bgr;
  cv::cvtColor(raw_data_nv12, raw_data_bgr, cv::COLOR_YUV2BGR_NV12);
  camera_diagnoser_->diagnose_an_image(raw_data_bgr);
  DeviceStatus cur_status = camera_diagnoser_->get_device_status();
  std::string cur_description = camera_diagnoser_->get_status_description();
  if (!cur_description.empty()) {
    LOG(ERROR) << "[" << get_thread_name() << "] " << cur_description;
    send_diagnose_input(
        sensor_position_id_, DeviceStatus(cur_status),
        Level::ERROR, "camera_driver_abnormal", cur_description);
    flag_status_ = false;
    return;
  }
}

#ifdef WITH_ROS2
#ifdef WITH_TDA4
bool Camera::calibrate(std::shared_ptr<const CameraRawData>& raw_data,
                       std::shared_ptr<CamIntrinsicParam>& camera_intrinsic) {
  if (validate_calib_ && calibrate_process_->get_vehicle_mode() == 2) {
    if (!calibrate_process_->process(raw_data, camera_intrinsic)) {
      return false;
    }
  }
  calibrate_process_->set_calib_end();
  return true;
}
#endif
#endif

#ifdef WITH_ROS2
void Camera::write_data_intermittently(uint32_t last_data_fps_index,
                               uint32_t downsampling_each_frame,
                               std::string channel_name,
                               sensor_msgs::msg::CompressedImage& data) {
  if (last_data_fps_index % downsampling_each_frame == 0) {
    common::Singleton<CameraAFOutput>::get()->write_image(channel_name, data);
  }
}
#endif

void Camera::camera_start() {
  if (camera_config_.input_config().name() == "GstCamera" ||
      camera_config_.input_config().name() == "Ros2Input") {
    while (!input_->start()) {
      LOG(ERROR) << "[" << get_thread_name() << "] " << " Failed to start.";
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }
  } else {
    while (!input_->start(cam_intrinsic_param_)) {
      LOG(ERROR) << "[" << get_thread_name() << "] " << " Failed to start.";
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }
  }
}

void Camera::send_mask_topic(std::shared_ptr<const CameraRawData>& raw_data,
                             uint32_t one_image_size, std::string& topic_name) {
  for (auto i = 0; i < raw_data->image_number; ++i) {
    auto offset = i * one_image_size;
    dill_mask_image(raw_data, offset, one_image_size);
#ifdef WITH_ROS2
    common::Singleton<CameraAFOutput>::get()->write_image(topic_name + "_by_sensor",
                                                                encode_mask_image_);
#endif
  }
}

void Camera::send_raw_data(std::shared_ptr<const CameraRawData>& raw_data,
                            uint32_t one_image_size, int32_t ret) {
  if (config_.has_channel_name()) {
    for (auto i = 0; i < raw_data->image_number; ++i) {
        auto offset = i * one_image_size;
        dill_raw_image(raw_data, offset, one_image_size);
#ifdef WITH_ROS2
        common::Singleton<CameraAFOutput>::get()->write_image(
          config_.channel_name() + "_" + std::to_string(i), image_msg_);
#else
        common::Singleton<CameraCyberOutput>::get()->write_image(
          config_.channel_name() + "_" + std::to_string(i), proto_image_);
#endif
        if (!diagnose_executed_ && config_.has_diagnose_config_file()) {
          uint32_t height = static_cast<uint32_t>(camera_config_.input_config().height());
          uint32_t width = static_cast<uint32_t>(camera_config_.input_config().width());
          if (image_count_ < diagnose_end_count_ && image_count_ >= diagnose_begin_count_) {
            diagnose_images(raw_data, offset, height, width, ret);
          }
          image_count_++;
        }
      }
      if (flag_status_) {
        send_diagnose_input(sensor_position_id_, DeviceStatus(ret), Level::NOERR,
                                "camera_driver_normal", "dill_raw_image_normal");
        flag_status_ = true;
      }
  }

  if (config_.has_channel_encode_name() && encoder_) {
    for (auto i = 0; i < raw_data->image_number; ++i) {
        auto offset = i * one_image_size;
        if (!dill_encode_image(raw_data, offset)) {
            continue;
        }
#ifdef WITH_ROS2
        write_data_intermittently(image_fps_index_,
                                encode_image_downsampling_each_image_frame_,
                                config_.channel_encode_name() + "_" +std::to_string(i),
                                encode_image_msg_);
        image_fps_index_ = (image_fps_index_ + 1) %
                            encode_image_downsampling_each_image_frame_;
#else
        common::Singleton<CameraCyberOutput>::get()->write_image(
          config_.channel_encode_name() + "_" + std::to_string(i), proto_encode_image_);
#endif
    }
    send_diagnose_input(sensor_position_id_, DeviceStatus(ret), Level::NOERR,
                        "camera_driver_normal", "dill_encode_image_normal");
  }
}

void Camera::run() {
  init_undistortion();
  init_encoder();
  init_input();
  camera_start();
  int32_t ret = 0;
  while (!stop_) {
    std::shared_ptr<const CameraRawData> raw_data;
    ret = input_->get_camera_data(&raw_data);
    if (ret < 0) {
        LOG(ERROR) << "[" << get_thread_name() << "] " << " Failed to get raw data. code: "
                    << std::to_string(ret);
        send_diagnose_input(sensor_position_id_, DeviceStatus(ret), Level::ERROR,
                        "camera_driver_abnormal", "input_data_abnormal");
        continue;
    }

#if defined(WITH_ROS2) && defined(WITH_TDA4)
    if (!calibrate(raw_data, cam_intrinsic_param_)) {
      continue;
    }
#endif

    auto one_image_size = raw_data->data_size / raw_data->image_number;
    std::string topic_name;
    if (input_->get_topic_name(&topic_name, raw_data)) {
        send_mask_topic(raw_data, one_image_size, topic_name);
    } else {
        send_raw_data(raw_data, one_image_size, ret);
    }
  }
  input_->release_camera_data();
}

}  // namespace hub
}  // namespace sensor
