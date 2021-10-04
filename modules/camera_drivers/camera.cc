// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera

#include "camera_drivers/camera.h"
#include "camera_drivers/output/cyber_output.h"

namespace crdc {
namespace airi {

Camera::Camera(const CameraComponentConfig& config) : common::Thread(true) {
    config_ = config;
    stop_ = false;
    camera_name_ = config_.frame_id();
    sensor_position_id_ = config_.sensor_position_id();
    std::string thread_name = camera_name_;
    if (thread_name.length() > MAX_THREAD_NAME_LENGTH) {
        thread_name = thread_name.substr(0, MAX_THREAD_NAME_LENGTH - 1);
    }
    set_thread_name(thread_name);

    if (config_.has_priotiry()) {
        set_priority(config_.priotiry());
    }

    if (crdc::airi::util::get_proto_from_file(config_.config_file(),
                                              &camera_config_)) {
        LOG(FATAL) << "[" << get_thread_name() << "] failed to read camera proto config: "
                   << config_.config_file() << std::endl;
        return;
    }

    init_proto_image();

    if (!init_undistortion() || !init_encoder() || !init_input()) {
        return;
    }
}

void Camera::init_proto_image() {
    LOG(INFO) << "[" << get_thread_name() << "] create Camera Instance: " 
              << config_.DebugString() << camera_config_.DebugString();
    proto_image_ = std::make_shared<Image>();
    proto_image_->set_frame_id(config_.frame_id());
    proto_image_->set_width(camera_config_.input_config().width());
    proto_image_->set_height(camera_config_.input_config().height());
    proto_image_->mutable_data()->reserve(camera_config_.input_config().width() * 
                                          camera_config_.input_config().height() * 3);
    proto_image_->set_step(3 * camera_config_.input_config().width());
}

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

        if (!encoder_->init(camera_config_.encoder_config())) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to init camera encoder: "
                    << camera_config_.encoder_config().name();
            return false;
        }
    }

    proto_encode_image_ = std::make_shared<Image>();
    proto_encode_image_->CopyFrom(*proto_image_);
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
    config.mutable_input_config()->set_width(sensor_config.img_width_);
    config.mutable_input_config()->set_height(sensor_config.img_height_);
    config.mutable_input_config()->set_offset_x(sensor_config.offset_x_);
    config.mutable_input_config()->set_offset_y(sensor_config.offset_y_);
    config.mutable_input_config()->set_sensor_width(sensor_config.sensor_width_);
    config.mutable_input_config()->set_sensor_height(sensor_config.sensor_height_);
}

void Camera::stop() {
    stop_ = true;
    input_->stop();
}

void Camera::run() {
    while (!input_->start()) {
        LOG(ERROR) << "[" << get_thread_name() << "] " << camera_name_ << " Failed to start.";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue; 
    }

    uint64_t start_time = 0;
    uint64_t full_start_time = 0;
    int32_t ret = 0;
    while (!stop_) {
        std::shared_ptr<const CameraRawData> raw_data;
        start_time = get_now_microsecond();
        full_start_time = start_time;
        ret = input_->get_camera_data(&raw_data);
        if (ret < 0) {
            LOG(ERROR) << "[" << get_thread_name() << "] " << camera_name_
                       << " Failed to get raw data. code: " << std::to_string(ret);
        }

        if (ret == 0) {
            continue;
        }

        LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [get_raw_data] elapsed_time(us): "
                  << get_now_microsecond() - start_time;
        start_time = get_now_microsecond();
        if (!undistortion_->process(raw_data->image_, image_undistorted_)) {
            continue;
        }
        
        if (config_.has_channel_name()) {
            LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [undistortion] elapsed_time(us): "
                      << get_now_microsecond() - start_time;
            // publish raw data
            proto_image_->mutable_header()->set_camera_timestamp(raw_data->utime_);
            proto_image_->mutable_header()->set_frame_id(config_.frame_id());
            proto_image_->mutable_header()->set_module_name("CameraDrivers");
            proto_image_->set_measurement_time(raw_data->exposure_time_);
            proto_image_->set_data(image_undistorted_.data, 
                                   camera_config_.input_config().width() * 
                                   camera_config_.input_config().height() * 3);
            common::Singleton<CameraCyberOutput>::get()->write_image(config_.channel_name(), 
                                                                     proto_image_);
            LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [total] elapsed_time(us): "
                      << get_now_microsecond() - start_time;
        }

        if (encoder_) {
            start_time = get_now_microsecond();
            unsigned char* compress_buffer;
            int32_t compress_buffer_size = encoder_->encode(image_undistorted_, &compress_buffer);
            if (compress_buffer_size == 0) {
                LOG(ERROR) << "[" << get_thread_name() << "] encode failed.";
                continue;
            }
            // publish encode data
            proto_encode_image_->mutable_header()->set_camera_timestamp(raw_data->utime_);
            proto_encode_image_->mutable_header()->set_frame_id(config_.frame_id());
            proto_encode_image_->mutable_header()->set_module_name("CameraDrivers");
            proto_encode_image_->set_measurement_time(raw_data->exposure_time_);
            proto_encode_image_->set_data(compress_buffer, compress_buffer_size);
            common::Singleton<CameraCyberOutput>::get()->write_image(config_.channel_encode_name(), 
                                                                     proto_image_);
            LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [encode] elapsed_time(us): "
                      << get_now_microsecond() - start_time;
        }

        input_->release_camera_data();
        LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [full] elapsed_time(us): "
                  << get_now_microsecond() - full_start_time;
    }
}

}  // namespace airi
}  // namespace crdc
