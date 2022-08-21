// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera
// Contributor: shichong.wang

#include <omp.h>
#include "camera_drivers/camera.h"
#include "camera_drivers/output/cyber_output.h"

namespace crdc {
namespace airi {

Camera::Camera(const CameraComponentConfig& config) : common::Thread(true) {
    config_ = config;
    stop_ = false;
    camera_name_ = config_.frame_id();
    sensor_position_id_ = config_.sensor_position_id();
    image_seq_ = 0;
    encode_image_seq_ = 0;
    std::string thread_name = camera_name_;
    if (thread_name.length() > MAX_THREAD_NAME_LENGTH) {
        thread_name = thread_name.substr(0, MAX_THREAD_NAME_LENGTH - 1);
    }
    set_thread_name(thread_name);

    if (config_.has_priotiry()) {
        set_priority(config_.priotiry());
    }

    std::string config_file_path = std::string(std::getenv("CRDC_WS"))
                                   + '/' + config_.config_file();
    if (!crdc::airi::util::is_path_exists(config_file_path)) {
        LOG(FATAL) << "[" << get_thread_name() << "] proto file not exits: "
                   << config_file_path;
        return;
    }

    if (!crdc::airi::util::get_proto_from_file(config_file_path,
                                              &camera_config_)) {
        LOG(FATAL) << "[" << get_thread_name() << "] failed to read camera proto config: "
                   << config_file_path;
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
    proto_image_ = std::make_shared<Image2>();
    proto_image_->mutable_header()->set_frame_id(config_.frame_id());
    proto_image_->set_width(camera_config_.input_config().width());
    proto_image_->set_height(camera_config_.input_config().height());
    proto_image_->mutable_data()->reserve(camera_config_.input_config().width() *
                                          camera_config_.input_config().height() * 3);
    proto_image_->set_step(3 * camera_config_.input_config().width());
    proto_image_->set_compression(Image2_Compression_RAW);
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

    proto_encode_image_ = std::make_shared<Image2>();
    proto_encode_image_->CopyFrom(*proto_image_);
    proto_encode_image_->set_compression(Image2_Compression_JPEG);
    before_encoder_rgb_ = cv::Mat(camera_config_.input_config().height(),
                                  camera_config_.input_config().width(), CV_8UC3);
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
    proto_image_->mutable_header()->set_camera_timestamp(raw_data->utime_);
    proto_image_->mutable_header()->set_sequence_num(image_seq_++);
    proto_image_->set_exposuretime(raw_data->exposure_time_);
    proto_image_->set_type(raw_data->data_type);
    proto_image_->set_data(raw_data->image_.data + offset, one_image_size);
}

bool Camera::dill_encode_image(const std::shared_ptr<const CameraRawData>& raw_data,
                                  const uint32_t& offset) {
    unsigned char* compress_buffer;
    if (raw_data->data_type == "Y") {
        cv::Mat y(camera_config_.input_config().height(),
                camera_config_.input_config().width(), CV_8UC1,
                reinterpret_cast<void*>(raw_data->image_.data + offset));
        cv::cvtColor(y, before_encoder_rgb_, cv::COLOR_GRAY2RGB);
    } else if (raw_data->data_type == "NV12") {
        cv::Mat nv12(camera_config_.input_config().height() * 3 / 2,
                    camera_config_.input_config().width(),
                    CV_8UC1, reinterpret_cast<void*>(raw_data->image_.data + offset));
        cv::cvtColor(nv12, before_encoder_rgb_, cv::COLOR_YUV2RGB_NV12);
    } else {
        LOG(ERROR) << "[" << get_thread_name() << "] type: "
                << raw_data->data_type << " not support.";
        return false;
    }
    int32_t compress_buffer_size = encoder_->encode(before_encoder_rgb_, &compress_buffer);
    if (compress_buffer_size == 0) {
        LOG(ERROR) << "[" << get_thread_name() << "] encode failed.";
        return false;
    }
    proto_encode_image_->mutable_header()->set_camera_timestamp(raw_data->utime_);
    proto_encode_image_->mutable_header()->set_sequence_num(encode_image_seq_++);
    proto_encode_image_->set_exposuretime(raw_data->exposure_time_);
    proto_encode_image_->set_data(compress_buffer, compress_buffer_size);
    proto_encode_image_->set_type(raw_data->data_type);
    return true;
}

void Camera::run() {
    while (!input_->start()) {
        LOG(ERROR) << "[" << get_thread_name() << "] " << " Failed to start.";
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
            LOG(ERROR) << "[" << get_thread_name() << "] "
                       << " Failed to get raw data. code: " << std::to_string(ret);
            continue;
        }

        if (ret == 0) {
            continue;
        }

        LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [get_raw_data] elapsed_time(us): "
                  << get_now_microsecond() - start_time;
        start_time = get_now_microsecond();
        // No need now
        // if (!undistortion_->process(raw_data->image_, image_undistorted_)) {
        //     continue;
        // }
        LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [undistortion] elapsed_time(us): "
                    << get_now_microsecond() - start_time;
        auto one_image_size = raw_data->data_size / raw_data->image_number;
        if (config_.has_channel_name()) {
            start_time = get_now_microsecond();
            for (auto i = 0; i < raw_data->image_number; i++) {
                auto offset = i * one_image_size;
                dill_raw_image(raw_data, offset, one_image_size);
                common::Singleton<CameraCyberOutput>::get()->write_image(
                    config_.channel_name() + "/" + std::to_string(i), proto_image_);
            }
            LOG(INFO) << "[" << get_thread_name() << "] [TIMER] [raw] elapsed_time(us): "
                << get_now_microsecond() - start_time;
        }

        if (config_.has_channel_encode_name() && encoder_) {
            start_time = get_now_microsecond();
            for (auto i = 0; i < raw_data->image_number; i++) {
                auto offset = i * one_image_size;
                if (!dill_encode_image(raw_data, offset)) {
                    continue;
                }
                common::Singleton<CameraCyberOutput>::get()->write_image(
                    config_.channel_encode_name() + "/" + std::to_string(i), proto_encode_image_);
            }
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
