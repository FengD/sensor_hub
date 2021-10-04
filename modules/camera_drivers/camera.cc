// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera

#include <boost/filesystem.hpp>
#include "camera_drivers/camera.h"

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

    boost::filesystem::path env_path(std::getenv("AIRI_ENV"));
    if (!boost::filesystem::exists(env_path)) {
        LOG(FATAL) << "[" << get_thread_name()
                   << "] AIRI_ENV not exist, please setup environment first." << std::endl;
        return;
    }

    if (crdc::airi::util::get_proto_from_file(env_path.string() + "/" + config_.config_file(),
                                              &camera_config_)) {
        LOG(FATAL) << "[" << get_thread_name() << "] failed to read camera proto config: "
                   << config_.config_file() << std::endl;
        return;
    }

    camera_config_.mutable_input_config()->set_frame_id(config_.frame_id());
}

void Camera::stop() {

}

void Camera::run() {

}

}  // namespace airi
}  // namespace crdc
