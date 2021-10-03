// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera

#include "camera_drivers/camera.h"

namespace crdc {
namespace airi {

Camera::Camera(const CameraComponentConfig& config) : common::Thread(true) {
    config_ = config;
}

void Camera::stop() {

}

void Camera::run() {

}

}  // namespace airi
}  // namespace crdc
