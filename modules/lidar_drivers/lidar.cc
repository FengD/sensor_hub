// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar

#include "lidar_drivers/lidar.h"

namespace crdc {
namespace airi {

Lidar::Lidar(const Component& config) : common::Thread(true) {
    config_ = config;
}

void Lidar::stop() {

}

void Lidar::run() {

}

}  // namespace airi
}  // namespace crdc
