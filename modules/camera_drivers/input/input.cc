// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera input

#include <vector>
#include "camera_drivers/input/input.h"


namespace crdc {
namespace airi {

bool CameraInput::init(const CameraInputConfig& config) {
    is_running_.store(false);
    config_ = config;
    period_ = 1000000 / config_.fps();
    if (!camera_init()) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed init.";
        return false;
    }

    if (!init_pool()) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed to init raw data pool.";
        return false;
    }
    return true;
}

bool CameraInput::start(std::shared_ptr<CamIntrinsicParam>& camera_intrinsic) {
    if (is_running_.load()) {
        LOG(ERROR) << "[" << config_.frame_id() << "] already in operation.";
        return false;
    }

    if (!camera_start(camera_intrinsic)) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed to start.";
        return false;
    }

    is_running_.store(true);
    LOG(INFO) << "[" << config_.frame_id() << "] started.";
    return true;
}

bool CameraInput::start() {
    if (is_running_.load()) {
        LOG(ERROR) << "[" << config_.frame_id() << "] already in operation.";
        return false;
    }

    if (!camera_start()) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed to start.";
        return false;
    }

    is_running_.store(true);
    LOG(INFO) << "[" << config_.frame_id() << "] started.";
    return true;
}

bool CameraInput::stop() {
    if (!is_running_.load()) {
        LOG(WARNING) << "[" << config_.frame_id() << "] not in operation.";
        return false;
    }

    if (!camera_stop()) {
        LOG(ERROR) << "[" << config_.frame_id() << "] failed to stop.";
        return false;
    }

    is_running_.store(false);
    LOG(INFO) << "[" << config_.frame_id() << "] stopped.";
    return true;
}

bool CameraInput::init_pool() {
    raw_pool_.reset(new common::CCObjectPool<CameraRawData>(config_.pool_size()));
    raw_pool_->ConstructAll();
    std::vector<std::shared_ptr<CameraRawData>> raw_datas;
    for (auto i = 0; i < config_.pool_size(); ++i) {
        auto raw_data = raw_pool_->GetObject();
        if (raw_data == nullptr) {
            LOG(ERROR) << "[" << config_.frame_id() << "] failed to getobject:" << i;
            return false;
        }
        raw_data->image_ = cv::Mat(config_.height(), config_.width(), CV_8UC3);
        raw_datas.emplace_back(raw_data);
    }
    raw_datas.clear();
    return true;
}

std::shared_ptr<CameraRawData> CameraInput::get_raw_data(float exposure_time,
                                                         uint64_t utime,
                                                         unsigned char* data) {
    std::shared_ptr<CameraRawData> raw_data = raw_pool_->GetObject();
    if (raw_data == nullptr) {
        LOG(WARNING) << "[" << config_.frame_id() << "] raw pool failed to getobject, will be new.";
        raw_data = std::make_shared<CameraRawData>();
        raw_data->image_ = cv::Mat(config_.height(), config_.width(), CV_8UC3);
        // default rgb size = height * width * 3;
        raw_data->data_size = config_.height() * config_.width() * 3;
    }

    if (raw_data) {
        raw_data->exposure_time_ = exposure_time;
        raw_data->utime_ = utime;
        raw_data->image_.data = data;
    }
    return raw_data;
}

void CameraInput::matching_fps_by_sleep(const uint64_t& start, const uint64_t& end) {
    std::this_thread::sleep_for(std::chrono::microseconds(period_ - end + start));
}

}  // namespace airi
}  // namespace crdc
