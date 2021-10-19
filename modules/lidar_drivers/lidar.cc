// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar

#include "lidar_drivers/lidar.h"
#include "lidar_drivers/output/cyber_output.h"

namespace crdc {
namespace airi {

Lidar::Lidar(const LidarComponentConfig& config) : common::Thread(true) {
    config_ = config;
    stop_ = false;
    lidar_name_ = config_.frame_id();
    sensor_position_id_ = config_.sensor_position_id();
    std::string thread_name = lidar_name_;
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
                                               &lidar_config_)) {
        LOG(FATAL) << "[" << get_thread_name() << "] failed to read lidar proto config: "
                   << config_file_path;
        return;
    }

    LOG(INFO) << "[" << get_thread_name() << "] create Lidar Instance: "
              << config_.DebugString() << lidar_config_.DebugString();
    if (!init_parser() || !init_input()) {
        return;
    }
}

bool Lidar::init_parser() {
    parser_ = nullptr;
    if (lidar_config_.has_parser_config()) {
        lidar_config_.mutable_parser_config()->set_frame_id(config_.frame_id());
        parser_ = LidarParserFactory::get(lidar_config_.parser_config().name());
        if (!parser_) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to get lidar parser ptr: "
                       << lidar_config_.parser_config().name();
            return false;
        }

        if (!parser_->init(lidar_config_.parser_config())) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to init lidar parser: "
                       << lidar_config_.parser_config().name();
            return false;
        }
    }

    return true;
}

bool Lidar::init_input() {
    input_ = nullptr;
    if (lidar_config_.has_input_config()) {
        lidar_config_.mutable_input_config()->set_frame_id(config_.frame_id());
        input_ = LidarInputFactory::get(lidar_config_.input_config().name());
        if (!input_) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to get lidar input ptr: "
                       << lidar_config_.input_config().name();
            return false;
        }

        if (!input_->init(lidar_config_.input_config())) {
            LOG(FATAL) << "[" << get_thread_name() << "] Failed to init lidar input: "
                       << lidar_config_.input_config().name();
            return false;
        }
    }

    return true;
}

void Lidar::stop() {
    LOG(WARNING) << "[" << get_thread_name() << "] lidar stop.";
    stop_ = false;
}

void Lidar::run() {
    while (!stop_) {
        Packet* raw_packet;
        int32_t code = input_->get_lidar_data(&raw_packet);
        if (code != DeviceStatus::SUCCESS) {
            LOG(WARNING) << "[" << get_thread_name() << "] failed to get lidar packet.";
        } else {
            if (raw_packet->port() == lidar_config_.input_config().lidar_port()) {
                std::shared_ptr<LidarPointCloud> cloud;
                if (parser_->parse_lidar_packet(raw_packet, &cloud)) {
                    auto now = get_now_microsecond();
                    auto receive_time = now - cloud->proto_cloud_->header().lidar_timestamp();
                    if (now < cloud->proto_cloud_->header().lidar_timestamp()) {
                        receive_time = cloud->proto_cloud_->header().lidar_timestamp() - now;
                    }
                    LOG(INFO) << "[" << get_thread_name()
                              << "] [TIMER] [receive] elapsed_time(us): " << receive_time;
                    cloud->proto_cloud_->mutable_header()->set_frame_id(config_.frame_id());
                    now = get_now_microsecond();
                    if (callback_ != nullptr) {
                        callback_(cloud->proto_cloud_);
                    }
                    LOG(INFO) << "[" << get_thread_name()
                              << "] [TIMER] [callback] elapsed_time(us): "
                              << get_now_microsecond() - now;
                    if (config_.has_channel_name()) {
                        common::Singleton<LidarCyberOutput>::get()->write_cloud(
                            config_.channel_name(), cloud->proto_cloud_);
                    }
                }
            }
        }

        if (input_->is_packet_pool_full()) {
            std::shared_ptr<Packets> packets;
            input_->get_lidar_packets(&packets);
            if (config_.has_raw_data_channel_name()) {
                common::Singleton<LidarCyberOutput>::get()->write_packet(
                    config_.raw_data_channel_name(), packets);
            }
            input_->clear_pool();
        }
    }
}

}  // namespace airi
}  // namespace crdc
