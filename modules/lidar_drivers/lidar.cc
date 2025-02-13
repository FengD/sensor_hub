// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar

#include "lidar_drivers/lidar.h"

namespace sensor {
namespace hub {

Lidar::Lidar(const LidarComponentConfig& config) : common::Thread(true) {
    config_ = config;
    stop_ = false;
    lidar_name_ = config_.frame_id();
    sensor_position_id_ = config_.sensor_position_id();
    data_fps_index_ = 0;
    last_raw_data_fps_index_ = 0;
    last_cloud_data_fps_index_ = 0;
    if (config_.has_cloud_data_downsampling_each_cloud_frame()) {
        cloud_data_downsampling_each_cloud_frame_ =
            config_.cloud_data_downsampling_each_cloud_frame();
    } else {
        cloud_data_downsampling_each_cloud_frame_ = 1;
    }
    if (config_.has_raw_data_downsampling_each_cloud_frame()) {
        raw_data_downsampling_each_cloud_frame_ =
            config_.raw_data_downsampling_each_cloud_frame();
    } else {
        raw_data_downsampling_each_cloud_frame_ = 1;
    }
    dynamic_calibrate_id_ = config_.dynamic_calibrate_id();
    if (config_.has_calibrate()) {
        iftransform_ = config_.calibrate();
    } else {
        iftransform_ = false;
    }
#ifdef WITH_TDA4
    getserver_ = false;
#else
    getserver_ = true;
#endif
    if (get_iftransform()) {
        compensator_.init(config_);
        dynamic_calibrate_init_ = false;
    }
    std::string thread_name = lidar_name_;
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

#ifdef WITH_ROS2
void Lidar::init_dyncalibrate(const std::string& frame_id) {
#ifdef WITH_TDA4
    if (dynamic_calibrate_id_ == 2) {
        dynamic_calibrate_mode_->dynamic_init(frame_id);
    }
#else
    if (dynamic_calibrate_id_ == 1) {
        dynamic_calibration_mode_.init(frame_id);
    }
#endif
}

void Lidar::dynamic_calibrate(std::shared_ptr<LidarPointCloud>& cloud) {
#ifdef WITH_TDA4
    if (dynamic_calibrate_id_ == 2 && dynamic_calibrate_mode_->get_vehicle_mode() == 2 &&
        dynamic_calibrate_mode_->get_calib_mode() == 2) {
        int ret = dynamic_calibrate_mode_->dynamic_process(cloud);
        if (ret == 0) {
            dynamic_calibrate_mode_->set_dynamic_calibrate_end();
        }
    }
#else
    if (dynamic_calibrate_id_ == 1) {
        dynamic_calibration_mode_.process(cloud);
    }
#endif
}
#endif

void Lidar::stop() {
    LOG(WARNING) << "[" << get_thread_name() << "] lidar stop.";
    stop_ = false;
}

#ifdef WITH_ROS2
void Lidar::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const uint8_t& level,
                                const std::string& custom_desc, const std::string& context) {
    // app lidar_drivers is driver of both lidar and radar, so the sensor type should be specified
    ModuleType module_type = SENSOR_LIDAR;  // default type is lidar
    if ("RADAR" == lidar_name_.substr(0, lidar_name_.find_first_of("_")) ||
        "MRR4D" == lidar_name_.substr(0, lidar_name_.find_first_of("_"))) {
        module_type = SENSOR_RADAR;
    }
    auto diagnose_input = DiagnoseInput(module_type, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}
#else
void Lidar::send_diagnose_input(const uint32_t& position_id,
                                const DeviceStatus& error, const Level& level,
                                const std::string& custom_desc, const std::string& context) {
    // app lidar_drivers is driver of both lidar and radar, so the sensor type should be specified
    ModuleType module_type = SENSOR_LIDAR;  // default type is lidar
    if ("RADAR" == lidar_name_.substr(0, lidar_name_.find_first_of("_")) ||
        "MRR4D" == lidar_name_.substr(0, lidar_name_.find_first_of("_"))) {
        module_type = SENSOR_RADAR;
    }
    auto diagnose_input = DiagnoseInput(module_type, position_id,
                                      error, level, custom_desc, context);
    diagnose_inputs_.emplace_back(std::move(diagnose_input));
    common::Singleton<ModuleDiagnose>::get()->diagnose_list(diagnose_inputs_);
}
#endif

void Lidar::verify_emptiness_and_send_diagnose(const int32_t code,
                                               const std::shared_ptr<LidarPointCloud>& cloud) {
#ifdef WITH_ROS2
    bool is_empty_cloud =
        cloud->cloud_msg_->width * cloud->cloud_msg_->height < 10;
#else
    bool is_empty_cloud =
        cloud->proto_cloud_->width() * cloud->proto_cloud_->height() < 10;
#endif
    if (is_empty_cloud) {
        send_diagnose_input(sensor_position_id_, DeviceStatus::DEVICE_ERROR, Level::ERROR,
                            "lidar_driver_normal", "input_data_abnormal");
    } else {
        send_diagnose_input(sensor_position_id_, DeviceStatus(code), Level::NOERR,
                            "lidar_driver_normal", "input_data_normal");
    }
}

template<typename T>
void Lidar::write_data_intermittently(int32_t &last_data_fps_index,
                                 int32_t downsampling_each_frame,
                                 std::string channel_name,
                                 T data,
                                 std::function<bool(const std::string&, const T&)> write_func) {
    if (downsampling_each_frame == 0) {
        LOG(FATAL) <<
            "[" << get_thread_name() << "] downsampling ratio each frame is zero.";
    }

    if (last_data_fps_index != data_fps_index_) {
        last_data_fps_index = data_fps_index_;
    }

    if (last_data_fps_index %
            downsampling_each_frame == 0) {
        write_func(channel_name, data);
    }
}

void Lidar::get_parser_lidar(Packet* raw_packet, int32_t code) {
#ifdef WITH_ROS2
    if (raw_packet->port == lidar_config_.input_config().lidar_port()) {
        std::shared_ptr<LidarPointCloud> cloud;
        if (parser_->parse_lidar_packet(raw_packet, &cloud)) {
            cloud->cloud_msg_->header.frame_id = config_.frame_id();
            if (callback_ != nullptr) {
                callback_(cloud);
            }
            if (get_iftransform()) {
                compensator_.cloud_transform(cloud);
                if (!dynamic_calibrate_init_ && getserver_) {
                    init_dyncalibrate(lidar_name_);
                    dynamic_calibrate_init_ = true;
                }
                if (dynamic_calibrate_init_) {
                    dynamic_calibrate(cloud);
                }
            }
            if (config_.has_channel_name()) {
                write_data_intermittently<std::shared_ptr<PointCloud2>>(
                                            last_cloud_data_fps_index_,
                                            cloud_data_downsampling_each_cloud_frame_,
                                            config_.channel_name(),
                                            cloud->cloud_msg_,
                                            std::bind(&LidarOutput::write_cloud, \
                                            common::Singleton<LidarOutput>::get(), \
                                            std::placeholders::_1, std::placeholders::_2));
            }
            data_fps_index_ = (data_fps_index_ + 1) % raw_data_downsampling_each_cloud_frame_;
            verify_emptiness_and_send_diagnose(code, cloud);
        }
    }
#else
    if (raw_packet->port() == lidar_config_.input_config().lidar_port()) {
        std::shared_ptr<LidarPointCloud> cloud;
        if (parser_->parse_lidar_packet(raw_packet, &cloud)) {
            cloud->proto_cloud_->mutable_header()->set_frame_id(config_.frame_id());
            if (callback_ != nullptr) {
                callback_(cloud);
            }
            if (get_iftransform()) {
                compensator_.cloud_transform(cloud);
            }
            if (config_.has_channel_name()) {
                write_data_intermittently<std::shared_ptr<PointCloud2>>(
                                            last_cloud_data_fps_index_,
                                            cloud_data_downsampling_each_cloud_frame_,
                                            config_.channel_name(),
                                            cloud->proto_cloud_,
                                            std::bind(&LidarOutput::write_cloud, \
                                            common::Singleton<LidarOutput>::get(), \
                                            std::placeholders::_1, std::placeholders::_2));
            }
            data_fps_index_++;
            verify_emptiness_and_send_diagnose(code, cloud);
        }
    }
#endif
}

void Lidar::run() {
    while (!stop_) {
      if (!get_flag()) {
        sleep(5);
        continue;
      }
        Packet* raw_packet;
        int32_t code = input_->get_lidar_data(&raw_packet);
        if (code != DeviceStatus::SUCCESS) {
            if (code == DeviceStatus::PCAP_END) {
                stop_ = true;
                LOG(FATAL) << "[" << get_thread_name() << "] the pcap file end.";
            }
            LOG(WARNING) << "[" << get_thread_name() << "] failed to get lidar packet.";
            send_diagnose_input(sensor_position_id_, DeviceStatus(code), Level::ERROR,
                                "lidar_driver_abnormal", "input_data_abnormal");
        } else {
            get_parser_lidar(raw_packet, code);
        }

        if (input_->is_packet_pool_full()) {
            std::shared_ptr<Packets> packets;
            input_->get_lidar_packets(&packets);
            if (config_.has_raw_data_channel_name()) {
                write_data_intermittently<std::shared_ptr<Packets>>(last_raw_data_fps_index_,
                                          raw_data_downsampling_each_cloud_frame_,
                                          config_.raw_data_channel_name(),
                                          packets,
                                          std::bind(&LidarOutput::write_packet, \
                                          common::Singleton<LidarOutput>::get(), \
                                          std::placeholders::_1, std::placeholders::_2));
            }
            input_->clear_pool();
        }
    }
}

}  // namespace hub
}  // namespace sensor
