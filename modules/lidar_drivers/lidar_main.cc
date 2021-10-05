// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar main

#include <gflags/gflags.h>
#include "common/common.h"
#include "lidar_drivers/output/cyber_output.h"
#include "lidar_drivers/lidar.h"

#define MODULE "LidarDriver"
DEFINE_string(config_file, "params/drivers/lidar/test/lidar_config.prototxt",
              "path of config file");

namespace crdc {
namespace airi {
namespace lidar {

int main(int argc, char* argv[]) {
    // gflags command setting
    google::ParseCommandLineFlags(&argc, &argv, true);

    if (!std::getenv("CRDC_WS")) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC_WS not setting!";
    } else {
        LOG(INFO) << "[LIDAR_MAIN] Current CRDC_WS: " << std::string(std::getenv("CRDC_WS"));
    }

    FLAGS_colorlogtostderr = true;
    FLAGS_minloglevel = 0;
    FLAGS_v = 0;
    FLAGS_stderrthreshold = 3;
    // log on screen
    // FLAGS_alsologtostderr = true;

    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<LidarCyberOutput>::get()->init(MODULE);

    LidarComponent lidar_component;

    std::string config = "";

    if (argc < 2) {
        LOG(WARNING) << "[LIDAR_MAIN] No lidar proto file given. Use default proto config.";
        config = std::string(std::getenv("CRDC_WS")) + '/' + FLAGS_config_file;
    } else {
        config = std::string(argv[1]);
    }

    if (!crdc::airi::util::is_path_exists(config)) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC not exists, please setup environment first.";
        return 1;
    }

    if (!crdc::airi::util::get_proto_from_file(config, &lidar_component)) {
        LOG(FATAL) << "[LIDAR_MAIN] failed to read lidar config proto.";
        return 1;
    }

    LOG(INFO) << lidar_component.DebugString();

    std::vector<std::shared_ptr<crdc::airi::Lidar>> lidars;
    for (const auto& config : lidar_component.component_config()) {
        std::shared_ptr<crdc::airi::Lidar> lidar = std::make_shared<crdc::airi::Lidar>(config);
        lidars.emplace_back(lidar);
    }

    for (const auto& lidar : lidars) {
        lidar->start();
    }

    LOG(INFO) << "[LIDAR_MAIN] lidar_driver started.";
    apollo::cyber::WaitForShutdown();
    for (const auto& lidar : lidars) {
        lidar->stop();
    }

    for (const auto& lidar : lidars) {
        if (lidar->is_alive()) {
            lidar->join();
        }
        LOG(INFO) << "[LIDAR_MAIN] lidar[" << lidar->get_name() << "] joined successfully.";
    }

    LOG(WARNING) << "[LIDAR_MAIN] lidar_driver terminated.";
    return 0;
}

}  // namespace lidar
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) { return crdc::airi::lidar::main(argc, argv); }
