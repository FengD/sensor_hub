// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: lidar main

#include <gflags/gflags.h>
#include "common/common.h"
#include "module_diagnose/module_diagnose.h"
#include "lidar_drivers/lidar_fusion.h"
#include "common/util.h"

#define MODULE "LidarDriver"
DEFINE_string(config_file, "params/drivers/lidar/test/lidar_config.prototxt",
              "path of config file");
DEFINE_bool(use_product_name, true, "use product_name to splicing config_file");

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

    apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
    apollo::cyber::Init(MODULE);
    common::Singleton<LidarCyberOutput>::get()->init(MODULE);

    LidarComponent lidar_component;
#ifdef WITH_TDA4
    std::string config;
    auto product_name = get_product_name();
    LOG(INFO) << "[" << MODULE << "] Product name is " << product_name;
    if (FLAGS_use_product_name) {
      config = std::string(std::getenv("CRDC_WS")) + "/params/drivers/lidar/" +
                product_name + "/lidar_config.prototxt";
    } else {
      config = std::string(std::getenv("CRDC_WS")) + "/" +
                FLAGS_config_file;
    }
#else
    std::string config = std::string(std::getenv("CRDC_WS")) + '/' + FLAGS_config_file;
#endif
    LOG(INFO) << "[LIDAR_MAIN] Use proto config: " << config;

    if (!crdc::airi::util::is_path_exists(config)) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC not exists, please setup environment first.";
        return 1;
    }

    if (!crdc::airi::util::get_proto_from_file(config, &lidar_component)) {
        LOG(FATAL) << "[LIDAR_MAIN] failed to read lidar config proto.";
        return 1;
    }

    LOG(INFO) << lidar_component.DebugString();

    std::shared_ptr<LidarFusion> lidar_fusion;
    lidar_fusion = std::make_shared<LidarFusion>(lidar_component);
    lidar_fusion->start();

    LOG(INFO) << "[LIDAR_MAIN] lidar_driver started.";
    common::Singleton<ModuleDiagnose>::get()->init(MODULE, "LIDAR_DIAGNOSE");
    common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
    common::Singleton<ModuleDiagnose>::get()->set_ready();
    common::Singleton<ModuleDiagnose>::get()->start();
    LOG(INFO) << "[LIDAR_MAIN] lidar module_diagnose started.";

    apollo::cyber::WaitForShutdown();
    lidar_fusion->stop();
    common::Singleton<ModuleDiagnose>::get()->stop();

    if (lidar_fusion->is_alive()) {
        lidar_fusion->join();
    }

    LOG(WARNING) << "[LIDAR_MAIN] lidar_driver terminated.";
    return 0;
}
}  // namespace lidar
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) { return crdc::airi::lidar::main(argc, argv); }
