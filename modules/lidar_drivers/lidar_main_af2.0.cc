// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING, Zilou Cao
// Description: lidar main

#ifdef WITH_TDA4
#include <utils/properties.h>
#endif
#include <gflags/gflags.h>
#include <memory>
#include "common/common.h"
#include "common/util.h"
#include "lidar_drivers/lidar_fusion.h"
#include "module_diagnose/module_diagnose.h"
#include "lidar_drivers/proto/af_args_config.pb.h"
#include "af/block/compute_block.hpp"

DEFINE_string(module, "LidarDriver", "The name of this module");
DEFINE_string(config_file, "params/drivers/lidar/test/lidar_config.prototxt",
              "path of config file");
DEFINE_string(crdc_ws, "CRDC_WS", "The prefix of the workspace");
DEFINE_bool(use_product_name, true, "use product_name to splicing config_file");

namespace crdc {
namespace airi {
namespace lidar {

#ifdef WITH_TDA4
void set_app_env(const std::string &config_path) {
  auto product_name = get_product_name();
  LOG(INFO) << "[Lidar_MAIN] Product name is " << product_name;
  std::string crdc_ws = config_path;
  set_env(FLAGS_crdc_ws, crdc_ws);
}
#endif

class LidarMain : public af::block::ComputeBlock<> {
 public:
  bool on_init(const af::block::CustomerConfig & af_config) override {
    LOG(INFO) << "[LIDAR_MAIN] af config file path is " << af_config.config_path;
#ifdef WITH_TDA4
    auto product_name = get_product_name();
    LOG(INFO) << "[Lidar_MAIN] Product name is " << product_name;
    std::string crdc_ws = af_config.config_path.substr(0,
            af_config.config_path.find_last_of("/"));
    set_env(FLAGS_crdc_ws, crdc_ws);
#endif
    google::InitGoogleLogging(FLAGS_module.c_str());
    google::EnableLogCleaner(LOG_SAVE_DAY);
    FLAGS_colorlogtostderr = true;
    FLAGS_stop_logging_if_full_disk = true;
    AFArgsConfig af_args_config;
    if (crdc::airi::util::is_path_exists(af_config.config_path) &&
          crdc::airi::util::get_proto_from_file(af_config.config_path, &af_args_config)) {
      FLAGS_v = af_args_config.log_v();
      FLAGS_alsologtostderr = af_args_config.alsologtostdrr();
      FLAGS_stderrthreshold = af_args_config.stderrthreshold();
      FLAGS_minloglevel = af_args_config.minloglevel();
      FLAGS_max_log_size = af_args_config.max_log_size();
      google::SetLogDestination(google::GLOG_FATAL, af_args_config.log_dir().c_str());
      google::SetLogDestination(google::GLOG_ERROR, af_args_config.log_dir().c_str());
      google::SetLogDestination(google::GLOG_WARNING, af_args_config.log_dir().c_str());
      google::SetLogDestination(google::GLOG_INFO, af_args_config.log_dir().c_str());
      if (!crdc::airi::util::is_directory_exists(af_args_config.log_dir())) {
        LOG(WARNING) << "[LIDAR_MAIN] The log directory does not exist, now create it";
        std::string command = "mkdir -p " + af_args_config.log_dir();
        if (!system(command.c_str())) {
          LOG(WARNING) << "[LIDAR_MAIN] Failed to create dir " <<
                af_args_config.log_dir();
        }
      }
    } else {
      FLAGS_logtostderr = true;
      LOG(WARNING) << "[LIDAR_MAIN] failed to read af args config proto.";
    }

    if (!std::getenv("CRDC_WS")) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC_WS not setting!";
    } else {
        LOG(INFO) << "[LIDAR_MAIN] Current CRDC_WS: " << std::string(std::getenv("CRDC_WS"));
    }

    common::Singleton<LidarAFOutput>::get()->init(FLAGS_module);
    common::Singleton<ModuleDiagnose>::get()->init(FLAGS_module, "LIDAR_DIAGNOSE");
    LidarComponent lidar_component;
#ifdef WITH_TDA4
    std::string config;
    if (FLAGS_use_product_name) {
      config = std::string(std::getenv("CRDC_WS")) + "/params/drivers/lidar/"+
                product_name + "/lidar_config.prototxt";
    } else {
      config = std::string(std::getenv("CRDC_WS")) + "/" +
        FLAGS_config_file;
    }
#else
#ifdef CALIBRATE
    std::string config = std::string(std::getenv("CRDC_WS")) + "params/" +
                         std::getenv("vehicle_type") + "/lidar_config.prototxt";
#else
    if (!std::getenv("LIDAR_FILE_PATH")) {
      LOG(FATAL) << "LIDAR_FILE_PATH not setting!";
    } else {
      LOG(INFO) << "Current LIDAR_FILE_PATH: "
                << std::string(std::getenv("LIDAR_FILE_PATH"));
    }
    std::string config = std::string(std::getenv("CRDC_WS"))
                                  + "/params/drivers/lidar/"
                                  + std::getenv("LIDAR_FILE_PATH")
                                  + "/lidar_config.prototxt";
#endif
#endif
    LOG(INFO) << "[LIDAR_MAIN] Use proto config: " << config;

    if (!crdc::airi::util::is_path_exists(config)) {
        LOG(FATAL) << "[LIDAR_MAIN] CRDC not exists, please setup environment first.";
        return false;
    }

    if (!crdc::airi::util::get_proto_from_file(config, &lidar_component)) {
        LOG(FATAL) << "[LIDAR_MAIN] failed to read lidar config proto.";
        return false;
    }

    LOG(INFO) << lidar_component.DebugString();

    lidar_fusion_ = std::make_shared<crdc::airi::LidarFusion>(lidar_component);
    lidar_fusion_->start();
    LOG(INFO) << "[LIDAR_MAIN] lidar_driver started.";

    common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
    common::Singleton<ModuleDiagnose>::get()->set_ready();
    common::Singleton<ModuleDiagnose>::get()->start();
    LOG(INFO) << "[LIDAR_MAIN] lidar module_diagnose started.";

    return true;
  }

  void on_shutdown() {
    lidar_fusion_->stop();
    if (lidar_fusion_->is_alive()) {
      lidar_fusion_->join();
    }
    google::ShutdownGoogleLogging();
    LOG(WARNING) << "[LIDAR_MAIN] lidar_driver terminated.";
  }

 private:
  std::shared_ptr<crdc::airi::LidarFusion> lidar_fusion_;
  static constexpr int LOG_SAVE_DAY = 15;
};

AF_REGISTER_BLOCK(LidarMain)

}  // namespace lidar
}  // namespace airi
}  // namespace crdc
