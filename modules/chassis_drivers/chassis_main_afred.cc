// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Zilou Cao, Yuanyuan WANG
// Description: Chassis main

#ifdef WITH_TDA4
#include <utils/properties.h>
#endif
#include <gflags/gflags.h>
#include <memory>
#include "common/common.h"
#include "common/util.h"
#include "module_diagnose/module_diagnose.h"
#include "chassis_drivers/chassis.h"
#include "chassis_drivers/output/af_output.h"
#include "af/block/compute_block.hpp"
#include "chassis_drivers/proto/af_args_config.pb.h"

DEFINE_string(module, "ChassisDriver", "The name of this module");
DEFINE_string(config_file, "test",
              "path of config file");
DEFINE_string(crdc_ws, "CRDC_WS", "The prefix of the workspace");
DEFINE_bool(use_product_name, true, "use product_name to splicing config_file");
DEFINE_string(exec_manifest, "EXEC_MANIFEST", "The macro of EXEC_MANIFEST");
DEFINE_string(ara_service_instanec, "ARA_SERVICE_INSTANCE",
              "The macro of ARA_SERVICE_INSTANCE");




namespace crdc {
namespace airi {
namespace chassis {

class ChassisMain : public af::block::ComputeBlock<> {
 public:
  bool on_init(const af::block::CustomerConfig & af_config) override {
    auto time1 = get_now_microsecond();
    LOG(INFO) << "[CHASSIS_MAIN] af config file path is " << af_config.config_path;
#ifdef WITH_TDA4
    product_name_ = get_product_name();
    LOG(INFO) << "[CHASSIS_MAIN] Product name is " << product_name_;
    set_app_env(af_config.config_path);
#endif
    google::InitGoogleLogging(FLAGS_module.c_str());
    FLAGS_colorlogtostderr = true;
    FLAGS_stop_logging_if_full_disk = false;
    AFArgsConfig af_args_config;
    if (crdc::airi::util::is_path_exists(af_config.config_path) &&
          crdc::airi::util::get_proto_from_file(af_config.config_path, &af_args_config)) {
      FLAGS_v = af_args_config.log_v();
      FLAGS_log_dir = af_args_config.log_dir();
      FLAGS_alsologtostderr = af_args_config.alsologtostdrr();
      FLAGS_stderrthreshold = af_args_config.stderrthreshold();
      FLAGS_minloglevel = af_args_config.minloglevel();
      FLAGS_max_log_size = af_args_config.max_log_size();
      google::SetLogDestination(google::GLOG_FATAL, af_args_config.log_dir().c_str());
      google::SetLogDestination(google::GLOG_ERROR, af_args_config.log_dir().c_str());
      google::SetLogDestination(google::GLOG_WARNING, af_args_config.log_dir().c_str());
      google::SetLogDestination(google::GLOG_INFO, af_args_config.log_dir().c_str());
      if (!crdc::airi::util::is_directory_exists(af_args_config.log_dir())) {
        LOG(WARNING) << "[CHASSIS_MAIN] The log directory does not exist, now create it";
        std::string command = "mkdir -p " + af_args_config.log_dir();
        if (!system(command.c_str())) {
          LOG(WARNING) << "[CHASSIS_MAIN] Failed to create dir " <<
                af_args_config.log_dir();
        }
      }
    } else {
      FLAGS_logtostderr = true;
      LOG(WARNING) << "[CHASSIS_MAIN] failed to read af args config proto.";
    }

    if (!std::getenv("CRDC_WS")) {
        LOG(FATAL) << "[CHASSIS_MAIN] CRDC_WS not setting!";
    } else {
        LOG(INFO) << "[CHASSIS_MAIN] Current CRDC_WS: " << std::string(std::getenv("CRDC_WS"));
    }

    common::Singleton<ChassisAFOutput>::get()->init(FLAGS_module);
    common::Singleton<ModuleDiagnose>::get()->init(FLAGS_module, "CHASSIS_DIAGNOSE");

#ifdef WITH_TDA4
    std::string config;
    if (FLAGS_use_product_name) {
      config = std::string(std::getenv("CRDC_WS")) + "/params/drivers/chassis/"+
                product_name_ + "/chassis_config.prototxt";
    } else {
      config = std::string(std::getenv("CRDC_WS")) + "/params/drivers/chassis/" +
        FLAGS_config_file + "/chassis_config.prototxt";
    }
#else
    std::string config = std::string(std::getenv("CRDC_WS")) + "/" +
            FLAGS_config_file;
#endif
    LOG(INFO) << "[CHASSIS_MAIN] Use proto config: " << config;

    if (!crdc::airi::util::is_path_exists(config)) {
        LOG(FATAL) << "[CHASSIS_MAIN] CRDC not exists, please setup environment first.";
        return false;
    }
    ChassisComponent chassis_component;

    if (!crdc::airi::util::get_proto_from_file(config, &chassis_component)) {
        LOG(FATAL) << "[CHASSIS_MAIN] failed to read chassis config proto.";
        return false;
    }

    LOG(INFO) << chassis_component.DebugString();
    for (const auto& config : chassis_component.component_config()) {
      std::shared_ptr<crdc::airi::ChassisSensor> chassis =
          std::make_shared<crdc::airi::ChassisSensor>(config);
      chassiss_.emplace_back(chassis);
    }
    LOG(INFO) <<"[TIMER] [callback] init_time(s): "
                    << (get_now_microsecond() - time1);
    for (const auto& chassis : chassiss_) {
      chassis->start();
    }

    LOG(INFO) << "[CHASSIS_MAIN] chassis_driver started.";

    common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
    common::Singleton<ModuleDiagnose>::get()->set_ready();
    common::Singleton<ModuleDiagnose>::get()->start();
    LOG(INFO) << "[CHASSIS_MAIN] chassis module_diagnose started.";

    return true;
  }

  void on_shutdown() {
    for (const auto& chassis : chassiss_) {
      chassis->stop();
    }

    for (const auto& chassis : chassiss_) {
      if (chassis->is_alive()) {
        chassis->join();
      }
    }

    google::ShutdownGoogleLogging();
    LOG(WARNING) << "[CHASSIS_MAIN] chassis_driver terminated.";
  }
#ifdef WITH_TDA4
  void set_app_env(const std::string &config_path) {
    std::string crdc_ws = config_path.substr(0, config_path.find_last_of("/"));
    set_env(FLAGS_crdc_ws, crdc_ws);
    std::string ara_service_instance;

    std::string exec_manifest_config = crdc_ws +  "/af_launcher/MANIFEST";
    set_env(FLAGS_exec_manifest, exec_manifest_config);

    LOG(INFO) << "[" << FLAGS_module << "] EXEC_MANIFEST is "
              << std::getenv(FLAGS_exec_manifest.c_str());

    if (FLAGS_use_product_name) {
      ara_service_instance = crdc_ws + "/etc_product_release/" +
              product_name_ + "/service_instance.toml";
    } else {
      ara_service_instance = crdc_ws + "/etc_product_release/" +
              FLAGS_config_file + "/service_instance.toml";
    }
    set_env(FLAGS_ara_service_instanec, ara_service_instance);
    LOG(INFO) << "[" << FLAGS_module << "] ARA_SERVICE_INSTANCE is "
              << std::getenv(FLAGS_ara_service_instanec.c_str());
  }
#endif

 private:
  std::vector<std::shared_ptr<crdc::airi::ChassisSensor>> chassiss_;
#ifdef WITH_TDA4
  std::string product_name_;
#endif
};

AF_REGISTER_BLOCK(ChassisMain)

}  // namespace chassis
}  // namespace airi
}  // namespace crdc
