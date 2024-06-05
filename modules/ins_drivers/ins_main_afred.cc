// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Zilou Cao
// Description: Ins main

#ifdef WITH_TDA4
#include <utils/properties.h>
#endif
#include <gflags/gflags.h>
#include <memory>
#include "common/common.h"
#include "common/util.h"
#include "module_diagnose/module_diagnose.h"
#include "ins_drivers/ins.h"
#include "ins_drivers/output/af_output.h"
#include "af/block/compute_block.hpp"
#include "ins_drivers/proto/af_args_config.pb.h"

DEFINE_string(module, "InsDriver", "The name of this module");
DEFINE_string(config_file, "params/drivers/ins/test/ins_config.prototxt",
              "path of config file");
DEFINE_string(crdc_ws, "CRDC_WS", "The prefix of the workspace");
DEFINE_bool(use_product_name, false, "use product_name to splicing config_file");

namespace crdc {
namespace airi {
namespace ins {

#ifdef WITH_TDA4
void set_app_env(const std::string &config_path) {
  auto product_name = get_product_name();
  LOG(INFO) << "[INS_MAIN] Product name is " << product_name;
  std::string crdc_ws = config_path;
  set_env(FLAGS_crdc_ws, crdc_ws);
}
#endif

class InsMain : public af::block::ComputeBlock<> {
 public:
  bool on_init(const af::block::CustomerConfig & af_config) override {
    LOG(INFO) << "[INS_MAIN] af config file path is " << af_config.config_path;
#ifdef WITH_TDA4
    auto product_name = get_product_name();
    LOG(INFO) << "[INS_MAIN] Product name is " << product_name;
    std::string crdc_ws = af_config.config_path.substr(0,
            af_config.config_path.find_last_of("/"));
    set_env(FLAGS_crdc_ws, crdc_ws);
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
      if (!crdc::airi::util::is_directory_exists(af_args_config.log_dir())) {
        LOG(WARNING) << "[INS_MAIN] The log directory does not exist, now create it";
        std::string command = "mkdir -p " + af_args_config.log_dir();
        if (!system(command.c_str())) {
          LOG(WARNING) << "[INS_MAIN] Failed to create dir " <<
                af_args_config.log_dir();
        }
      }
    } else {
      FLAGS_logtostderr = true;
      LOG(WARNING) << "[INS_MAIN] failed to read af args config proto.";
    }

    if (!std::getenv("CRDC_WS")) {
        LOG(FATAL) << "[INS_MAIN] CRDC_WS not setting!";
    } else {
        LOG(INFO) << "[INS_MAIN] Current CRDC_WS: " << std::string(std::getenv("CRDC_WS"));
    }

    common::Singleton<InsAFOutput>::get()->init(FLAGS_module);
    common::Singleton<ModuleDiagnose>::get()->init(FLAGS_module, "INS_DIAGNOSE");

#ifdef WITH_TDA4
    std::string config;
    if (FLAGS_use_product_name) {
      config = std::string(std::getenv("CRDC_WS")) + "/params/drivers/ins/"+
                product_name + "/ins_config.prototxt";
    } else {
      config = std::string(std::getenv("CRDC_WS")) + "/" +
        FLAGS_config_file;
    }
#else
    std::string config = std::string(std::getenv("CRDC_WS")) + "/" +
            FLAGS_config_file;
#endif
    LOG(INFO) << "[INS_MAIN] Use proto config: " << config;

    if (!crdc::airi::util::is_path_exists(config)) {
        LOG(FATAL) << "[INS_MAIN] CRDC not exists, please setup environment first.";
        return false;
    }
    InsComponent ins_component;

    if (!crdc::airi::util::get_proto_from_file(config, &ins_component)) {
        LOG(FATAL) << "[INS_MAIN] failed to read ins config proto.";
        return false;
    }

    LOG(INFO) << ins_component.DebugString();
    for (const auto& config : ins_component.component_config()) {
      std::shared_ptr<crdc::airi::InsSensor> ins =
          std::make_shared<crdc::airi::InsSensor>(config);
      inss_.emplace_back(ins);
    }

    for (const auto& ins : inss_) {
      ins->start();
    }

    LOG(INFO) << "[INS_MAIN] ins_driver started.";

    common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
    common::Singleton<ModuleDiagnose>::get()->set_ready();
    common::Singleton<ModuleDiagnose>::get()->start();
    LOG(INFO) << "[INS_MAIN] ins module_diagnose started.";

    return true;
  }

  void on_shutdown() {
    for (const auto& ins : inss_) {
      ins->stop();
    }

    for (const auto& ins : inss_) {
      if (ins->is_alive()) {
        ins->join();
      }
    }

    google::ShutdownGoogleLogging();
    LOG(WARNING) << "[INS_MAIN] ins_driver terminated.";
  }

 private:
  std::vector<std::shared_ptr<crdc::airi::InsSensor>> inss_;
};

AF_REGISTER_BLOCK(InsMain)

}  // namespace ins
}  // namespace airi
}  // namespace crdc
