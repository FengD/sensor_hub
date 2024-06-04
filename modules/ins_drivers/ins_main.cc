// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins main

#include "ins_drivers/ins.h"
#include <gflags/gflags.h>
#include "common/common.h"
#include "module_diagnose/module_diagnose.h"
#include "ins_drivers/output/cyber_output.h"

#define MODULE "InsDriver"
DEFINE_string(config_file, "params/drivers/ins/test/ins_config.prototxt",
              "path of config file");

namespace crdc {
namespace airi {
namespace ins {

int main(int argc, char* argv[]) {
  // gflags command setting
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!std::getenv("CRDC_WS")) {
    LOG(FATAL) << "[INS_MAIN] CRDC_WS not setting!";
  } else {
    LOG(INFO) << "[INS_MAIN] Current CRDC_WS: "
              << std::string(std::getenv("CRDC_WS"));
  }

  apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
  apollo::cyber::Init(MODULE);
  common::Singleton<InsCyberOutput>::get()->init(MODULE);

  InsComponent ins_component;

  std::string config = std::string(std::getenv("CRDC_WS")) + '/' + FLAGS_config_file;
  LOG(INFO) << "[INS_MAIN] Use proto config: " << config;

  if (!crdc::airi::util::is_path_exists(config)) {
    LOG(FATAL) << "[INS_MAIN] CRDC not exists, please setup environment first.";
    return 1;
  }

  if (!crdc::airi::util::get_proto_from_file(config, &ins_component)) {
    LOG(FATAL) << "[INS_MAIN] failed to read ins config proto.";
    return 1;
  }

  LOG(INFO) << ins_component.DebugString();

  std::vector<std::shared_ptr<crdc::airi::InsSensor>> inss;
  for (const auto& config : ins_component.component_config()) {
    std::shared_ptr<crdc::airi::InsSensor> ins =
        std::make_shared<crdc::airi::InsSensor>(config);
    inss.emplace_back(ins);
  }

  for (const auto& ins : inss) {
    ins->start();
  }

  LOG(INFO) << "[INS_MAIN] ins_driver started.";

  common::Singleton<ModuleDiagnose>::get()->init(MODULE, "INS_DIAGNOSE");
  common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
  common::Singleton<ModuleDiagnose>::get()->set_ready();
  common::Singleton<ModuleDiagnose>::get()->start();
  LOG(INFO) << "[INS_MAIN] ins module_diagnose started.";
  apollo::cyber::WaitForShutdown();
  for (const auto& ins : inss) {
    ins->stop();
  }

  for (const auto& ins : inss) {
    if (ins->is_alive()) {
      ins->join();
    }
    LOG(INFO) << "[INS_MAIN] ins[" << ins->get_name()
              << "] joined successfully.";
  }

  LOG(WARNING) << "[INS_MAIN] ins_driver terminated.";
  return 0;
}

}  // namespace ins
}  // namespace airi
}  // namespace crdc

int main(int argc, char* argv[]) { return crdc::airi::ins::main(argc, argv); }
