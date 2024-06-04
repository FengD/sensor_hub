// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera main

#include <gflags/gflags.h>
#ifdef WITH_TDA4
#include <utils/properties.h>
#include <map>
#endif
#include "common/common.h"
#include "common/util.h"
#include "module_diagnose/module_diagnose.h"
#include "camera_drivers/output/af_output.h"
#include "camera_drivers/camera.h"
#include "camera_drivers/proto/af_args_config.pb.h"
#include "af/block/compute_block.hpp"

DEFINE_string(module, "CameraDriver", "The name of this module");
DEFINE_string(config_file, "params/drivers/camera/default/camera_config.prototxt",
              "The config path of this module");
DEFINE_string(crdc_ws, "CRDC_WS", "The prefix of the workspace");
DEFINE_bool(use_product_name, true, "use product_name to splicing config_file");

namespace crdc {
namespace airi {
namespace camera {

class CameraMain : public af::block::ComputeBlock<> {
 public:
  bool on_init(const af::block::CustomerConfig & af_config) override {
    LOG(INFO) << "[CAMERA_MAIN] af config file path is " << af_config.config_path;
#ifdef WITH_TDA4
    auto product_name = get_product_name();
    LOG(INFO) << "[CAMERA_MAIN] Product name is " << product_name;
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
        LOG(WARNING) << "[CAMERA_MAIN] The log directory does not exist, now create it";
        std::string command = "mkdir -p " + af_args_config.log_dir();
        if (!system(command.c_str())) {
          LOG(WARNING) << "[CAMERA_MAIN] Failed to create dir " <<
                af_args_config.log_dir();
        }
      }
    } else {
      FLAGS_logtostderr = true;
      LOG(WARNING) << "[CAMERA_MAIN] failed to read af args config proto.";
    }

    if (!std::getenv("CRDC_WS")) {
        LOG(FATAL) << "[CAMERA_MAIN] CRDC_WS not setting!";
    } else {
        LOG(INFO) << "[CAMERA_MAIN] Current CRDC_WS: " << std::string(std::getenv("CRDC_WS"));
    }
    common::Singleton<CameraAFOutput>::get()->init(FLAGS_module);

    CameraComponent camera_component;
#ifdef WITH_TDA4
    std::string config;
    if (FLAGS_use_product_name) {
        config = std::string(std::getenv("CRDC_WS")) + "/params/drivers/camera/"+
                product_name + "/camera_config.prototxt";
    } else {
        config = std::string(std::getenv("CRDC_WS")) + '/' + FLAGS_config_file;
    }
#else
    if (!std::getenv("CAMERA_FILE_PATH")) {
      LOG(FATAL) << "CAMERA_FILE_PATH not setting!";
    } else {
      LOG(INFO) << "Current CAMERA_FILE_PATH: "
                << std::string(std::getenv("CAMERA_FILE_PATH"));
    }
    std::string config = std::string(std::getenv("CRDC_WS"))
                                  + "/params/drivers/camera/"
                                  + std::getenv("CAMERA_FILE_PATH")
                                  + "/camera_config.prototxt";
#endif
    LOG(INFO) << "[CAMERA_MAIN] Use proto config: " << config;

    if (!crdc::airi::util::is_path_exists(config)) {
      LOG(FATAL) << "[CAMERA_MAIN] CRDC not exists, please setup environment first.";
      return false;
    }

    if (!crdc::airi::util::get_proto_from_file(config, &camera_component)) {
      LOG(FATAL) << "[CAMERA_MAIN] failed to read camera config proto.";
      return false;
    }

    LOG(INFO) << camera_component.DebugString();


    for (const auto& config : camera_component.component_config()) {
      std::shared_ptr<crdc::airi::Camera> camera = std::make_shared<crdc::airi::Camera>(config);
      cameras_.emplace_back(camera);
    }

    for (const auto& camera : cameras_) {
      camera->start();
    }

    LOG(INFO) << "[CAMERA_MAIN] camera_driver started.";
    common::Singleton<ModuleDiagnose>::get()->init(FLAGS_module, "CAMERA_DIAGNOSE");
    common::Singleton<ModuleDiagnose>::get()->set_period_usec(100000);
    common::Singleton<ModuleDiagnose>::get()->set_ready();
    common::Singleton<ModuleDiagnose>::get()->start();
    LOG(INFO) << "[CAMERA_MAIN] camera module_diagnose started.";
    return true;
  }

  void on_shutdown() {
    common::Singleton<ModuleDiagnose>::get()->stop();
    for (const auto& camera : cameras_) {
      camera->join();
    }
    for (const auto& camera : cameras_) {
      if (camera->is_alive()) {
        camera->join();
      }
      LOG(INFO) << "[CAMERA_MAIN] camera[" << camera->get_name() << "] joined successfully.";
    }
    google::ShutdownGoogleLogging();
  }

 private:
  std::vector<std::shared_ptr<crdc::airi::Camera>> cameras_;
  static constexpr int LOG_SAVE_DAY = 15;
};

AF_REGISTER_BLOCK(CameraMain)

}  // namespace camera
}  // namespace airi
}  // namespace crdc
