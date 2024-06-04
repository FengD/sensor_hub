#include <iostream>
#include <string>
#include <gflags/gflags.h>
#include "common/common.h"
#include "cyber/cyber.h"
#include "transform/transform.h"
#include "transform/transform_perception_obstacles.h"
#include "transform/transform_2d_gt.h"

namespace crdc {
namespace airi {

#define MODULE "TRANSFOR_RECORD"
DEFINE_string(config_file, "params/transform_config.prototxt", "config file");

int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  apollo::cyber::GlobalData::Instance()->SetProcessGroup(MODULE);
  apollo::cyber::Init(MODULE);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  // process_offline_data(FLAGS_input_file);
  TransformComponent transform_component;
  std::string config_file = FLAGS_config_file;
  LOG(INFO) << "[CAMERA_MAIN] Use proto config: " << config_file;

  if (!crdc::airi::util::is_path_exists(config_file)) {
      LOG(FATAL) << "[CAMERA_MAIN] CRDC not exists, please setup environment first.";
      return 1;
  }

  if (!crdc::airi::util::get_proto_from_file(config_file, &transform_component)) {
      LOG(FATAL) << "[CAMERA_MAIN] failed to read camera config proto.";
      return 1;
  }

  LOG(INFO) << transform_component.DebugString();
  TransformImage transform_image;
  TransformPointCloud transform_pointcloud;
  TransformPercptionObstacles transform_perception_obstacles;
  Transform2DGT transform_2d_gt;

  std::thread thread_transform_image;
  std::thread thread_transform_pointcloud;
  std::thread thread_transform_perception_obstacles;
  std::thread thread_transform_2d_gt;

  for (auto& config : transform_component.transform_component_config()) {
    if (0 == config.message_type().compare("PointCloud")) {
      thread_transform_pointcloud = std::thread(&TransformPointCloud::init, transform_pointcloud, config);
    } else if (0 == config.message_type().compare("CompressedImage")) {
      thread_transform_image = std::thread(&TransformImage::init, transform_image, config);
    } else if (0 == config.message_type().compare("PerceptionObstacles")) {
      thread_transform_perception_obstacles = std::thread(&TransformPercptionObstacles::init, transform_perception_obstacles, config);
    } else if (0 == config.message_type().compare("2DGT")) {
      thread_transform_2d_gt = std::thread(&Transform2DGT::init, transform_2d_gt, config);
    }
  }
  if (thread_transform_pointcloud.joinable()) thread_transform_pointcloud.join();
  if (thread_transform_image.joinable()) thread_transform_image.join();
  if (thread_transform_perception_obstacles.joinable()) thread_transform_perception_obstacles.join();
  if (thread_transform_2d_gt.joinable()) thread_transform_2d_gt.join();
  return 0;
}

}  // namespace airi
}  // namespace crdc

int main(int argc, char *argv[]) {return crdc::airi::main(argc, argv);}