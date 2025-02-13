#pragma once

#include <unordered_map>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include "camera_drivers/proto/camera_diagnose_config.pb.h"

namespace sensor {
namespace hub {

class DetectorBase {
 public:
  DetectorBase() {}
  virtual ~DetectorBase() {}
  virtual int classify_an_image(const cv::Mat& img, const cv::Mat& gray) = 0;
  virtual void init_cfg(const CameraDiagnoseConfig& cfg) = 0;
};

}  // namespace hub
}  // namespace sensor
