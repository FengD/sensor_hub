#pragma once

#include "camera_drivers/camera_diagnose/base/detector_base.h"

namespace crdc {
namespace airi {

class GreenScreenDetector : public DetectorBase {
 public:
  explicit GreenScreenDetector(const CameraDiagnoseConfig& cfg);
  ~GreenScreenDetector() {}
  int classify_an_image(const cv::Mat& img, const cv::Mat& gray) override;

 private:
  void init_cfg(const CameraDiagnoseConfig& cfg) override;
  cv::Vec3b target_value_;
};

}  // namespace airi
}  // namespace crdc
