#pragma once

#include "camera_drivers/camera_diagnose/base/detector_base.h"

namespace crdc {
namespace airi {

class DarkDetector : public DetectorBase {
 public:
  explicit DarkDetector(const CameraDiagnoseConfig& cfg);
  ~DarkDetector() {}
  int classify_an_image(const cv::Mat& img, const cv::Mat& gray) override;

 private:
  void init_cfg(const CameraDiagnoseConfig& cfg) override;
  int pixel_value_min_;
  double dark_prop_;
};

}  // namespace airi
}  // namespace crdc
