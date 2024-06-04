#pragma once

#include "camera_drivers/camera_diagnose/base/detector_base.h"
#include "camera_drivers/camera_diagnose/sky_mask_extractor.h"

/**
 * @brief This function is used to determine whether the image is facula
 */
namespace crdc {
namespace airi {

class FaculaDetector : public DetectorBase {
 public:
  explicit FaculaDetector(const CameraDiagnoseConfig& cfg);
  ~FaculaDetector() {}
  int classify_an_image(const cv::Mat& img, const cv::Mat& gray) override;
 private:
  void init_cfg(const CameraDiagnoseConfig& cfg) override;
  int sum_mask(cv::Mat img);
  int facula_area_min_;
  int pixel_value_min_;
};

}  // namespace airi
}  // namespace crdc
