#pragma once

#include "camera_drivers/camera_diagnose/base/detector_base.h"
#include "camera_drivers/camera_diagnose/sky_mask_extractor.h"

/**
 * @brief This function is used to determine whether the image is blur
 */
namespace sensor {
namespace hub {

class BlurDetector : public DetectorBase {
 public:
  explicit BlurDetector(const CameraDiagnoseConfig& cfg);
  ~BlurDetector() {}
  int classify_an_image(const cv::Mat& img, const cv::Mat& gray) override;
 private:
  void init_cfg(const CameraDiagnoseConfig& cfg);
  void get_tenengrad(const cv::Mat& img, cv::Mat& g_xy);
  void get_fore_mask(const cv::Mat& sky_mask, cv::Mat& fore_mask);
  int n_cell_x_;
  int n_cell_y_;
  double blur_min_value_;
  double blur_miax_value_;
  int gray_min_value_;
  int gray_max_value_;
  int n_min_value_;
  double sub_mask_max_value_;
};

}  // namespace hub
}  // namespace sensor
