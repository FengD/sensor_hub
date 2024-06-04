#include "camera_drivers/camera_diagnose/green_screen_detector.h"

namespace crdc {
namespace airi {
  GreenScreenDetector::GreenScreenDetector(const CameraDiagnoseConfig& cfg) {
    init_cfg(cfg);
  }

void GreenScreenDetector::init_cfg(const CameraDiagnoseConfig& cfg) {
  target_value_ = {0, cfg.green_screen_config().green_value(), 0};
}

/**
 * @brief This function is used to detect if the image is green screen
 * @param A image
 * @return 1 means image is green screen, 0 means image is normal
 */
int GreenScreenDetector::classify_an_image(const cv::Mat& img,
                                           const cv::Mat& gray) {
  for (int row = 0; row < img.rows; row++) {
    for (int col = 0; col < img.cols; col++) {
      cv::Vec3b pixel = img.at<cv::Vec3b>(row, col);
      if (pixel != target_value_) {
        return 0;
      }
    }
  }
  return 1;
}

}  // namespace airi
}  // namespace crdc
