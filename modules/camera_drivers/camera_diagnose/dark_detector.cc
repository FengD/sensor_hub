#include "camera_drivers/camera_diagnose/dark_detector.h"

/**
 * @brief This function is used to detect if the dark image
 * @param A image
 * @return 1 means False, 0 means True
 */
namespace crdc {
namespace airi {
DarkDetector::DarkDetector(const CameraDiagnoseConfig& cfg) {
  init_cfg(cfg);
}
void DarkDetector::init_cfg(const CameraDiagnoseConfig& cfg) {
  pixel_value_min_ = cfg.dark_config().pixel_value_min_();
  dark_prop_ = cfg.dark_config().dark_prop();
}

int DarkDetector::classify_an_image(const cv::Mat& img, const cv::Mat& gray) {
  int hight = gray.rows;
  int width = gray.cols;
  int pixels_sum = hight * width;

  int dark_sum = 0;
  for (int row = 0; row < hight; row++) {
    for (int col = 0; col < width; col++) {
      if (gray.at<uchar>(row, col) < pixel_value_min_) {
        dark_sum++;
      }
    }
  }
  double dark_prop = static_cast<double>(dark_sum * 1.0 / pixels_sum);
  if (dark_prop >= dark_prop_) {
    return 1;
  }
  return 0;
}
}  // namespace airi
}  // namespace crdc
