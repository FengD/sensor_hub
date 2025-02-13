#include "camera_drivers/camera_diagnose/facula_detector.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

namespace sensor {
namespace hub {
FaculaDetector::FaculaDetector(const CameraDiagnoseConfig& cfg) {
  init_cfg(cfg);
}
void FaculaDetector::init_cfg(const CameraDiagnoseConfig& cfg) {
  facula_area_min_ = cfg.facula_config().facula_area_min_();
  pixel_value_min_ = cfg.facula_config().pixel_value_min_();
}

/**
 * @brief This function is used to detect if the image is flare
 * @param A image
 * @return 1 means image is facula, 0 means image is normal
 */
int FaculaDetector::classify_an_image(const cv::Mat& img, const cv::Mat& gray) {
  SkyMaskExtractor sky_extractor;
  cv::Mat sky_mask = cv::Mat::zeros(img.size(), CV_8UC1);
  sky_extractor.get_sky_roi(img, gray, sky_mask);

  int bottom = 0;
  int bottom_y;
  if (bottom == 0 || bottom * 4 < img.cols * 0.4) {
    bottom_y = floor(img.cols / 3 * 2);
  } else {
    bottom_y = bottom * 4;
  }

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::Mat img_bin;
  cv::threshold(gray, img_bin, pixel_value_min_,
                255, cv::THRESH_BINARY);
  cv::Mat kernel = cv::Mat::ones(5, 5, CV_8UC1);
  cv::Mat erosion;
  cv::erode(img_bin, erosion, kernel, cv::Point(-1, -1), 5);
  cv::findContours(erosion, contours, hierarchy, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  std::sort(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& contour1,
               const std::vector<cv::Point>& contour2) {
              return cv::contourArea(contour1) > cv::contourArea(contour2);
            });
  cv::Mat ground_mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
  sky_mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

  for (const auto& contour : contours) {
    int idx = 1;
    auto minmax_y = std::minmax_element(
        contour.begin(), contour.end(),
        [](const cv::Point& p1, const cv::Point& p2) { return p1.y < p2.y; });
    cv::Point bottom_most = *minmax_y.first;
    cv::Point top_most = *minmax_y.second;
    double area = cv::contourArea(contour);
    if (top_most.y > bottom_y && area > facula_area_min_) {
      cv::drawContours(ground_mask, contours, idx, cv::Scalar(1), -1);
    }
    if (bottom_most.y < bottom_y && area > facula_area_min_) {
      cv::drawContours(sky_mask, contours, idx, cv::Scalar(1), -1);
    }
    idx++;
  }

  int ground_area = sum_mask(ground_mask);
  int sky_area = sum_mask(sky_mask);

  if (sky_area > facula_area_min_) {
    return 1;
  } else {
    return 0;
  }
}

int FaculaDetector::sum_mask(cv::Mat img) {
  int area_sum = 0;
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      if (img.at<uchar>(i, j) > 0) area_sum++;
    }
  }
  return area_sum;
}

}  // namespace hub
}  // namespace sensor
