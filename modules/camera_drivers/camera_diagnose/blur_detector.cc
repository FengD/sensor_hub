#include "camera_drivers/camera_diagnose/blur_detector.h"

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

namespace crdc {
namespace airi {
BlurDetector::BlurDetector(const CameraDiagnoseConfig& cfg) {
  init_cfg(cfg);
}
void BlurDetector::init_cfg(const CameraDiagnoseConfig& cfg) {
  n_cell_x_ = cfg.blur_config().n_cell_x();
  n_cell_y_ = cfg.blur_config().n_cell_y();
  blur_min_value_ = cfg.blur_config().blur_min_value();
  blur_miax_value_ = cfg.blur_config().blur_max_value();
  gray_min_value_ = cfg.blur_config().gray_min_value();
  gray_max_value_ = cfg.blur_config().gray_max_value();
  n_min_value_ = cfg.blur_config().n_min_value();
  sub_mask_max_value_ = cfg.blur_config().sub_mask_max_value();
}

/**
 * @brief This function is used to get tenengrad
 * @param raw image
 * @param tenengrad image
 * @return tenengrad image
 */
void BlurDetector::get_tenengrad(const cv::Mat& img, cv::Mat& g_xy) {
  cv::Mat gray;
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  cv::Mat sobel_x, sobel_y;
  cv::Sobel(gray, sobel_x, CV_64F, 1, 0, 3);
  cv::Sobel(gray, sobel_y, CV_64F, 0, 1, 3);
  cv::Mat x2 = sobel_x.mul(sobel_x);
  cv::Mat y2 = sobel_y.mul(sobel_y);

  cv::sqrt(x2 + y2, g_xy);
}

/**
 * @brief This function is used to detect if the blur iamge
 * @param A image
 * @return 1 means image is blur, 0 means image is normal
 */
int BlurDetector::classify_an_image(const cv::Mat& img, const cv::Mat& gray) {
  SkyMaskExtractor sky_extractor;
  cv::Mat sky_mask = cv::Mat::zeros(img.size(), CV_8UC1);
  sky_extractor.get_sky_roi(img, gray, sky_mask);
  cv::Mat reversed_sky_mask_8u;
  cv::bitwise_not(sky_mask, reversed_sky_mask_8u);
  cv::Mat reversed_sky_mask;
  reversed_sky_mask_8u.convertTo(reversed_sky_mask, CV_64F, 1.0 / 255.0);

  double mean_gray = cv::mean(gray)[0];
  double clip_gray =
      std::min(std::max(mean_gray, static_cast<double>(gray_min_value_)),
               static_cast<double>(gray_max_value_));
  double blur_threshold = (blur_miax_value_ - blur_min_value_) /
                       (gray_max_value_ - gray_min_value_) *
                       (clip_gray - gray_min_value_) +
                   blur_min_value_;
  int height = gray.rows;
  int width = gray.cols;
  int gap_y = height / n_cell_y_;
  int gap_x = width / n_cell_x_;

  // img and blured_img have different gray
  cv::Mat g_xy;
  get_tenengrad(img, g_xy);
  g_xy = reversed_sky_mask.mul(g_xy);
  cv::Mat blured_img;
  cv::blur(img, blured_img, cv::Size(5, 5));
  cv::Mat blured_g_xy;
  get_tenengrad(blured_img, blured_g_xy);
  blured_g_xy = reversed_sky_mask.mul(blured_g_xy);
  int n_blur = 0;
  std::vector<std::vector<double>> arr1(8, std::vector<double>(8, 0));
  std::vector<std::vector<double>> blur_value_arr(n_cell_y_, std::vector<double>(n_cell_x_, 0));
  std::vector<std::vector<double>> blur2_value_arr(n_cell_y_, std::vector<double>(n_cell_x_, 0));
  double blur_value, blur2_value;
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      cv::Mat sub_g_xy = g_xy.rowRange(j * gap_y, (j + 1) * gap_y)
                             .colRange(i * gap_x, (i + 1) * gap_x);
      cv::Mat sub_blured_g_xy = blured_g_xy.rowRange(j * gap_y, (j + 1) * gap_y)
                                    .colRange(i * gap_x, (i + 1) * gap_x);
      cv::Mat sub_mask = sky_mask.rowRange(j * gap_y, (j + 1) * gap_y)
                             .colRange(i * gap_x, (i + 1) * gap_x);
      double sub_mask_sum = cv::sum(sub_mask)[0] / (gap_x * gap_y);
      if (sub_mask_sum > sub_mask_max_value_) {
        blur_value = 0;
      } else {
        blur_value = cv::mean(sub_g_xy)[0];
        blur_value_arr[i][j] = blur_value;
      }
      blur2_value = cv::mean(sub_blured_g_xy)[0];
      blur2_value_arr[i][j] = blur2_value;
    }
  }

  double blur_value_i_j;
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      blur_value_i_j = blur_value_arr[i][j] / blur2_value_arr[i][j];
      if (blur_value_i_j > 0.0 && blur_value_i_j < blur_threshold) {
        n_blur++;
      }
    }
  }

  if (n_blur > n_min_value_) {
    return 1;
  } else {
    return 0;
  }
}
}  // namespace airi
}  // namespace crdc
