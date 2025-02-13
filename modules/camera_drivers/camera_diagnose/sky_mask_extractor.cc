#include <iostream>
#include <algorithm>
#include "camera_drivers/camera_diagnose/sky_mask_extractor.h"


namespace sensor {
namespace hub {

SkyMaskExtractor::SkyMaskExtractor(const cv::Mat& cfg) : cfg_(cfg) {}

/**
 * @brief This function is used to get sky mask
 * @param img: Input image
 * @param grad_th: Grad threshold
 * @return sky_mask: Mask of the sky area
 */
void SkyMaskExtractor::get_sky_roi(const cv::Mat& img, const cv::Mat& gray,
                                   cv::Mat& sky_mask, const double grad_th) {
  cv::Mat cur_img = img.clone();
  cv::resize(img, cur_img, cv::Size(), 0.25, 0.25);
  cv::Mat cur_gray;
  cv::cvtColor(cur_img, cur_gray, cv::COLOR_BGR2GRAY);

  cv::Mat sobel_x, sobel_y;
  cv::Sobel(cur_gray, sobel_x, CV_64F, 1, 0, 3);
  cv::Sobel(cur_gray, sobel_y, CV_64F, 0, 1, 3);

  cv::Mat sobel_x_bin, sobel_y_bin;
  cv::threshold(sobel_x, sobel_x_bin, grad_th, 1, cv::THRESH_BINARY);
  cv::threshold(sobel_y, sobel_y_bin, grad_th, 1, cv::THRESH_BINARY);

  cv::Mat sobel_xy_bin;
  cv::bitwise_or(sobel_x_bin, sobel_x_bin, sobel_xy_bin);
  sobel_xy_bin.convertTo(sobel_xy_bin, CV_64F);

  cv::Mat ones_matrix = cv::Mat::ones(sobel_xy_bin.size(), CV_64F);
  cv::Mat reversed_xy_bin;
  cv::subtract(ones_matrix, sobel_xy_bin, reversed_xy_bin);

  cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
  cv::Mat reversed_xy_bin_erode;
  cv::erode(reversed_xy_bin, reversed_xy_bin_erode, kernel7, cv::Point(-1, -1),
            5);

  reversed_xy_bin_erode.convertTo(reversed_xy_bin_erode, CV_8U);
  cv::findContours(reversed_xy_bin_erode, cnts_, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  std::sort(cnts_.begin(), cnts_.end(),
            [](const std::vector<cv::Point>& cnts1,
               const std::vector<cv::Point>& cnts2) {
              return cv::contourArea(cnts1) > cv::contourArea(cnts2);
            });
  sky_mask = cv::Mat::zeros(img.size(), CV_8UC1);

  SatisfiedData satisfied_data = get_satisfied_area(cur_img, cnts_);

  sim_list_.resize(cnts_.size(), 1.0);

  if (satisfied_data.first == -1) {
    return;
  }
  double area_ratio =
      cv::contourArea(cnts_[satisfied_data.first]) / (cur_img.rows * cur_img.cols);
  if (area_ratio > 0.75) {
    return;
  }

  cv::Mat max_mask;
  get_sky_mask(cur_img, max_mask, cnts_, satisfied_data.first);

  cv::Mat cur_mask;
  for (auto i : satisfied_data.all_satisfy) {
    get_sky_mask(cur_img, cur_mask, cnts_, i);
    SimilarityData simi_data = get_color_similarity(cur_img, max_mask, cur_mask);
    sim_list_[i] = simi_data.sim;

    if (simi_data.flag) {
      max_mask |= cur_mask;
    }
  }
  cv::resize(max_mask, sky_mask, img.size(), 0, 0, cv::INTER_NEAREST);
  return;
}

/**
 * @brief This function is used to get the satisfied area
 * @param img: Input image after resize
 * @param contours: All contours detected in the image
 * @return data.dafirst: The maximum region index that meets the criteria
 * @return data.all_satisfy: All profile index that meet the criteria after
 * removing the largest area
 */
SatisfiedData SkyMaskExtractor::get_satisfied_area(
    cv::Mat img, std::vector<std::vector<cv::Point>>& contours) {
  SatisfiedData data;

  for (int i = 0; i < contours.size(); i++) {
    double area = cv::contourArea(contours[i]);
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    drawContours(mask, contours, i, cv::Scalar(1), -1);
    double up_area = cv::sum(mask(cv::Rect(0, 0, mask.cols, mask.rows / 2)))[0];

    if (up_area / area > 0.5) {
      if (data.first == -1) data.first = i;
      if (data.first != -1 && data.first != i)
        data.all_satisfy.push_back(i);
    }
  }
  return data;
}

/**
 * @brief This function is used to get the sky mask
 * @param idx: The index of the contours
 * @param contours: All contours detected in the image
 * @return dst: A mask diagram that specifies the contour of the index
 */
void SkyMaskExtractor::get_sky_mask(
    const cv::Mat& img, cv::Mat& mask,
    std::vector<std::vector<cv::Point>>& contours, int idx) {
  mask = cv::Mat::zeros(img.size(), CV_8UC1);
  cv::drawContours(mask, contours, idx, cv::Scalar(255), -1);
}

/**
 * @brief This function is used to get the bgr normalize
 * @param img: Input image after resize
 * @param mask: The area where the color histogram needs to be calculated
 * @return bgr_normalize: Normalized color histogram
 */
void SkyMaskExtractor::calc_color_hist(const cv::Mat& img, const cv::Mat& mask,
                                       cv::Mat& bgr_normalize) {
  std::vector<cv::Mat> bgr_channels;
  cv::split(img, bgr_channels);

  int hist_size = 25;
  float range[] = {0, 256};
  const float* hist_ranges[] = {range};
  int channels[] = {0};

  cv::Mat hist_b, hist_g, hist_r;

  cv::calcHist(&bgr_channels[0], 1, channels, mask, hist_b, 1, &hist_size,
               hist_ranges);
  cv::calcHist(&bgr_channels[1], 1, channels, mask, hist_g, 1, &hist_size,
               hist_ranges);
  cv::calcHist(&bgr_channels[2], 1, channels, mask, hist_r, 1, &hist_size,
               hist_ranges);

  cv::Mat bg, bgr;
  cv::vconcat(hist_b, hist_g, bg);
  cv::vconcat(bg, hist_r, bgr);

  double l1_value = cv::sum(cv::abs(bgr))[0];
  bgr_normalize = bgr / l1_value;
}

/**
 * @brief This function is used to get the color similarity.
 * @param img: Input image after resize
 * @param mask1: The first area needs to be compared
 * @param mask2: The second area needs to be compared
 * @return sim_data.sim: Color similarity between the two regions
 */
SimilarityData SkyMaskExtractor::get_color_similarity(const cv::Mat& img,
                                                      const cv::Mat& mask1,
                                                      const cv::Mat& mask2) {
  SimilarityData sim_data;
  cv::Mat hist1;
  cv::Mat hist2;
  calc_color_hist(img, mask1, hist1);
  calc_color_hist(img, mask2, hist2);
  for (int i = 0; i < hist1.rows; i++)
    sim_data.sim += std::min(hist1.at<float>(i), hist2.at<float>(i));
  sim_data.flag = sim_data.sim > 0.3 ? true : false;
  return sim_data;
}

}  // namespace hub
}  // namespace sensor
