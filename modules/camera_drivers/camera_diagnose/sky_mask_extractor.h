#pragma once

#include <vector>
#include "camera_drivers/camera_diagnose/base/feature_extractor.h"

namespace sensor {
namespace hub {

struct SatisfiedData {
  int first = -1;
  std::vector<int> all_satisfy;
};
struct SimilarityData {
  bool flag;
  double sim = 0.0;
};

class SkyMaskExtractor : public FeatureExtractor {
 public:
  SkyMaskExtractor() {}
  ~SkyMaskExtractor() {}
  explicit SkyMaskExtractor(const cv::Mat& cfg);
  void get_sky_roi(const cv::Mat& img, const cv::Mat& gray, cv::Mat& sky_mask,
                   const double grad_th = 35.0);

 private:
  SatisfiedData get_satisfied_area(
      cv::Mat img, std::vector<std::vector<cv::Point>>& contours);
  void get_sky_mask(const cv::Mat& img, cv::Mat& mask,
                    std::vector<std::vector<cv::Point>>& contours, int idx);
  void calc_color_hist(const cv::Mat& img, const cv::Mat& mask,
                       cv::Mat& bgr_normalize);
  SimilarityData get_color_similarity(const cv::Mat& img, const cv::Mat& mask1,
                                      const cv::Mat& mask2);

  cv::Mat cfg_;
  std::vector<std::vector<cv::Point>> contours, cnts_;
  std::vector<double> sim_list_;
};

}  // namespace hub
}  // namespace sensor
