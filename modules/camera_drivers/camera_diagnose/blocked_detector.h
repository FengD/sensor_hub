#pragma once

#include <queue>
#include <utility>
#include <vector>
#include <unordered_map>
#include "camera_drivers/camera_diagnose/base/detector_base.h"
#include "camera_drivers/camera_diagnose/sky_mask_extractor.h"

/**
 * @brief This function is used to determine whether the image is blocked
 */
namespace sensor {
namespace hub {

class BlockedDetector : public DetectorBase {
 public:
  explicit BlockedDetector(const CameraDiagnoseConfig& cfg);
  ~BlockedDetector() {}
  int classify_an_image(const cv::Mat& img, const cv::Mat& gray) override;

 private:
  void init_cfg(const CameraDiagnoseConfig& cfg) override;
  void get_neighbors(int i, int j, std::vector<std::pair<int, int>>& neighbors);
  void find_connected_domain(const std::vector<std::vector<int>>& arr,
                             std::vector<std::vector<int>>& result);
  void find_max_area(
      const std::vector<std::vector<int>>& mask,
      std::unordered_map<int, std::vector<std::pair<int, int>>>& index,
      int& max_area);
  void get_tenengrad(const cv::Mat& img, cv::Mat& g_xy);
  int n_cell_x_;
  int n_cell_y_;
  double blocked_min_value_;
  double blocked_max_value_;
  int gray_min_value_;
  int gray_max_value_;
  int n_min_value_;
  double sub_mask_max_value_;
};

}  // namespace hub
}  // namespace sensor
