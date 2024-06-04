#include "camera_drivers/camera_diagnose/blocked_detector.h"

#include <algorithm>
#include <vector>

namespace crdc {
namespace airi {
BlockedDetector::BlockedDetector(const CameraDiagnoseConfig& cfg) {
  init_cfg(cfg);
}
/**
 * @brief This function is used to init config
 */
void BlockedDetector::init_cfg(const CameraDiagnoseConfig& cfg) {
  n_cell_x_ = cfg.blocked_config().n_cell_x();
  n_cell_y_ = cfg.blocked_config().n_cell_y();
  blocked_min_value_ = cfg.blocked_config().blocked_min_value();
  blocked_max_value_ = cfg.blocked_config().blocked_max_value();
  gray_min_value_ = cfg.blocked_config().gray_min_value();
  gray_max_value_ = cfg.blocked_config().gray_max_value();
  n_min_value_ = cfg.blocked_config().n_min_value();
  sub_mask_max_value_ = cfg.blocked_config().sub_mask_max_value();
}

/**
 * @brief This function is used to get the neighbor of the coordinate (i,j)
 * @param i x coordinate
 * @param j y coordinate
 * @param neighbors all neighbor points
 */
void BlockedDetector::get_neighbors(
    int i, int j, std::vector<std::pair<int, int>>& neighbors) {
  if (i > 0) {
    neighbors.push_back(std::make_pair(i - 1, j));
  }
  if (i < n_cell_y_ - 1) {
    neighbors.push_back(std::make_pair(i + 1, j));
  }
  if (j > 0) {
    neighbors.push_back(std::make_pair(i, j - 1));
  }
  if (j < n_cell_x_ - 1) {
    neighbors.push_back(std::make_pair(i, j + 1));
  }
}

void BlockedDetector::find_connected_domain(
    const std::vector<std::vector<int>>& arr,
    std::vector<std::vector<int>>& result) {
  int label = 1;
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      if (arr[i][j]) {
        std::vector<std::pair<int, int>> neighbors;
        get_neighbors(i, j, neighbors);
        bool is_connected = false;
        std::vector<int> tmp;
        for (const auto& neighbor : neighbors) {
          if (result[neighbor.first][neighbor.second] > 0) {
            is_connected = true;
            tmp.push_back(result[neighbor.first][neighbor.second]);
          }
        }
        if (is_connected) {
          result[i][j] = *min_element(tmp.begin(), tmp.end());
        } else {
          result[i][j] = label;
          label++;
        }
      }
    }
  }
  std::vector<std::vector<int>> vis(n_cell_y_, std::vector<int>(n_cell_x_, 0));
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      if (result[i][j] == 0 || vis[i][j]) {
        continue;
      }
      // BFS
      int min_label = result[i][j];
      std::queue<std::pair<int, int>> q;
      q.push(std::make_pair(i, j));

      while (!q.empty()) {
        std::pair<int, int> cur = q.front();
        q.pop();
        result[cur.first][cur.second] = min_label;
        vis[cur.first][cur.second] = 1;

        std::vector<std::pair<int, int>> neighbors;
        get_neighbors(cur.first, cur.second, neighbors);
        for (const auto& neighbor : neighbors) {
          if (!vis[neighbor.first][neighbor.second] &&
              result[neighbor.first][neighbor.second] > 0) {
            q.push(neighbor);
            min_label =
                std::min(min_label, result[neighbor.first][neighbor.second]);
          }
        }
      }
    }
  }
}

void BlockedDetector::find_max_area(
    const std::vector<std::vector<int>>& mask,
    std::unordered_map<int, std::vector<std::pair<int, int>>>& index,
    int& max_area) {
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      auto it = index.find(mask[i][j]);
      if (mask[i][j] != 0 && it != index.end()) {
        it->second.push_back(std::make_pair(i, j));
      } else {
        index[mask[i][j]] = std::vector<std::pair<int, int>>{{i, j}};
      }
    }
  }
  int idx_area;
  for (const auto& idx : index) {
    idx_area = idx.second.size();
    max_area = std::max(max_area, idx_area);
  }
}

/**
 * @brief This function is used to get tenengrad
 * @param raw image
 * @param tenengrad image
 * @return tenengrad image
 */
void BlockedDetector::get_tenengrad(const cv::Mat& img, cv::Mat& g_xy) {
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
 * @brief This function is used to detect if the blocked iamge
 * @param A image
 * @return 1 means image is blocked, 0 means image is normal
 */
int BlockedDetector::classify_an_image(const cv::Mat& img, const cv::Mat& gray) {
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
  double blocked_threshold = (blocked_max_value_ - blocked_min_value_) /
                          (gray_max_value_ - gray_min_value_) *
                          (clip_gray - gray_min_value_) +
                      blocked_min_value_;
  int height = gray.rows;
  int width = gray.cols;
  int gap_y = height / n_cell_y_;
  int gap_x = width / n_cell_x_;

  cv::Mat g_xy;
  get_tenengrad(img, g_xy);
  g_xy = reversed_sky_mask.mul(g_xy);

  std::vector<std::vector<double>> blocked_value_arr(
      n_cell_y_, std::vector<double>(n_cell_x_, 0));
  double blocked_value;
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      cv::Mat sub_g_xy = g_xy.rowRange(j * gap_y, (j + 1) * gap_y)
                             .colRange(i * gap_x, (i + 1) * gap_x);
      cv::Mat sub_mask = sky_mask.rowRange(j * gap_y, (j + 1) * gap_y)
                             .colRange(i * gap_x, (i + 1) * gap_x);
      double sub_mask_sum = cv::sum(sub_mask)[0] / (gap_x * gap_y);
      if (sub_mask_sum > sub_mask_max_value_) {
        blocked_value = -1;
      } else {
        blocked_value = cv::mean(sub_g_xy)[0];
      }
      blocked_value_arr[i][j] = blocked_value;
    }
  }
  std::vector<std::vector<int>> bool_mask(n_cell_y_,
                                          std::vector<int>(n_cell_x_));
  for (int i = 0; i < n_cell_y_; i++) {
    for (int j = 0; j < n_cell_x_; j++) {
      bool_mask[i][j] = (blocked_value_arr[i][j] < blocked_threshold &&
                         blocked_value_arr[i][j] >= 0)
                            ? 1
                            : 0;
    }
  }
  std::vector<std::vector<int>> blocked_mask(n_cell_y_,
                                             std::vector<int>(n_cell_x_, 0));
  find_connected_domain(bool_mask, blocked_mask);
  std::unordered_map<int, std::vector<std::pair<int, int>>> index;
  int n_blocked = 0;
  int max_area = 0;
  find_max_area(blocked_mask, index, max_area);
  n_blocked = std::max(max_area, n_blocked);
  if (n_blocked > n_min_value_) {
    return 1;
  } else {
    return 0;
  }
}
}  // namespace airi
}  // namespace crdc
