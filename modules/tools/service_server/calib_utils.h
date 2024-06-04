
#pragma once

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <zlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <filesystem>

namespace crdc {
namespace airi {
class CalibrationUtils {
 public:
  CalibrationUtils() {}
  void delete_file(const std::string &file_path);
  void write_base64_stream_file(const std::string &file_path, const std::string base64_stream);
  void pack_files(const std::string &folderpath);
  std::string get_current_time();
  void check_or_create_file(const std::string &folder_path);
  void find_image_points(const cv::Mat image, std::vector<cv::Point2f> &image_points,
                         const float area_low_threshold);
 private:
  std::string get_file_prefix(const std::string &filename);
  std::vector<cv::Point2f> find_circle_center(const cv::Mat image, const float area_low_threshold);
  void seperate_points(const std::vector<cv::Point2f> &image_points, int image_width,
                       std::vector<cv::Point2f> &calib_val_points,
                       std::vector<cv::Point2f> &calib_cal_points);
  float cal_dis(const cv::Point2f &pt_left, const cv::Point2f &pt_right);
  static bool compare_by_points(const cv::Point2f &point_a, const cv::Point2f &point_b);
  static bool compare_by_dis(const std::pair<float, cv::Point2f> &pair_a,
                             const std::pair<float, cv::Point2f> &pair_b);
};
}  // namespace airi
}  // namespace crdc
