#include "tools/service_server/calib_utils.h"

namespace crdc {
namespace airi {

void CalibrationUtils::delete_file(const std::string &file_path) {
  const std::filesystem::path path(file_path);
  if (std::filesystem::exists(path)) {
    std::filesystem::remove(path);
  }
}

std::string CalibrationUtils::get_file_prefix(const std::string &filename) {
  size_t pos = filename.find('.');
  if (pos != std::string::npos) {
    return filename.substr(0, pos);
  }
  return filename;
}

void CalibrationUtils::check_or_create_file(const std::string &folder_path) {
  if (!std::filesystem::exists(folder_path) || !std::filesystem::is_directory(folder_path)) {
    if (std::filesystem::create_directories(folder_path)) {
      std::cout << "Folder " << folder_path << "does not exist,created successfully." << std::endl;
    } else {
      std::cerr << "Unbale to create folder " << folder_path << "." << std::endl;
    }
  } else {
    std::cout << "Folder " << folder_path << "already exists." << std::endl;
  }
}

void CalibrationUtils::pack_files(const std::string &folderpath) {
  std::vector<std::string> files;
  DIR *dirp = opendir(folderpath.c_str());
  struct dirent *dp;
  while ((dp = readdir(dirp)) != NULL) {
    if (dp->d_type == DT_REG) {
      std::string filename = dp->d_name;
      files.push_back(filename);
    }
  }
  closedir(dirp);

  std::sort(files.begin(), files.end());

  std::vector<std::vector<std::string>> groups;
  for (size_t i = 0; i < files.size();) {
    std::string prefix = get_file_prefix(files[i]);
    std::vector<std::string> group;
    group.push_back(files[i]);
    ++i;
    while (i < files.size() && get_file_prefix(files[i]) == prefix) {
      group.push_back(files[i]);
      ++i;
    }
    groups.push_back(group);
  }

  for (std::vector<std::string> &group : groups) {
    std::string prefix = get_file_prefix(group[0]);
    std::string foldername = folderpath + "/" + prefix;
    mkdir(foldername.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    for (const std::string &filename : group) {
      std::string filepath = folderpath + "/" + filename;
      std::string destpath = foldername + "/" + filename;
      if (rename(filepath.c_str(), destpath.c_str()) != 0) {
        std::cerr << "Failed to move file: " << filepath << std::endl;
      }
    }
    std::string zip_file_name = foldername + ".zip";
    std::string compress_command = "zip -j " + zip_file_name + " " + foldername + "/*";
    if (system(compress_command.c_str())) {
      std::cerr << "Failed to compress file" << zip_file_name << std::endl;
    }
  }
}

std::string CalibrationUtils::get_current_time() {
  auto now = std::chrono::system_clock::now();
  std::time_t current_time = std::chrono::system_clock::to_time_t(now);
  std::tm current_tm = *std::localtime(&current_time);

  int year = current_tm.tm_year + 1900;
  int month = current_tm.tm_mon + 1;
  int day = current_tm.tm_mday;
  int hour = current_tm.tm_hour;
  int min = current_tm.tm_min;
  int sec = current_tm.tm_sec;
  std::ostringstream oss;
  auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
  oss << std::to_string(year) << "-" << std::setw(2) << std::setfill('0') << std::to_string(month)
      << "-" << std::setw(2) << std::setfill('0') << std::to_string(day) << "-" << std::setw(2)
      << std::setfill('0') << std::to_string(hour) << "-" << std::setw(2) << std::setfill('0')
      << std::to_string(min) << "-" << std::setw(2) << std::setfill('0') << std::to_string(sec)
      << "-" << std::setw(3) << std::setfill('0') << std::to_string(milliseconds.count());
  std::string time_str = oss.str();
  return time_str;
}

void CalibrationUtils::write_base64_stream_file(const std::string &file_path,
                                                const std::string base64_stream) {
  std::ofstream file(file_path, std::ofstream::trunc);
  file << base64_stream << std::endl;
  file.close();
}

std::vector<cv::Point2f> CalibrationUtils::find_circle_center(const cv::Mat image,
                                                              const float area_low_threshold) {
  int width = image.cols;
  int height = image.rows;
  cv::Mat roi_image = image.clone();
  roi_image = roi_image(cv::Rect(width / 4, height / 2, width / 2, height / 4)); // Crop region
  cv::Mat gray, gauss, binary, morph;
  float low_threshold;
  roi_image.convertTo(roi_image, -1, 1.5, 0);
  double mean_value = cv::mean(roi_image)[0];
  cv::cvtColor(roi_image, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(gray, gauss, cv::Size(3, 3), 0);
  if (mean_value > 128) {
    low_threshold = 200;
  } else {
    low_threshold = mean_value + 60;
  }
  cv::threshold(gauss, binary, low_threshold, 255, cv::THRESH_BINARY);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
  cv::morphologyEx(binary, morph, cv::MORPH_CLOSE, element);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(morph, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point2f> ret;
  for (const auto& c : contours) {
    double area = cv::contourArea(c);
    if (area > 3500 || area < area_low_threshold) continue;
    cv::Point2f center_roi;
    float radius;
    cv::minEnclosingCircle(c, center_roi, radius);
    cv::Point2f center(center_roi.x + width / 4, center_roi.y + height / 2);
    if (radius < 25) {
      ret.push_back(center);
    }
  }
  for (size_t i = 0; i < ret.size(); ++i) {
    for (size_t j = i + 1; j < ret.size(); ++j) {
      double distance = cv::norm(ret[i] - ret[j]);
      if (distance < 5) {
        cv::Point2f average_point = (ret[i] + ret[j]) / 2;
        ret.erase(remove(ret.begin(), ret.end(), ret[i]), ret.end());
        ret.erase(remove(ret.begin(), ret.end(), ret[j]), ret.end());
        ret.push_back(average_point);
      }
    }
  }
  return ret;
}

void CalibrationUtils::seperate_points(const std::vector<cv::Point2f>& image_points, int image_width,
                     std::vector<cv::Point2f>& calib_val_points,
                     std::vector<cv::Point2f>& calib_cal_points) {
  int center_x = image_width / 2;
  for (const auto& point : image_points) {
    if (point.x < center_x) {
      calib_val_points.push_back(point);
    } else {
      calib_cal_points.push_back(point);
    }
  }
}

float CalibrationUtils::cal_dis(const cv::Point2f& pt_left, const cv::Point2f& pt_right) {
  return std::sqrt(std::pow(pt_left.x - pt_right.x, 2) + std::pow(pt_left.y - pt_right.y, 2));
}

bool CalibrationUtils::compare_by_points(const cv::Point2f& point_a, const cv::Point2f& point_b) {
  if (point_a.y == point_b.y) {
    return point_a.y < point_b.y;
  }
  return point_a.x > point_b.x;
}

bool CalibrationUtils::compare_by_dis(const std::pair<float, cv::Point2f>& pair_a,
                                      const std::pair<float, cv::Point2f>& pair_b) {
  return pair_a.first > pair_b.first;
}

void CalibrationUtils::find_image_points(const cv::Mat image,
                                         std::vector<cv::Point2f>& image_points,
                                         const float area_low_threshold) {
  std::vector<cv::Point2f> calib_val_points, calib_cal_points;
  int width = image.cols;
  std::vector<cv::Point2f> image_points_temp = find_circle_center(image, area_low_threshold);
  if (image_points_temp.size() != 8) {
    std::cout << "Can't extract enough points from image!" << std::endl;
    return;
  }
  seperate_points(image_points_temp, width, calib_val_points, calib_cal_points);
  std::vector<std::pair<float, cv::Point2f>> distances;
  for (const auto& val_point : calib_val_points) {
    for (const auto& cal_point : calib_cal_points) {
      float dis = cal_dis(val_point, cal_point);
      if (std::abs(val_point.y - cal_point.y) < 30) continue;
      distances.push_back(std::make_pair(dis, val_point));
    }
  }
  std::sort(distances.begin(), distances.end(), CalibrationUtils::compare_by_dis);
  int distance_size = distances.size();
  for (int i = 1; i < distance_size; i++) {
    if (distances[i].second.x == distances[i - 1].second.x) {
      distances.erase(distances.begin() + i);
    }
  }
  for (const auto& val_point : calib_val_points) {
    if (val_point.x != distances[0].second.x && val_point.x != distances[1].second.x) {
      calib_cal_points.push_back(val_point);
    }
  }
  calib_val_points.clear();
  calib_val_points.push_back(distances[0].second);
  calib_val_points.push_back(distances[1].second);
  std::sort(calib_cal_points.begin(), calib_cal_points.end(), CalibrationUtils::compare_by_points);
  std::sort(calib_val_points.begin(), calib_val_points.end(), CalibrationUtils::compare_by_points);
  image_points.insert(image_points.end(), calib_cal_points.begin(), calib_cal_points.end());
  image_points.insert(image_points.end(), calib_val_points.begin(), calib_val_points.end());
}
}  // namespace airi
}  // namespace crdc
