#include "camera_drivers/camera_diagnose/camera_diagnose.h"
#include <utility>

namespace crdc {
namespace airi {
CameraDiagnoser::CameraDiagnoser(const CameraDiagnoseConfig& cfg) {
  initialize_status_mappings();
  green_screen_detector_ = std::make_unique<crdc::airi::GreenScreenDetector>(cfg);
  facula_detector_ = std::make_unique<crdc::airi::FaculaDetector>(cfg);
  dark_detector_ = std::make_unique<crdc::airi::DarkDetector>(cfg);
  blocked_detector_ = std::make_unique<crdc::airi::BlockedDetector>(cfg);
  blur_detector_ = std::make_unique<crdc::airi::BlurDetector>(cfg);
}

void CameraDiagnoser::green_screen_diagnose(const cv::Mat& image, const cv::Mat& gray) {
  if (green_screen_detector_ &&
      diagnose_results_.find(CameraDiagnoseStatus::GREEN_SCREEN) !=
          diagnose_results_.end()) {
    diagnose_results_[CameraDiagnoseStatus::GREEN_SCREEN] =
        green_screen_detector_->classify_an_image(image, gray);
    if (diagnose_results_[CameraDiagnoseStatus::GREEN_SCREEN] == 1) {
      status_description_ =
          status_description_map_[CameraDiagnoseStatus::GREEN_SCREEN];
      device_status_ = device_status_map_[CameraDiagnoseStatus::GREEN_SCREEN];
      return;
    }
  }
}
void CameraDiagnoser::facula_diagnose(const cv::Mat& image, const cv::Mat& gray) {
  if (facula_detector_ && diagnose_results_.find(CameraDiagnoseStatus::FACULA) !=
                              diagnose_results_.end()) {
    diagnose_results_[CameraDiagnoseStatus::FACULA] =
        facula_detector_->classify_an_image(image, gray);
    if (diagnose_results_[CameraDiagnoseStatus::FACULA] == 1) {
      status_description_ = status_description_map_[CameraDiagnoseStatus::FACULA];
      device_status_ = device_status_map_[CameraDiagnoseStatus::FACULA];
      return;
    }
  }
}
void CameraDiagnoser::dark_diagnose(const cv::Mat& image, const cv::Mat& gray) {
  if (dark_detector_ && diagnose_results_.find(CameraDiagnoseStatus::DARK) !=
                            diagnose_results_.end()) {
    diagnose_results_[CameraDiagnoseStatus::DARK] =
        dark_detector_->classify_an_image(image, gray);
    if (diagnose_results_[CameraDiagnoseStatus::DARK] == 1) {
      status_description_ = status_description_map_[CameraDiagnoseStatus::DARK];
      device_status_ = device_status_map_[CameraDiagnoseStatus::DARK];
      return;
    }
  }
}
void CameraDiagnoser::blocked_diagnose(const cv::Mat& image, const cv::Mat& gray) {
  if (blocked_detector_ && diagnose_results_.find(CameraDiagnoseStatus::BLOCKED) !=
                            diagnose_results_.end()) {
    diagnose_results_[CameraDiagnoseStatus::BLOCKED] =
        blocked_detector_->classify_an_image(image, gray);
    if (diagnose_results_[CameraDiagnoseStatus::BLOCKED] == 1) {
      status_description_ = status_description_map_[CameraDiagnoseStatus::BLOCKED];
      device_status_ = device_status_map_[CameraDiagnoseStatus::BLOCKED];
      return;
    }
  }
}
void CameraDiagnoser::blur_diagnose(const cv::Mat& image, const cv::Mat& gray) {
  if (blur_detector_ && diagnose_results_.find(CameraDiagnoseStatus::BLUR) !=
                            diagnose_results_.end()) {
    diagnose_results_[CameraDiagnoseStatus::BLUR] =
        blur_detector_->classify_an_image(image, gray);
    if (diagnose_results_[CameraDiagnoseStatus::BLUR] == 1) {
      status_description_ = status_description_map_[CameraDiagnoseStatus::BLUR];
      device_status_ = device_status_map_[CameraDiagnoseStatus::BLUR];
      return;
    }
  }
}


/**
 * @brief Diagnose the scene category of the image
 * @param The input image data
 * @param The diagnostic information
 */
void CameraDiagnoser::diagnose(const cv::Mat& image, const cv::Mat& gray) {
  green_screen_diagnose(image, gray);
  if (device_status_ == device_status_map_[CameraDiagnoseStatus::GREEN_SCREEN]) {
    return;
  }
  dark_diagnose(image, gray);
  if (device_status_ == device_status_map_[CameraDiagnoseStatus::DARK]) {
    return;
  }
  facula_diagnose(image, gray);
  if (device_status_ == device_status_map_[CameraDiagnoseStatus::FACULA]) {
    return;
  }
  blocked_diagnose(image, gray);
  if (device_status_ == device_status_map_[CameraDiagnoseStatus::BLOCKED]) {
    return;
  }
  blur_diagnose(image, gray);
  if (device_status_ == device_status_map_[CameraDiagnoseStatus::BLUR]) {
    return;
  }
}

void CameraDiagnoser::diagnose_an_image(const cv::Mat& raw_data_bgr) {
  device_status_ = static_cast<DeviceStatus>(1);
  status_description_ = "";
  cv::Mat gray_img;
  cv::cvtColor(raw_data_bgr, gray_img, cv::COLOR_BGR2GRAY);
  diagnose(raw_data_bgr, gray_img);
}

void CameraDiagnoser::initialize_status_mappings() {
  status_description_map_ = {
      {CameraDiagnoseStatus::GREEN_SCREEN,
       "camera_raw_image_is_all_green(pixel_are_all_zero)"},
      {CameraDiagnoseStatus::FACULA,
       "camera_is_being_exposed_to_extremely_strong_light"},
      {CameraDiagnoseStatus::DARK,
       "camera_is_being_exposed_to_extremely_weak_light"},
      {CameraDiagnoseStatus::BLOCKED,
       "camera_is_being_blocked_by_other_objects"},
      {CameraDiagnoseStatus::BLUR,
       "camera_image_data_is_blurry"}};

  device_status_map_ = {
      {CameraDiagnoseStatus::GREEN_SCREEN, DeviceStatus::CAMERA_DATA_GREEN},
      {CameraDiagnoseStatus::FACULA, DeviceStatus::CAMERA_STRONG_BACKLIGHT},
      {CameraDiagnoseStatus::DARK, DeviceStatus::CAMERA_IMAGE_DARK},
      {CameraDiagnoseStatus::BLOCKED, DeviceStatus::CAMERA_WINDOW_DIRTY_BLOCKED},
      {CameraDiagnoseStatus::BLUR, DeviceStatus::CAMERA_IMAGE_BLUR}};

  diagnose_results_ = {{CameraDiagnoseStatus::GREEN_SCREEN, -1},
                       {CameraDiagnoseStatus::FACULA, -1},
                       {CameraDiagnoseStatus::DARK, -1},
                       {CameraDiagnoseStatus::BLOCKED, -1},
                       {CameraDiagnoseStatus::BLUR, -1}};
}

void CameraDiagnoser::reset_diagnose_results() {
  diagnose_results_.at(CameraDiagnoseStatus::GREEN_SCREEN) = -1;
  diagnose_results_.at(CameraDiagnoseStatus::FACULA) = -1;
  diagnose_results_.at(CameraDiagnoseStatus::DARK) = -1;
  diagnose_results_.at(CameraDiagnoseStatus::BLOCKED) = -1;
  diagnose_results_.at(CameraDiagnoseStatus::BLUR) = -1;
}
}  // namespace airi
}  // namespace crdc
