// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera undistortion cv

#include "camera_drivers/undistortion/cv/cv_undistortion.h"

namespace sensor {
namespace hub {

bool CvUndistortion::init(const CameraSensorConfig& config) {
    return true;
}

bool CvUndistortion::process(const cv::Mat& img, cv::Mat& img_distorted) {
    img_distorted = img;
    return true;
}

}  // namespace hub
}  // namespace sensor
