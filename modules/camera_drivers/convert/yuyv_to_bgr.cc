// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera yuyv_to_bgr

#include "camera_drivers/convert/convert.h"

namespace crdc {
namespace airi {

bool Convert::yuyv_to_bgr(const unsigned char *src, unsigned char *dst,
                          const int& width, const int& height,
                          const std::string& sensor) {
    if (width <= 0) {
        LOG(ERROR) << "[" << sensor << "] width input error: " << width << std::endl;
        return false;
    }

    if (height <= 0) {
        LOG(ERROR) << "[" << sensor << "] height input error: " << height << std::endl;
        return false;
    }

    if (width % 8 != 0) {
        LOG(ERROR) << "[" << sensor << "] width must be a multiple of 8: " << width << std::endl;
        return false;
    }

    if (src == nullptr) {
        LOG(ERROR) << "[" << sensor << "] src input error." << std::endl;
        return false;
    }

    if (dst == nullptr) {
        LOG(ERROR) << "[" << sensor << "] dst input error." << std::endl;
        return false;
    }

    // todo

    return true;
}

}  // namespace airi
}  // namespace crdc