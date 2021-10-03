// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera undistortion

#pragma once

#include <vector>
#include <string>
#include "common/common.h"

namespace crdc {
namespace airi {

class Undistortion {

};

REGISTER_COMPONENT(Undistortion);
#define REGISTER_UNDISTORTION(name) REGISTER_CLASS(Undistortion, name)

}  // namespace airi
}  // namespace crdc