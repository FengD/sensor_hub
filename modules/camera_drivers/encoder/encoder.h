// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera encoder

#pragma once

#include <vector>
#include <string>
#include "camera_drivers/proto/encoder_config.pb.h"
#include "common/common.h"

namespace crdc {
namespace airi {

class Encoder {

};

REGISTER_COMPONENT(Encoder);
#define REGISTER_ENCODER(name) REGISTER_CLASS(Encoder, name)

}  // namespace airi
}  // namespace crdc