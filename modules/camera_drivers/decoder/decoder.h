// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: camera decoder

#pragma once

#include <vector>
#include <string>
#include "common/common.h"
#include "camera_drivers/proto/decoder_config.pb.h"

namespace sensor {
namespace hub {

class Decoder {
  // todo
};

REGISTER_COMPONENT(Decoder);
#define REGISTER_DECODER(name) REGISTER_CLASS(Decoder, name)

}  // namespace hub
}  // namespace sensor
