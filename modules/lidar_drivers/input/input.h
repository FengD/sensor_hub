// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: input

#pragma once

#include <memory>
#include <string>

#include "common/common.h"

namespace crdc {
namespace airi {

class LidarInput {

 public:
  LidarInput() = default;
  virtual ~LidarInput() = default;
  // virtual bool init(const C& config) {
  //   return false;
  // }

//   virtual int get_data() {
//     return 
//   }

//   virtual std::string name() const = 0;

//   bool 

//  protected:
//   C config_;
//   int 

};

REGISTER_COMPONENT(LidarInput);

}  // airi
}  // crdc