#pragma once

#include <memory>
#include <string>

#include "common/common.h"

namespace crdc {
namespace airi {

template <typename C, typename D>
class SensorInput {

 public:
  SensorInput() = default;
  virtual ~SensorInput() = default;
  virtual bool init(const C& config) {
    return false;
  }

  virtual int get_data() {
    return 
  }

  virtual std::string name() const = 0;

  bool 

 protected:
  C config_;
  int 

};

REGISTER_COMPONENT(SensorInput);

}  // airi
}  // crdc