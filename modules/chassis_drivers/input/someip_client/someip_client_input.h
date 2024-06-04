#pragma once

#include <string>
#include <memory>
#include "chassis_drivers/input/input.h"

namespace crdc {
namespace airi {

class SomeipClientInput : public ChassisInput {
 public:
  SomeipClientInput() = default;
  virtual ~SomeipClientInput() = default;

  bool init(const ChassisInputConfig& config) override;
  std::string get_name() const override {
    return "SomeipClientInput";
  }
  int32_t get_chassis_data(Packet** packet) override;
//  private:
//   Packet raw_packet_;
};

}  // namespace airi
}  // namespace crdc
