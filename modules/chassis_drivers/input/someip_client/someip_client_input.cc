#include "chassis_drivers/input/someip_client/someip_client_input.h"

namespace crdc {
namespace airi {

bool SomeipClientInput::init(const ChassisInputConfig& config) {
  config_ = config;
  return true;
}

int32_t SomeipClientInput::get_chassis_data(Packet** packet) {
  // raw_packet_.port = config_.chassis_port();
  // raw_packet_.time_system = get_now_microsecond();
  // *packet = &raw_packet_;
  std::this_thread::sleep_for(std::chrono::milliseconds(9));
  return DeviceStatus::SUCCESS;
}

}  // namespace airi
}  // namespace crdc
