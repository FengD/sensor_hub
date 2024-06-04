#include "chassis_drivers/parser/someip/parser_someip.h"

namespace crdc {
namespace airi {

bool ChassisParserSomeip::init_chassis_parser() {
#ifdef PROJ_V201_HAV02
  srr_client_.someip_start();
  vehicle_client_.someip_start();
  pre_programming_client_.someip_start();
#elif defined(PROJ_V210_HAV30)
  srr_client_.someip_start();
  vehicle_client_.someip_start();
  pre_programming_client_.someip_start();
  mbstatusinfomation_client_.someip_start();
  aduvehiclebodyregiinfo_client_.someip_start();
  quaysidecraneinformation_client_.someip_start();
  cameraposeinformation_client_.someip_start();
#endif
  return true;
}

}  // namespace airi
}  // namespace crdc
