// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: chassis factory

#include "chassis_drivers/chassis.h"
#include "chassis_drivers/input/someip_client/someip_client_input.h"
#ifdef WITH_TDA4
#include "chassis_drivers/parser/someip/parser_someip.h"
#endif

namespace crdc {
namespace airi {

REGISTER_CHASSIS_INPUT(SomeipClientInput);
#ifdef WITH_TDA4
REGISTER_CHASSIS_PARSER(ChassisParserSomeip);
#endif

}  // namespace airi
}  // namespace crdc

