// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins factory

#include "ins_drivers/ins.h"
#include "ins_drivers/input/socket/ins_socket.h"
#include "ins_drivers/parser/asensing_ins570d/asensing_ins570d.h"

namespace crdc {
namespace airi {

REGISTER_INS_INPUT(InsSocketInput);

REGISTER_INS_PARSER(InsParser570d);

}  // namespace airi
}  // namespace crdc

