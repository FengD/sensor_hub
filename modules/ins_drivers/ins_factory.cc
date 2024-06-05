// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: ins factory

#include "ins_drivers/ins.h"
#include "ins_drivers/input/socket/ins_socket.h"
#include "ins_drivers/input/pcap/pcap_input.h"
#ifndef WITH_ROS2
#include "ins_drivers/input/cyber/cyber_input.h"
#endif
#include "ins_drivers/parser/asensing_ins570d/asensing_ins570d.h"
#include "ins_drivers/parser/pcap/parser_pcap.h"

namespace crdc {
namespace airi {

REGISTER_INS_INPUT(InsSocketInput);
#ifndef WITH_ROS2
REGISTER_INS_INPUT(CyberInput);
#endif
REGISTER_INS_INPUT(InsPcapInput);
REGISTER_INS_PARSER(InsParser570d);
REGISTER_INS_PARSER(InsParserPcap);

}  // namespace airi
}  // namespace crdc

