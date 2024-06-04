/**
 * @file parser_someip.h
 * @author yuanyuan.wang3 (yuanyuan.wang3@hirain.com)
 * @brief chassis parser for someip
 * @date 2023-04-26
 * @license Modified BSD Software License Agreement
 * @copyright Copyright (C) 2023 Hirain Technologies Inc.
 * 
 */

#pragma once
#include <memory>
#include <string>
#include "common/common.h"
#include "chassis_drivers/parser/parser.h"
#ifdef PROJ_V201_HAV02
#include "3rdparty/someip_v201_hav02/client_adapter/include/srr_client_adapter.h"
#include "3rdparty/someip_v201_hav02/client_adapter/include/vehicle_client_adapter.h"
#include "3rdparty/someip_v201_hav02/client_adapter/include/preprogramming_client_adapter.h"
#elif defined(PROJ_V210_HAV30)
#include "3rdparty/someip_v210_hav30/client_adapter/include/srr_client_adapter.h"
#include "3rdparty/someip_v210_hav30/client_adapter/include/vehicle_client_adapter.h"
#include "3rdparty/someip_v210_hav30/client_adapter/include/preprogramming_client_adapter.h"
#include "3rdparty/someip_v210_hav30/client_adapter/include/mbstatusinfomation_client_adapter.h"
#include "3rdparty/someip_v210_hav30/client_adapter/include/aduvehiclebodyregiinfo_client_adapter.h"
#include "3rdparty/someip_v210_hav30/client_adapter/include/quaysidecraneinformation_client_adapter.h"
#include "3rdparty/someip_v210_hav30/client_adapter/include/cameraposeinformation_client_adapter.h"
#endif

namespace crdc {
namespace airi {

class ChassisParserSomeip : public ChassisParser {
 public:
  ChassisParserSomeip() = default;
  virtual ~ChassisParserSomeip() = default;

  bool init_chassis_parser() override;

  uint64_t get_packet_timestamp(const Packet* packet) override { (void)packet; return 1; };
  void parse_chassis_can_frame(char* can_frame_) {}

  std::string get_name() const override {
    return "ChassisParserSomeip";
  }

 private:
#ifdef PROJ_V201_HAV02
  SRRClient srr_client_;
  VehicleClient vehicle_client_;
  PreProgrammingClient pre_programming_client_;
#elif defined(PROJ_V210_HAV30)
  SRRClient srr_client_;
  VehicleClient vehicle_client_;
  PreProgrammingClient pre_programming_client_;
  MbstatusinfomationClient mbstatusinfomation_client_;
  ADUVehicleBodyRegiInfoClient aduvehiclebodyregiinfo_client_;
  QuaysideCraneInformationClient quaysidecraneinformation_client_;
  CameraPoseInformationClient cameraposeinformation_client_;
#endif
};

}  // namespace airi
}  // namespace crdc
