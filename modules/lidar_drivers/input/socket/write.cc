// Copyright (C) 2020 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: socket write

#include "lidar_drivers/input/socket/write.h"

namespace sensor {
namespace hub {

SocketWrite::SocketWrite() {
  MessageNumber = 0;
  ctrl_sock = -1;
  ctrlIp = "172.20.2.210";
  Mode = "4d";
  RangeType = "Mid";
  ctrlPort = 6001;
  Threshold4d = SENSITIVITY + NOISE_LEVEL;
  Threshold3d = Threshold4d - BIAS_4D;
  DynamicAzimuth = DYNAMIC_AZIMUTH;
  DynamicElevation = DYNAMIC_ELEVATION;
}

SocketWrite::~SocketWrite() {
  RAF_API_RdrCtrl_StopTx(MessageNumber, &stopInfo);
  close(ctrl_sock);
}

/*
   Send a provide data buffer to the radar over the control socket
*/
uint32_t SocketWrite::TransmitBuffer(uint8_t* buffer, uint32_t len) {
  if (write(ctrl_sock, buffer, len) < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to write tcp socket.";
    return DeviceStatus::SOCKET_SETOPT_ERROR;
  }
  return DeviceStatus::NO_ERROR;
}

/*
   provides the system time needed by the radar framework (RAF)
*/
time_t SocketWrite::GetSystemTime() {
  time_t tt;
  time(&tt);
  return tt;
}

int32_t SocketWrite::set_tcp_connect(const LidarInputConfig& config) {
  config_ = config;
  if (config_.control_config().has_port())
    ctrlPort = config_.control_config().port();
  if (config_.control_config().has_ip())
    ctrlIp = config_.control_config().ip();
  if (config_.control_config().has_mode())
    Mode = config_.control_config().mode();
  if (config_.control_config().has_range_type())
    RangeType = config_.control_config().range_type();

  ctrl_sock = socket(PF_INET, SOCK_STREAM, 0);
  if (ctrl_sock < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] failed to create tcp socket.";
    return DeviceStatus::CREATE_SOCKET_ERROR;
  }
  int one = 1;
  struct timeval timeout;
  timeout.tv_sec  = 3;  // after 2 seconds connect() will timeout
  timeout.tv_usec = 0;
  setsockopt(ctrl_sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  setsockopt(ctrl_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  struct sockaddr_in ctrl_serv_addr;
  memset(&ctrl_serv_addr, 0, sizeof(ctrl_serv_addr));
  ctrl_serv_addr.sin_family = AF_INET;
  ctrl_serv_addr.sin_addr.s_addr = inet_addr(ctrlIp.c_str());
  ctrl_serv_addr.sin_port = htons(ctrlPort);
  if (connect(ctrl_sock, (struct sockaddr*)&ctrl_serv_addr, sizeof(ctrl_serv_addr)) < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] Failed to connect to IP " << ctrlIp;
    close(ctrl_sock);
    return DeviceStatus::IP_EMPTY_ERROR; /* Completely fail only if first radar didn't connect */
  }
  if (fcntl(ctrl_sock, F_SETFL, O_NONBLOCK) < 0) {
    LOG(ERROR) << "[" << config_.frame_id() << "] non block port " << ctrlPort;
    close(ctrl_sock);
    return DeviceStatus::NON_BLOCK_ERROR;
  }
  return ctrl_sock;
}

void SocketWrite::radar_change_seq() {
  TSelectActiveSeqInfo sequenceInfo;
  std::unordered_map<std::string, int> rangetype_map = {
    {"Short", 0}, {"Mid", 1}, {"Long", 2}, {"Ultra-Long", 3}, {"Fast-Calibration", 4} };
  int type = rangetype_map.at(RangeType);
  if (Mode == "3d") {
    switch (type) {
      case 0: sequenceInfo.eSequenceType = CoarseShortSeq; break;
      case 1: sequenceInfo.eSequenceType = CoarseMidSeq; break;
      case 2: sequenceInfo.eSequenceType = CoarseLongSeq; break;
      case 3: sequenceInfo.eSequenceType = CoarseUltraLongSeq; break;
      default: sequenceInfo.eSequenceType = FineMidSeq; break;
    }
  } else if (Mode == "4d") {
    switch (type) {
      case 0: sequenceInfo.eSequenceType = FineShortSeq; break;
      case 1: sequenceInfo.eSequenceType = FineMidSeq; break;
      case 2: sequenceInfo.eSequenceType = FineLongSeq; break;
      case 3: sequenceInfo.eSequenceType = FineUltraLongSeq; break;
      case 4: sequenceInfo.eSequenceType = FastCalibration; break;
      default: sequenceInfo.eSequenceType = FineMidSeq; break;
    }
  } else {
    sequenceInfo.eSequenceType = FineMidSeq;  // Default to 4d Mid if not defined
  }

  RAF_API_RdrCtrl_SetActiveSeq(MessageNumber, &sequenceInfo);
}

void SocketWrite::radar_set_time() {
  TSetTimeInfo time_info;
  struct timeval time_now;
  /* Set radar time as system time */
  gettimeofday(&time_now, NULL);
  uint64_t msecs_time = ((uint64_t)(time_now.tv_sec) * 1000) +
                        ((uint64_t)(time_now.tv_usec) / 1000);
  time_info.unInitateTimeLsb = (uint32_t)((msecs_time & 0xffffffffffffffff));
  time_info.unInitateTimeMsb = (uint32_t)((msecs_time >> 32) & 0xffff);

  RAF_API_SysCfg_SetTime(MessageNumber, &time_info);
}

void SocketWrite::radar_set_params() {
  TSetThresholdsInfo radarThresholds;
  radarThresholds.opcode = SetStaticAndDynamicThresholds;
  radarThresholds.aunParams[0] = (uint32_t)(Threshold3d * (16/3));
  radarThresholds.aunParams[1] = (uint32_t)(Threshold4d  * (16/3));
  radarThresholds.aunParams[2] = (uint32_t)(DynamicAzimuth * (16/3));
  radarThresholds.aunParams[3] = (uint32_t)(DynamicElevation * (16/3));
  RAF_API_RdrCtrl_SetThresholds(MessageNumber, &radarThresholds);
}

void SocketWrite::radar_start_transmit() {
  RAF_API_RdrCtrl_StartTx(MessageNumber);
}

void SocketWrite::radar_stop_transmit() {
  RAF_API_RdrCtrl_StopTx(MessageNumber, &stopInfo);
}

void SocketWrite::RAF_API_SysCfg_SetTime(uint32_t &MessageNumber, TSetTimeInfo* info) {
  TSetTime tSetTime;
  int _size = sizeof(TSetTime);
  memset(&tSetTime, 0, _size);
  tSetTime.tSetTimeInfo.unInitateTimeLsb = info->unInitateTimeLsb;
  tSetTime.tSetTimeInfo.unInitateTimeMsb = info->unInitateTimeMsb;
  BuildHeader(MessageNumber, SetTime, _size, &tSetTime.tHeader);
  TransmitBuffer(reinterpret_cast<uint8_t*>(&tSetTime), _size);
}

void SocketWrite::RAF_API_RdrCtrl_SetThresholds(uint32_t &MessageNumber,
                                          TSetThresholdsInfo* info) {
  TSetThresholds thresholdsCmd;
  int _size = sizeof(TSetThresholds);
  memset(&thresholdsCmd, 0, _size);
  BuildHeader(MessageNumber, SetThresholds, _size, &thresholdsCmd.tHeader);

  thresholdsCmd.tSetThresholdsInfo.opcode = info->opcode;
  thresholdsCmd.tSetThresholdsInfo.aunParams[0] = info->aunParams[0];
  thresholdsCmd.tSetThresholdsInfo.aunParams[1] = info->aunParams[1];
  thresholdsCmd.tSetThresholdsInfo.aunParams[2] = info->aunParams[2];
  thresholdsCmd.tSetThresholdsInfo.aunParams[3] = info->aunParams[3];

  TransmitBuffer(reinterpret_cast<uint8_t*>(&thresholdsCmd), _size);
}

void SocketWrite::RAF_API_RdrCtrl_SetActiveSeq(uint32_t &unMessageNumber,
                                  TSelectActiveSeqInfo* info) {
  TSelectActiveSeq selectActiveSeqCmd;
  int _size = sizeof(selectActiveSeqCmd);
  memset(&selectActiveSeqCmd, 0, _size);
  BuildHeader(unMessageNumber, SelectActiveSeq, _size, &selectActiveSeqCmd.tHeader);
  selectActiveSeqCmd.tSelectActiveSeqInfo.eSequenceType = info->eSequenceType;
  TransmitBuffer(reinterpret_cast<uint8_t*>(&selectActiveSeqCmd), _size);
}

void SocketWrite::RAF_API_RdrCtrl_StartTx(uint32_t &MessageNumber) {
  TStartTx startCmd;
  int _size = sizeof(TStartTx);
  memset(&startCmd, 0, _size);
  BuildHeader(MessageNumber, Start_Tx, _size, &startCmd.tHeader);
  TransmitBuffer(reinterpret_cast<uint8_t*>(&startCmd), _size);
}

void SocketWrite::RAF_API_RdrCtrl_StopTx(uint32_t &MessageNumber, TStopInfo* info) {
  TStopTx stopCmd;
  int _size = sizeof(TStopTx);
  memset(&stopCmd, 0, _size);
  BuildHeader(MessageNumber, Stop_Tx, _size, &stopCmd.tHeader);
  stopCmd.tStopTxInfo.unReserved = info->unReserved;
  TransmitBuffer(reinterpret_cast<uint8_t*>(&stopCmd), _size);
}

void SocketWrite::BuildHeader(uint32_t &MessageNumber, uint32_t unCmdType,
                uint32_t unMessageSize, TRAF_API_Header* tHeader) {
  tHeader->usPrefix = PACKET_PREFIX;
  tHeader->unLength = unMessageSize;
  tHeader->usType = unCmdType;
  tHeader->unMessageNumber = MessageNumber++;
  if (GetSystemTime() != time_t(-1)) {
    uint64_t ulSystemTime = GetSystemTime();
    tHeader->unTimeLsb = (uint32_t)LSB_TIME(ulSystemTime);
    tHeader->unTimeMsb = (uint32_t)MSB_TIME(ulSystemTime);
  } else {
    tHeader->unTimeLsb = 0;
    tHeader->unTimeMsb = 0;
  }
}


}  // namespace hub
}  // namespace sensor
