// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Yuan SUN
// Description: socket write

#pragma once
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <unordered_map>
#include <string>
#include "common/common.h"
#include "lidar_drivers/proto/lidar_config.pb.h"

namespace crdc {
namespace airi {

#define PACKET_PREFIX             0xA55A
#define API_SET_TH_PARAMS_NUM     4
#define API_SET_TH_OPCODE_NUM     4
#define LSB_TIME(x)               ((x) & 0x00000000ffffffff)
#define MSB_TIME(x)               (((x) & 0xffffffff00000000) >> 32)
#define USER_API_CMD_TYPE_START   0xF000
#define USER_API_CMD_TYPE_END     0xFFFE

#define SENSITIVITY       8
#define NOISE_LEVEL       60
#define BIAS_4D           4
#define DYNAMIC_AZIMUTH   20
#define DYNAMIC_ELEVATION 5
#define MIN_DOPPLER       -100
#define MAX_DOPPLER       100

typedef struct TRAF_API_Header {
  uint16_t  usPrefix;
  uint16_t  usType;
  uint32_t  unLength;
  uint32_t  unTimeLsb;
  uint32_t  unTimeMsb;
  uint32_t  unMessageNumber;
}TRAF_API_Header, * PTRAF_API_Header;

typedef struct TStartTxInfo {
  uint32_t unReserved;
}TStartTxInfo, * PTStartTxInfo;

typedef struct TStopInfo {
  uint32_t unReserved;
}TStopInfo, * PTStopInfo;

typedef struct TStartTx {
  TRAF_API_Header tHeader;
  TStartTxInfo tStartTxInfo;
}TStartTx, * PTStartTx;

typedef struct TStopTx {
  TRAF_API_Header tHeader;
  TStopInfo tStopTxInfo;
}TStopTx, * PTStopTx;

typedef enum EThresholdOpcodes {
  SetStaticThresholdCoarseAndFine   = 0x0001,
  SetDynamicAzimuteThresholdCoarseFine = 0x0002,
  SetDynamicElevationThresholdFine  = 0x0003,
  SetStaticAndDynamicThresholds   = 0x0004,
  LastSetThresholdApiOpcode
}EThresholdOpcodes;

typedef struct TSetThresholdsInfo {
  EThresholdOpcodes  opcode;
  uint32_t aunParams[API_SET_TH_PARAMS_NUM];
}TSetThresholdsInfo, * PTSetThresholdsInfo;


typedef enum EApiCmdType {
  ApiCmdNone = 0x1000,
  SetTime,
  KeepAlive,
  Status,
  Start_Tx,
  Stop_Tx,
  RecordFrameRawData,
  ConfigurePointCloud,
  SetThresholds,
  SelectActiveSeq,
  RficOperation_Input,
  MemoryOperation_Input,
  DebugOperation_Input,
  VaractorTable,
  SetHistogram,
  ConfigureInjectPC,
  ConfigureRDrecording,
  ConfigureCalibrationFrame,
  GetNxPool,
  FreeNxPool,
  DataByReference,
  SetDspParams,
  FileLocation,
  GetMapFile,
  SetDspKernelsChain,
  SetProcessingLimits,
  ConfigureSeq,
  EnableFramePcOutput,
  GetNoiseVector,
  ConfigPacketFormat,
  SetFrameControlData,
  SetLogCfg,
  CfarMode,
  NtcMode,
  LastInternalCmdType,

  /********************** User Allocated ****************************************/
  UserCmd001 = USER_API_CMD_TYPE_START,
  UserCmd002,
  UserCmdLast = USER_API_CMD_TYPE_END,
  /********************** End of User    ****************************************/
  // Only 16 LSBits of EApiCmdType are used. Do not add commands here.
  LastApiCmdType  // 0xFFFF
}EApiCmdType;

typedef enum ESeuqenceType {
  IdleSeq = 0,
  CoarseShortSeq,
  CoarseMidSeq,
  CoarseLongSeq,
  FineShortSeq,
  FineMidSeq,
  FineLongSeq,
  CalibrationSeq,
  LabSeq,
  DelayCalibrationSeq,
  CoarseUltraLongSeq,
  FineUltraLongSeq,
  UserConfigureSeq1,
  UserConfigureSeq2,
  FastCalibration,
  LastSeq,
}ESeuqenceType;

typedef struct TSelectActiveSeqInfo {
  ESeuqenceType eSequenceType;
}TSelectActiveSeqInfo,  * PTSelectActiveSeqInfo;

typedef struct TSelectActiveSeq {
  TRAF_API_Header tHeader;
  TSelectActiveSeqInfo tSelectActiveSeqInfo;
}TSelectActiveSeq, * PTSelectActiveSeq;

typedef struct TSetThresholds {
  TRAF_API_Header tHeader;
  TSetThresholdsInfo tSetThresholdsInfo;
}TSetThresholds, * PTSetThresholds;

typedef struct TSetTimeInfo {
  uint32_t unInitateTimeLsb;
  uint32_t unInitateTimeMsb;
}TSetTimeInfo, *PTSetTimeInfo;

typedef struct TSetTime {
  TRAF_API_Header tHeader;
  TSetTimeInfo tSetTimeInfo;
}TSetTime, *PTSetTime;

class SocketWrite {
 public:
  SocketWrite();
  virtual ~SocketWrite();
  void radar_change_seq();
  void radar_set_params();
  void radar_start_transmit();
  void radar_stop_transmit();
  void radar_set_time();
  int32_t set_tcp_connect(const LidarInputConfig& config);

 private:
  void RAF_API_RdrCtrl_StartTx(uint32_t &MessageNumber);
  void BuildHeader(uint32_t &MessageNumber, uint32_t unCmdType,
          uint32_t unMessageSize, TRAF_API_Header* tHeader);
  void RAF_API_RdrCtrl_SetActiveSeq(uint32_t &unMessageNumber,
                                  TSelectActiveSeqInfo* info);
  void RAF_API_RdrCtrl_SetThresholds(uint32_t &MessageNumber,
                                          TSetThresholdsInfo* info);
  void RAF_API_RdrCtrl_StopTx(uint32_t &MessageNumber,
                                                TStopInfo* info);
  void RAF_API_SysCfg_SetTime(uint32_t &MessageNumber, TSetTimeInfo* info);
  uint32_t TransmitBuffer(uint8_t* buffer, uint32_t len);
  time_t GetSystemTime();

  int32_t ctrl_sock;
  std::string ctrlIp;
  std::string Mode;
  std::string RangeType;
  uint16_t ctrlPort;

  uint8_t Threshold3d;
  uint8_t Threshold4d;
  uint8_t DynamicAzimuth;
  uint8_t DynamicElevation;
  uint32_t MessageNumber;
  TStopInfo stopInfo;
  LidarInputConfig config_;
};

}  // namespace airi
}  // namespace crdc
