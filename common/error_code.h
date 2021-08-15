// Copyright (C) 2020 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: error code

#pragma once

enum CommunicationStatus {
  ERR_MQTT_INIT = -23,
  ERR_MQTT_CONNECT = -22,
  ERR_MQTT_USER_PWD = -21,
  ERR_CAN_CLOSE = -20,
  ERR_CAN_OPEN = -19,
  ERR_BIND = -18,
  ERR_CAN_READ = -17,
  ERR_CAN_WRITE = -16,
  ERR_INTERFACEMTU = -15,
  ERR_INTERFACE = -14,
  ERR_IO = -13,
  CONNECT_ERROR = -12,
  UNKOWN_HOST_ERROR = -11,
  IP_EMPTY_ERROR = -10,
  SOCKET_SETOPT_ERROR = -9,
  NON_BLOCKERR = -8,
  SOCKET_BIND_ERROR = -7,
  SOCKET_OPEN_ERROR = -6,
  MSG_SEND_ERROR = -5,
  MSG_RECEIVE_ERROR = -4,
  TIMEOUT_ERROR = -3,
  POLL_ERROR = -2,
  DEVICE_ERROR = -1,
  NO_ERROR = 0
};