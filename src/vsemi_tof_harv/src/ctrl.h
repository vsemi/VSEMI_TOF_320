#pragma once

#include <iostream>
#include <time.h>
#include <string>
#include <chrono>

#ifndef _CAN_CTRL_H_
#define _CAN_CTRL_H_

const float CTRL_SIGNAL_INTERVAL = 0.1;
const float CTRL_SIGNAL_CHANGE_DIRECTION_MIN_PERIOD = 0.2;
const int   CTRL_SIGNAL_START_X = 1;
const int   CTRL_SIGNAL_REPEAT_PER_METER_X = 2;
const int   CTRL_SIGNAL_START_Y = 0;
const int   CTRL_SIGNAL_REPEAT_PER_METER_Y = 1;

const float CTRL_SIGNAL_MIN_DISTANCE_X = 0.5;
const float CTRL_SIGNAL_MIN_DISTANCE_Y = 0.5;

// - SocketCan -------------------------------------------------------------------------------------------------------------
const int CAN_ID_CTRL   = 0x98eeeeef;
const int CAN_ID_SYSTEM = 0x99eeeeef;
const int CAN_ID_ANGLES = 0x9aeeeeef;
const int CAN_ID_MANUAL_INTERFERED = 0x9beeeeef;

void update_ctrl_signal();
void ctrl_rcv();
void ctrl_send();

#endif
