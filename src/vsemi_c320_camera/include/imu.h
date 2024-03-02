/**
 * Copyright (C) 2022 Visionary Semiconductor Inc.
 *
 * @defgroup imu IMU
 * @brief IMU
 * @ingroup driver
 *
 * @{
 */
#ifndef VSEMI_IMU_H
#define VSEMI_IMU_H

#include <stdint.h>
#include <string>

//! IMU
/*!
 * IMU
 */
class IMU{
private:
	char chrBuf[100];
	unsigned char chrCnt=0;

	const char *port;
	int nSpeed;
	int nBits;
	char nEvent;
	int nStop;

	bool running = true;
	int ret;
	int fd;

	int openSerialPort();

	int configSerialPort();
	int sendData(char *send_buffer,int length);
	int recvData(char* recv_buffer,int length);

	void parseData(char chr);

	int closeSerialPort();

public:	
	float a[3],w[3],Angle[3],h[3];

	IMU(const char *portPath, int nSpeed, int nBits, char nEvent, int nStop);
	void start();
	void stop();
};

#endif // VSEMI_IMU_H

/** @} */
