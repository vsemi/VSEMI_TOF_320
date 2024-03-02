/**
 * Copyright (C) 2023 Visionary Semiconductor Inc.
 *
 * @{
 */
#ifndef SERIAL_CONNECTION_H
#define SERIAL_CONNECTION_H

#include <memory.h>
#include <vector>
#include <string>
#include <list>
#include <iostream>
#include <cstring>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>

//! implementation of serial port
/*!
* This class implements some specific functionality for the serial communication
*/
class SerialConnection
{    
public:
	SerialConnection();
	~SerialConnection();

	bool openConnection(std::string port);
	void closeConnection();

	size_t sendData(uint8_t *data, int len);
	int readData(int len);

	std::vector<uint8_t> data_array;

private:
	std::string port;

	std::vector<uint8_t> rxArray;

	int fileDescription;

	int setInterfaceAttribs (int speed);

	bool receiveDataSegment();
};

#endif // SERIAL_CONNECTION_H

/** @} */
