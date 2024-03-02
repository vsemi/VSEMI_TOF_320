#include "serial.hpp"

#include <stdio.h>
#include <iomanip>
#include <ios>
#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <chrono>
#include <thread>

int main(int argc, char **argv)
{
	// test serial ports:
	//std::thread th_serial_rcv(&serial_rcv);
	//th_serial_rcv.detach();

	SerialConnection serial;
	serial.openConnection("/dev/ttyUSB1");
	uint8_t *data = new uint8_t[12]; 
	data[0]  = 0xFF;
	data[1]  = 0x00;
	data[2]  = 0x00;
	data[3]  = 0x01;

	data[4]  = 0x06;

	data[5]  = 0x02;
	data[6]  = 0x00;
	data[7]  = 0x00;
	data[8]  = 0x00;
	data[9]  = 0x00;
	data[10]  = 0x00;

	data[11] = 0xFF;
	//////////////////
    /*
	data[12]  = 0xFF;
	data[13]  = 0x00;
	data[14]  = 0x00;
	data[15]  = 0x02;

	data[16]  = 0x06;

	data[17]  = 0x40;
	data[18]  = 0x00;
	data[19]  = 0x00;
	data[20]  = 0x00;
	data[21]  = 0x00;
	data[22]  = 0x00;

	data[23] = 0xFF;
    */

	serial.sendData(data, 12);

    std::cerr << std::endl;
    for (int i = 0; i < 12; i ++)
    {
        std::stringstream ss;
        ss << std::hex << std::setw(2) << static_cast<int>(data[i]);
        std::cerr << ss.str() << " ";
    }
    std::cerr << std::endl;
}