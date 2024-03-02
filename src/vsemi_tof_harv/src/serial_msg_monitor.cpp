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
	SerialConnection serial;
	serial.openConnection("/dev/ttyUSB1");

	int sender_count = 0;

	while (true)
	{
		try {
			int len = serial.readData(6);
			if (len > 0)
			{
				if (sender_count > 10)
				{
					sender_count = 0;
				}
				
				std::cerr << "Data received: " << len << std::endl;
				std::cerr << std::endl;
				for (int i = 0; i < len; i ++)
				{
					std::stringstream ss;
					ss << std::hex << std::setw(2) << static_cast<int>(serial.data_array[i]);
					std::cerr << ss.str() << " ";
				}
				std::cerr << std::endl;

				if (sender_count == 0)
				{
					//
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

					serial.sendData(data, 12);

					std::cerr << "Data Sent: " << len << std::endl;
					for (int i = 0; i < 12; i ++)
					{
						std::stringstream ss;
						ss << std::hex << std::setw(2) << static_cast<int>(data[i]);
						std::cerr << ss.str() << " ";
					}
					std::cerr << std::endl;
				}

				sender_count ++;
			}
		}
		catch(...)
		{
			std::cerr << " ... serial error " << std::endl;
		}
	}
}
