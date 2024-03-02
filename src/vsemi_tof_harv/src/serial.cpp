#include "serial.hpp"

#include <stdio.h>
#include <iostream>
#include <iterator>

#include <termios.h>
#include <sys/ioctl.h>

SerialConnection::SerialConnection()
{
	fileDescription = 0;
}

SerialConnection::~SerialConnection()
{
	closeConnection();
}

bool SerialConnection::openConnection(std::string port)
{  
	if(fileDescription > 0) closeConnection();

	fileDescription = open(port.data(), O_RDWR | O_NOCTTY | O_SYNC);

	if(fileDescription <=0)
	{
		std::cerr << "Error opening serial port: " << errno << std::endl;
		return false;
	}

	ioctl(fileDescription, TIOCEXCL);

	std::cout << "Open serial port: " << fileDescription << std::endl;

	setInterfaceAttribs(B4000000);  // set speed to 10000000 bps, 8n1 (no parity)

	//rxArray.clear();

	return true;
}

/**
 * @brief Close the port
 *
 */
void SerialConnection::closeConnection()
{
	rxArray.clear();
	std::cout << "Closing serial port: " << fileDescription << std::endl;
	fileDescription = close(fileDescription);
}

/**
 * @brief Send a command
 *
 * @param data Pointer to the data to send
 */
size_t SerialConnection::sendData(uint8_t *data, int len)
{
	rxArray.clear();

	return write(fileDescription, data, len);
}

/**
 * @brief Slot called on reception
 *
 * This function is called, when data from the serial port has been received.
 */
int SerialConnection::readData(int data_len)
{
	data_array.clear();

	int len = data_len + 4 + 1 + 1; // 4 for header, 1 for len, 1 for end mark
	int n = 0;
	uint8_t buf[len] = { 0 };

	//std::cerr << "   ===> readData len: " << len << std::endl;

	n = read(fileDescription, buf, len);
	//std::cerr << "   ===> n: " << n << std::endl;

	if (n == -1) {
		std::cerr << "Error on readData." << std::endl;
		return -1;
	}
	
	rxArray.insert(rxArray.end(), buf, buf + n);

	if (receiveDataSegment())
	{
		//std::cout << "read valid n: " << n << std::endl;
		return data_array.size();
	} else
	{
		//std::cout << "read invalid n: " << n << std::endl;
		return -1;
	}
}

bool SerialConnection::receiveDataSegment()
{
	if (rxArray.size() == 0){
		std::cerr << "ERROR Connection::receiving data: array.size() = 0" << std::endl;
		return false;
	}

	if(rxArray.at(0) != 0xFF)
	{
		std::cerr << "ERROR Connection::receiving data: starting byte is not START_MARK" << std::endl;
		while (rxArray.size() > 0)
		{
			std::cerr << "   ... trying to erase the first byte" << std::endl;
			rxArray.erase(rxArray.begin(), rxArray.begin() + 1 );
			if(rxArray.size() > 0 && rxArray.at(0) == 0xFF)
			{
				break;
			}
		}
		if (rxArray.size() == 0)
		{
			return false;
		}
	}

	if (rxArray.size() > 1)
	{
		if(rxArray.at(1) == 0xFF)
		{
			rxArray.erase(rxArray.begin(), rxArray.begin() + 1 ); // the 0 position should be end mark in thiscase
		}
	}

	int expected_len = 0;

	if (rxArray.size() >= 5)
	{
		expected_len = rxArray.at(4);

		if (expected_len == 0)
		{
			rxArray.clear();
		}
	}
	//std::cout << "   expected_len: " << expected_len << std::endl;

	int total_len = expected_len + 4 + 1 + 1; // 4 for header, 1 for len, 1 for end mark
	//std::cout << "   total_len: " << total_len << std::endl;

	if (rxArray.size() < total_len)
	{
		return false;
	}

	//std::vector<uint8_t>::const_iterator first = rxArray.begin() + 5;
	std::vector<uint8_t>::const_iterator first = rxArray.begin(); // to include the header
	//std::vector<uint8_t>::const_iterator last = rxArray.begin() + 5 + expected_len;
	std::vector<uint8_t>::const_iterator last = rxArray.begin() + total_len; // to include end mark
	std::vector<uint8_t> tmp_data(first, last);
	//std::cout << "   tmp_data.size(): " << tmp_data.size() << std::endl;

	rxArray.erase(rxArray.begin(), rxArray.begin() + total_len); // remove current data section

	data_array = tmp_data;
	//std::cout << "   array left over size(): " << rxArray.size() << std::endl;

	return true;
}

int SerialConnection::setInterfaceAttribs(int speed)
{
	tcflush(fileDescription, TCIOFLUSH);

	struct termios tty;
	memset (&tty, 0, sizeof tty);
	bzero(&tty, sizeof(struct termios));

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 1;            // read block
	tty.c_cc[VTIME] = 30;           // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	//tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fileDescription, TCSANOW, &tty) != 0){
		std::cerr << "Error from tcsetattr: " << errno << std::endl;
		return -1;
	}

	return 0;
}

