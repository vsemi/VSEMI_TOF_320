#include "imu.h"

#include <iostream>
#include <string>
#include <thread>
#include <map>
#include <iterator>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <termios.h>
#include <sys/time.h>
#include <time.h>
#include <sys/types.h>
#include <errno.h>

#include "boost/endian/conversion.hpp"

using namespace std;

IMU::IMU(const char *portPath, int nSpeed, int nBits, char nEvent, int nStop)
{
	this->port = portPath;
	this->nSpeed = nSpeed;
	this->nBits = nBits;
	this->nEvent = nEvent;
	this->nStop = nStop;
}
int IMU::openSerialPort()
{
    fd = open(port, O_RDWR|O_NOCTTY); 
    if (-1 == fd)
    { 
        perror("Can't Open Serial Port"); 
		return(-1); 
	} 
    else
		printf("Open Serial Port %s successfully!\n", port);
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("Serial Port %s is working properly!\n", port); 
    return fd; 
}

int IMU::configSerialPort()
{
	struct termios newtio,oldtio; 
	if  ( tcgetattr( fd,&oldtio)  !=  0) {  
		perror("SetupSerial 1");
		printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
		return -1; 
	} 
	bzero( &newtio, sizeof( newtio ) ); 
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  
	switch( nBits ) 
	{ 
		case 7: 
			newtio.c_cflag |= CS7; 
			break; 
		case 8: 
			newtio.c_cflag |= CS8; 
			break; 
	} 
	switch( nEvent ) 
	{ 
		case 'o':
		case 'O': 
			newtio.c_cflag |= PARENB; 
			newtio.c_cflag |= PARODD; 
			newtio.c_iflag |= (INPCK | ISTRIP); 
			break; 
		case 'e':
		case 'E': 
			newtio.c_iflag |= (INPCK | ISTRIP); 
			newtio.c_cflag |= PARENB; 
			newtio.c_cflag &= ~PARODD; 
			break;
		case 'n':
		case 'N': 
			newtio.c_cflag &= ~PARENB; 
			break;
		default:
			break;
	} 

	switch( nSpeed ) 
	{ 
		case 2400: 
			cfsetispeed(&newtio, B2400); 
			cfsetospeed(&newtio, B2400); 
			break; 
		case 4800: 
			cfsetispeed(&newtio, B4800); 
			cfsetospeed(&newtio, B4800); 
			break; 
		case 9600: 
			cfsetispeed(&newtio, B9600); 
			cfsetospeed(&newtio, B9600); 
			break; 
		case 115200: 
			cfsetispeed(&newtio, B115200); 
			cfsetospeed(&newtio, B115200); 
			break; 
		case 460800: 
			cfsetispeed(&newtio, B460800); 
			cfsetospeed(&newtio, B460800); 
			break; 
		default: 
			cfsetispeed(&newtio, B9600); 
			cfsetospeed(&newtio, B9600); 
			break; 
	} 
	if( nStop == 1 ) 
		newtio.c_cflag &=  ~CSTOPB; 
	else if ( nStop == 2 ) 
		newtio.c_cflag |=  CSTOPB; 
	
	newtio.c_cc[VTIME]  = 0; 
	newtio.c_cc[VMIN] = 0; 
	tcflush(fd,TCIFLUSH); 

	if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
	{ 
		perror("com set error"); 
		return -1; 
	} 

	printf("Serial Port %s configured successfully!\n", port);
	return 0; 
}

int IMU::closeSerialPort()
{
	assert(fd);
	close(fd);

	return 0;
}

int IMU::sendData(char *send_buffer,int length)
{
	length=write(fd,send_buffer,length*sizeof(unsigned char));
	return length;
}
int IMU::recvData(char* recv_buffer,int length)
{
	length=read(fd,recv_buffer,length);
	//std::cout << "length: " << length << std::endl;
	//std::cout << std::string(recv_buffer) << std::endl;
	return length;
}

void IMU::start()
{
    char r_buf[1024];
    bzero(r_buf,1024);

    openSerialPort();
    if(fd == -1)
    {
        fprintf(stderr, "Serial port open error\n");
        exit(EXIT_FAILURE);
    }

    if(configSerialPort() == -1)
    {
        fprintf(stderr, "Serial port set failed!\n");
        exit(EXIT_FAILURE);
    }
	std::cerr << "IMU configured. "<< std::endl;

	// may or may not need these command, but I can't test as my IMU does not work:
	//char imu_set_return_rate[] = { 0x50, 0x06, 0x00, 0x03, 0x00, 0x08, 0x00, 0x00 };
	//char imu_set_baud[]        = { 0x50, 0x06, 0x00, 0x04, 0x00, 0x02, 0x00, 0x00 };
	//send_data(fd, imu_set_return_rate, 8);
	//send_data(fd, imu_set_baud, 8);
	
    while(running)
    {
        ret = recvData(r_buf, 44);
		//std::cerr << "   IMU ret: " << ret << std::endl;
        if(ret == -1)
        {
            fprintf(stderr,"Serial port read failed!\n");
            exit(EXIT_FAILURE);
        }

		for (int i=0; i<ret; i++) 
		{
			parseData(r_buf[i]);			
		}
		// sleep for a minisecond in sample code, may or may not sleep that long, change to 0.1 minisecond?
		//usleep(1000);
		usleep(1000);
	}

	closeSerialPort();
}

void IMU::stop()
{
	running = false;
}

void IMU::parseData(char chr)
{
		signed short sData[4];
		unsigned char i;
		char cTemp=0;
		time_t now;
		chrBuf[chrCnt++]=chr;
		if (chrCnt<11) return;
		for (i=0;i<10;i++) cTemp+=chrBuf[i];
		if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) 
		{
			//printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);
			std::memcpy(&chrBuf[0],&chrBuf[1],10);
			chrCnt--;
			return;
		}
		
		std::memcpy(&sData[0],&chrBuf[2],8);
		switch(chrBuf[1])
		{
				case 0x51:
					for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
					time(&now);
					//printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
					
					break;
				case 0x52:
					for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
					//printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);					
					break;
				case 0x53:
					for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
					//printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
					break;
				case 0x54:
					for (i=0;i<3;i++) h[i] = (float)sData[i];
					//printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);
					
					break;
		}		
		chrCnt=0;		
}
