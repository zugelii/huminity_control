/*
 * @Author: gabe
 * @Date: 2022-04-26 23:08:35
 * @LastEditors: gabe
 * @LastEditTime: 2022-06-25 16:02:53
 * @Description: 
 */
#pragma once
#include <cmath>
#include <iostream>
#include <ctime>
#include <random>
#include <chrono>


#include <iomanip>
#include <sstream>
#include <fstream>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <termios.h>
#include <errno.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <sys/epoll.h>
#include <string>
#include <sys/msg.h>
#include "Helper/Log.h"
using namespace  std;

//#define DEBUG_M

#define MAX_TEXT 48
#define MAX_TIME_OUT  5
#define MAX_TRY_COUNT 3
#define SERIAL_BAUDRATE   115200

#pragma  pack(push, 1)
typedef unsigned char U08;
typedef unsigned short  U16;
typedef unsigned int U32;

#define HEAD_MSG  (0xaabb)
#define FUNC_CODE_OPERATE	(0x01)
#define FUNC_CODE_READ_UL	(0x03)
#define FUNC_CODE_READ_TEMP	(0x04)


typedef struct
{
	U16 header;
	U08 addr;
	U08 func;
	U32 data;
	U16 crc;
}REQUEST_MSG;

enum STATUS
{
	NORMAL_STATUS = 0x00,
	NO_CONNECT_STATUS,
	UNKNOW_STATUS
};

typedef struct
{
	U16 header;
	U08 addr;
	U08 func;
	U08 status;
	U16 crc;
}ACK_MSG;


struct msg_st
{
	long int msg_type;
	uint8_t text[MAX_TEXT];
};
#pragma pack(pop)
#define EPOLL_WAITE_TIME 2000
#define EPOLL_WAITE_MAX  1


class CSerialPort
{
public:
    CSerialPort(const char *PortName,
                int32_t BaudRate = 115200,
                int32_t ByteSize = 8,
                int32_t StopBits = 1,
                int8_t Parity = 'N');
    CSerialPort();
    ~CSerialPort();
    int32_t SetSpeed(int32_t, int32_t);
    int32_t SetParity(int32_t, int32_t, int32_t, int8_t, int32_t);
    int32_t SetUart(int &, const char *, int32_t, int32_t, int32_t,int8_t);
    int32_t UartEpoll(int32_t &efd, int32_t &sfd);
    int32_t GetSerialfd() const;
    int32_t GetEpollfd() const;
    int32_t SendMessage(string &buffer, uint32_t buf_size);
    int32_t ReadMessage(string &buffer, uint32_t buf_size);


    int32_t SendMessage(uint8_t *buffer, uint32_t buf_size);  
    int32_t ReadMessage(uint8_t *buffer, uint32_t buf_size);  
    bool isConnected();
    void CloseSerial();
private:
    int32_t serial_fd{-1};
    int32_t epoll_fd{-1};  //epoll fd
    struct epoll_event wait_event;
};

// uint16_t usMBCRC16(uint8_t *src, uint16_t srclen);
// int32_t serial_msg(int32_t &msgid, uint8_t addr, uint16_t t);
// int32_t serial_msg_on_off(int32_t &msgid, uint8_t addr, uint16_t status);



