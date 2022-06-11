
#ifndef _MSG_H
#define _MSG_H


#include <stdio.h>

typedef unsigned char U08;
typedef unsigned int  U16;
typedef unsigned long U32;

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

#endif // !MSG_H