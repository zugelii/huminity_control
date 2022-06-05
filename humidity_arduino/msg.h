
#ifndef _MSG_H
#define _MSG_H


#include <stdio.h>

typedef unsigned char U08;
typedef unsigned int  U16;
typedef unsigned long U32;

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
	STATUS status;
	U16 crc;
}ACK_MSG;

#endif // !MSG_H