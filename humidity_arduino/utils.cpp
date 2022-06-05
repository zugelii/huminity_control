// 
// 
// 

#include "utils.h"


U16 CRC(U08 *data1, char length)
{
	int j;
	U16 rec_crc = 0xFFFF;
	while (length--)
	{
		rec_crc ^= *data1++;
		for (j = 0; j < 8; j++)
		{
			if (rec_crc & 0x01)
			{
				rec_crc = (rec_crc >> 1) ^ 0xA001;
			}
			else
			{
				rec_crc = rec_crc >> 1;
			}
		}
	}
	return rec_crc;
}

