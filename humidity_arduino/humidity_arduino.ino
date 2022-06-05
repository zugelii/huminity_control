
#include <EEPROM.h>
#include "wiring_private.h"
#include <math.h> 
#include "msg.h"
#include "utils.h"
#include "bh1750.h"

U08 node_address;

//ego address map
//00 : 0x0B     //start
//01 : 0x01	    //adress
//02 : 's'/'m'  //mode s: slave   m: master
//03 : crc      //high
//04 : crc      //low

bool read_ego_address()
{
	U08 buf[4];
	U16 crc_res;
	bool res = true;
	buf[0] = EEPROM[0x00];
	buf[1] = EEPROM[0x01];
	buf[2] = EEPROM[0x02];
	buf[3] = EEPROM[0x03];
	buf[3] = EEPROM[0x04];
	crc_res = CRC(buf, 3);
	if ((EEPROM[0x00] != 0xb) ||
		(EEPROM[0x01] < 0x01) ||
		(EEPROM[0x03] != ((crc_res >> 0x8) & 0xff)) ||
		(EEPROM[0x04] != ((crc_res >> 0x0) & 0xff)))
	{
		Serial.println("the node address is wrong or no set");
		Serial.println("set the default addrss to 01");
		res = false;
	}
	else
	{
		node_address = buf[1];
	}
	return res;
}

bool write_ego_address(U08 addr, char mode)
{
	U08 buf[4];
	U16 crc_res;
	bool res = true;
	buf[0] = 0x0b;
	buf[1] = addr;
	buf[2] = mode;
	crc_res = CRC(buf, 3);
	buf[3] = (crc_res >> 0x8) & 0xff;
	buf[4] = (crc_res >> 0x0) & 0xff;
	EEPROM.write(0, buf[0]);
	EEPROM.write(1, buf[1]);
	EEPROM.write(2, buf[2]);
	EEPROM.write(3, buf[3]);
	EEPROM.write(4, buf[4]);
}

void setup()
{
    Wire.begin();
    Serial.begin(115200);
	//get node address
	if (false == read_ego_address())
	{
		Serial.println("please input the node address");
		while (!Serial.available()) {
			// wait for user input
		}
		U08 addr = Serial.parseInt();
		Serial.print("input address = ");
		Serial.println((int)addr);
		char mode = Serial.read();
		Serial.print("input mode = ");
		Serial.println(mode);
		write_ego_address(addr, mode);
	}
}





void loop()
{
    //BH1750_task();
}

