#include <Wire.h> //IIC
#include <EEPROM.h>
#include "wiring_private.h"
#include <math.h> 

typedef unsigned char U08;
typedef unsigned int  U16;
typedef unsigned long U32;

unsigned int CRC(U08 *data1, char length)
{
	int j;
	unsigned int rec_crc = 0xFFFF;
	while(length--)
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


int BH1750address = 0x23; 
byte buff[2];
void setup()
{
    Wire.begin();
    Serial.begin(115200);
}

int BH1750_Read(int address) //
{
    int i=0;
    Wire.beginTransmission(address);
    Wire.requestFrom(address, 2);
    while(Wire.available()) //
    {
        buff[i] = Wire.receive();  // receive one byte
        i++;
    }
    Wire.endTransmission();  
    return i;
}


void BH1750_Init(int address) 
{
    Wire.beginTransmission(address);
    Wire.send(0x10);//1lx reolution 120ms
    Wire.endTransmission();
}

void BH1750_task()
{
    int i;
    uint16_t val=0;
    BH1750_Init(BH1750address);
    delay(200);
    if(2 == BH1750_Read(BH1750address))
    {
        val=((buff[0]<<8)|buff[1])/1.2;
        Serial.print(val,DEC);     
        Serial.println("[lx]"); 
    }
    delay(150);
}


void loop()
{
    BH1750_task();
}

