// 
// 
// 

#include "bh1750.h"

int BH1750address = 0x23;
byte buff[2];
int BH1750_Read(int address) //
{
	int i = 0;
	Wire.beginTransmission(address);
	Wire.requestFrom(address, 2);
	while (Wire.available()) //
	{
		buff[i] = Wire.read();  // receive one byte
		i++;
	}
	Wire.endTransmission();
	return i;
}


void BH1750_Init(int address)
{
	Wire.beginTransmission(address);
	Wire.write(0x10);//1lx reolution 120ms
	Wire.endTransmission();
	delay(200);
}

void BH1750_task(uint16_t &val)
{
	if (2 == BH1750_Read(BH1750address))
	{
		val = ((buff[0] << 8) | buff[1]) / 1.2;
		Serial.print(val, DEC);
		Serial.println("[lx]");
	}
}