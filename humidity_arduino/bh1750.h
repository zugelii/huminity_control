// bh1750.h

#ifndef _BH1750_h
#define _BH1750_h

#include <Wire.h> //IIC

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

void BH1750_task();
#endif

