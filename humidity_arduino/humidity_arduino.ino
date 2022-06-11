
#include <SPI.h>
#include <RF24Network_config.h>
#include <RF24Network.h>
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include "rf_connection.h"
#include <Wire.h>
#include <EEPROM.h>
#include "wiring_private.h"
#include <math.h> 
#include "utils.h"
#include "bh1750.h"

#define DEBUG_ON

//nrf2401 mode
#define MY_CE_PIN       (5)
#define MY_CS_PIN       (2)
#define MY_RELAY_PIN    (4)

#define RADIO_CHANEL	(90)



void nrf_init(uint16_t addr);
uint16_t nrf_read(REQUEST_MSG& msg_);
bool nrf_write(REQUEST_MSG& msg_, uint16_t addr);

// instantiate an object for the nRF24L01 transceiver
RF24 radio(MY_CE_PIN, MY_CS_PIN); // using pin 5 for the CE pin, and pin 2 for the CSN pin

RF24Network network(radio);       // Network uses that radio
uint16_t payloadSize = 0;

U08 node_address;
char node_mode;

enum
{
	NODE_MASTER = 'm',
	NODE_SLAVE  = 's'
};

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
	buf[4] = EEPROM[0x04];
	crc_res = CRC(buf, 3);
	if ((EEPROM[0x00] != 0xb) ||
		(EEPROM[0x03] != ((crc_res >> 0x8) & 0xff)) ||
		(EEPROM[0x04] != ((crc_res >> 0x0) & 0xff)))
	{
#ifdef DEBUF_ON
		Serial.println("the node address is wrong or no set");
		Serial.println("set the default addrss to 01");
#endif // DEBUF_ON


		res = false;
	}
	else
	{
		node_address = buf[1];
		node_mode = buf[2];
	}
	return res;
}

bool write_ego_address(U08 addr, char mode)
{
	U08 buf[4];
	U16 crc_res;
	bool res = true;
	if ((mode == 's') || (mode = 'm'))
	{
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
		node_address = addr;
		node_mode = mode;
		Serial.print("write mode/address: ");
		Serial.print(node_mode);
		Serial.println(node_address);
	}
	else
	{
		res = false;
	}

	return res;
}

void setup()
{
	pinMode(MY_RELAY_PIN, OUTPUT);
	
    Wire.begin();
    Serial.begin(115200);
	//get node address
	U16 addr = 0;
	char mode = 's';
	if (false == read_ego_address())
	{
		Serial.println("please input the node mode: m/s");
		while (!Serial.available()) {
			// wait for user input
		}
		mode = Serial.read();
		Serial.print("input mode = ");
		Serial.println(mode);
		if (NODE_SLAVE == mode)
		{
			Serial.println("please input the slave node address");
			while (!Serial.available()) {
				// wait for user input
			}
			addr = Serial.parseInt();
			Serial.print("input address = ");
			Serial.println((int)addr);
			
		}
		bool res = write_ego_address(addr, mode);
		if (res == false)
		{
			Serial.println("can not get right set");
		}
	}

	//init rf2401
#ifdef DEBUG_ON
	Serial.print("system write mode/address: ");
	Serial.print(node_mode);
	Serial.print('/');
	Serial.println(node_address);
#endif
	nrf_init(node_address);
}



void nrf_init(uint16_t addr)
{
	if (!radio.begin())
	{
		Serial.println(F("radio hardware is not responding!!"));
		while (1) {} // hold in infinite loop
	}
	radio.setChannel(RADIO_CHANEL);
	network.begin(addr);
#ifdef DEBUG_ON
	printf_begin();             // needed only once for printing details
	radio.printPrettyDetails(); // (larger) function that prints human readable data
#endif
	payloadSize = sizeof(REQUEST_MSG);
}

uint16_t nrf_read(REQUEST_MSG& msg_)
{
	network.update();
	//read data from slave
	RF24NetworkHeader headers;                       // If so, grab it and print it out
	uint16_t payloadSize = network.peek(headers);    // Use peek() to get the size of the payload
	if (payloadSize > 0)
	{
		payloadSize = network.read(headers, &msg_, payloadSize); // Get the data
	}
	return payloadSize;
}
bool nrf_write(REQUEST_MSG& msg_, uint16_t addr)
{
	RF24NetworkHeader header(addr);
	bool ok = network.write(header, &msg_, sizeof(REQUEST_MSG));
	return ok;
}

bool nrf_write_ack(ACK_MSG& msg_, uint16_t addr)
{
	RF24NetworkHeader header(addr);
	bool ok = network.write(header, &msg_, sizeof(ACK_MSG));
	return ok;
}

void parse_command(REQUEST_MSG& msg_)
{
	switch (msg_.func)
	{
	case FUNC_CODE_OPERATE:
		digitalWrite(MY_RELAY_PIN, (U08)(msg_.data));
		ACK_MSG m_msg;
		m_msg.header = HEAD_MSG;
		m_msg.addr = 00;
		m_msg.func = msg_.func;
		m_msg.status = 0x0;
		m_msg.crc = CRC((U08*)&m_msg, sizeof(ACK_MSG) - 2);
		delay(100);
		nrf_write_ack(m_msg, 00);
		break;
	case FUNC_CODE_READ_UL:
		Serial.println("FUNC_CODE_READ_UL cmd");
		break;
	case FUNC_CODE_READ_TEMP:
		Serial.println("FUNC_CODE_READ_UL cmd");
		break;
	default:
		Serial.println("no such command");
		break;
	}
}

U32 value = 0;
bool wait_ack_flag = false;
void loop()
{
    //BH1750_task();
	network.update();
	

	if (NODE_MASTER == node_mode)
	{
		//write msg from uart
		
		if (false == wait_ack_flag)
		{
			value = (value == 0 ? 1 : 0);
			//test master send mesage
			Serial.println("master start send");
			Serial.print("wait_ack_flag: ");
			Serial.println(wait_ack_flag);
			REQUEST_MSG msg_;
			msg_.header = HEAD_MSG;
			msg_.addr = 0x01;
			msg_.data = value;
			msg_.func = FUNC_CODE_OPERATE;
			msg_.crc = CRC((U08*)&msg_, sizeof(REQUEST_MSG) - 2);
#ifdef DEBUG_ON
			Serial.print(msg_.header, HEX);
			Serial.print(F(": "));
			Serial.print(msg_.addr, HEX);
			Serial.print(F(": "));
			Serial.print(msg_.func, HEX);
			Serial.print(F(": "));
			Serial.print(msg_.data, HEX);
			Serial.print(F(": "));
			Serial.print(msg_.crc, HEX);
			Serial.print(F(": "));
#endif // _ON

			bool res = nrf_write(msg_, 0x01);
			if (res = true)
			{
				Serial.println(F("master write right"));
			}
			else
			{
				Serial.println(F("master write false"));
			}
			wait_ack_flag = true;
		}
		else if (true == wait_ack_flag)
		{
			ACK_MSG m_ack;
			//read data from slave
			RF24NetworkHeader headers;                       // If so, grab it and print it out
			uint16_t payloadSize = network.peek(headers);    // Use peek() to get the size of the payload
			if (payloadSize > 0)
			{
				network.read(headers, &m_ack, payloadSize); // Get the data
				Serial.println("master receive data");
				Serial.print(m_ack.header, HEX);
				Serial.print(F(": "));
				Serial.print(m_ack.addr, HEX);
				Serial.print(F(": "));
				Serial.print(m_ack.func, HEX);
				Serial.print(F(": "));
				Serial.print(m_ack.status, HEX);
				Serial.print(F(": "));
				Serial.print(m_ack.crc, HEX);
				Serial.print(F(": "));
				wait_ack_flag = 0;
			}	
		}
	}
	else
	{
		//read msg from rf2401
		REQUEST_MSG my_msg;
		U16 len = nrf_read(my_msg);
		
		if (len > 0)
		{
			U16 crc = CRC((U08*)&my_msg, len - 2);
			if ((crc == my_msg.crc) && (HEAD_MSG == my_msg.header))
			{
#ifdef DEBUG_ON
				Serial.print("Received packet, size: ");         // Print info about received data
				Serial.print(len);
				Serial.print(" ; ");
				// Uncomment below to print the entire payload  
				Serial.print(my_msg.header, HEX);
				Serial.print(F(": "));
				Serial.print(my_msg.addr, HEX);
				Serial.print(F(": "));
				Serial.print(my_msg.func, HEX);
				Serial.print(F(": "));
				Serial.print(my_msg.data, HEX);
				Serial.print(F(": "));
				Serial.print(my_msg.crc, HEX);
				Serial.print(F(": "));
				Serial.println();
#endif
				if (node_address == my_msg.addr)
				{
					parse_command(my_msg);
				}
			}
		}
	}

	delay(1000);
}

