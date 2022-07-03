
#include <SPI.h>
#include <RF24Network_config.h>
#include <RF24Network.h>
#include <RF24_config.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "wiring_private.h"
#include <math.h> 
#include "utils.h"
#include "bh1750.h"

//#define DEBUG_ON
//#define MASTER_MODE 
#define RELAY_ADDRESS	(1)

//nrf2401 mode
#define MY_CE_PIN       (5)
#define MY_CS_PIN       (2)
#define MY_RELAY_PIN    (4)

#define RADIO_CHANEL	(90)

#define REQUEST_LENGTH	(10)
#define ACK_LENGTH		(8)

#define MASTER_ADDRESS	(0)

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

enum
{
	STATUS_OK = 0,
	STATUS_NO_CONNECTION,
	STATUS_OTHER
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

//void setup()
//{
//	Wire.begin();
//	Serial.begin(115200);
//	node_mode = NODE_MASTER;
//}

void setup()
{
	wdt_disable();
	pinMode(MY_RELAY_PIN, HIGH);
	pinMode(MY_RELAY_PIN, OUTPUT);
	
    Wire.begin();
    Serial.begin(115200);
	//get node address
	U16 addr = 0;
	char mode = NODE_SLAVE;
#ifdef MASTER_MODE
	mode = NODE_MASTER;
#endif // 
	node_mode = mode;
	node_address = RELAY_ADDRESS;


#if 0
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
#endif
	//init rf2401

	if (NODE_MASTER == node_mode)
	{
		node_address = MASTER_ADDRESS;
	}
#ifdef DEBUG_ON
	Serial.print("system write mode/address: ");
	Serial.print(mode);
	Serial.print('/');
	Serial.println(node_address);
#endif
	wdt_enable(WDTO_4S);
	nrf_init(node_address);
}



void nrf_init(uint16_t addr)
{
	char re_check = 0;
	while (!radio.begin())
	{
		Serial.println(F("radio hardware is not responding!!"));
		delay(1000);
		if (re_check++ > 5)
		{
			//reboot 
			while (1)
			{

			}
		}
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
bool nrf_write_command(REQUEST_MSG& msg_)
{
	RF24NetworkHeader header(msg_.addr);
	bool res = network.write(header, &msg_, sizeof(REQUEST_MSG));
	return res;
}

bool nrf_write_ack(ACK_MSG& msg_)
{
	RF24NetworkHeader header(msg_.addr);
	bool res = network.write(header, &msg_, sizeof(ACK_MSG));
	return res;
}

bool send_ack(uint16_t to_addr, U08 func, U08 sts)
{
	ACK_MSG m_msg;
	bool res = false;
	m_msg.header = HEAD_MSG;
	m_msg.addr = (U08)to_addr;
	m_msg.func = func;
	m_msg.status = sts;
	m_msg.crc = CRC((U08*)&m_msg, sizeof(ACK_MSG) - 2);
	
	if (NODE_MASTER == node_mode)  //this is master node then only ack by uart
	{
		Serial.write((char *)&m_msg, sizeof(m_msg));
		res = true;
	}
	else
	{
		delay(100);
		res = nrf_write_ack(m_msg);
	}
	return res;
}

bool send_command(uint16_t to_addr, U08 func, U32 val)
{
	REQUEST_MSG msg_;
	bool res;
	msg_.header = HEAD_MSG;
	msg_.addr = to_addr;
	msg_.func = func;
	msg_.data = val;	
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
	Serial.println(msg_.crc, HEX);
#endif // _ON
	//if (NODE_MASTER == node_mode)
	//{
	//	Serial.write((char*)&msg_, sizeof(msg_));
	//	res = true;
	//}
	//else
	{
		res = nrf_write_command(msg_);
	}
	
	return res;
}

void parse_command(REQUEST_MSG& msg_)
{
	switch (msg_.func)
	{
	case FUNC_CODE_OPERATE:
		digitalWrite(MY_RELAY_PIN, (U08)(msg_.data));
		send_ack(msg_.addr, msg_.func, STATUS_OK);
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
U08 read_data[20];
void master_node_loop()
{
	//read message from uart
	static char read_len = 0;
	static unsigned long last_receve_data_time = millis(); 
	while (Serial.available())
	{
		char inbyte = Serial.read();
		read_data[read_len] = inbyte;
		read_len++;
		if (read_len > 0)
		{
			last_receve_data_time = millis(); //only fresh when receiving data from uart
		}
		if (read_len == 10) 
		{
			if ((read_data[0] == 0xbb) && (read_data[1] == 0xaa))// big / little end problem
			{
				REQUEST_MSG msg_;
				memcpy(&msg_, read_data, read_len);
				if (MASTER_ADDRESS == msg_.addr)  //only master address
				{
					parse_command(msg_);
				}
				else  //other node then transpond that command
				{
					bool res = nrf_write_command(msg_);  
					send_ack(msg_.addr, msg_.func, res);
					//TODO  need to wait slave answer the command
				}				
				read_len = 0;
			}
		}
	}

	if ((read_len > 0) && (millis() - last_receve_data_time > 500)) // > 500ms then receive message timeout , need to set read_len to 0
	{
		read_len = 0;
	}
#if 0
	//wait the slave send ack message
	if (false == wait_ack_flag)
	{
		value = (value == 0 ? 1 : 0);
		//test master send mesage
		Serial.println("master start send");
		Serial.print("wait_ack_flag: ");
		Serial.println(wait_ack_flag);
		
		bool res = send_command(0x01, FUNC_CODE_OPERATE, value);
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
			wait_ack_flag = 0;
		}
	}
#endif
}

void slave_node_loop()
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


char val = 0;
void loop()
{
    //BH1750_task();
	network.update();
	if (NODE_MASTER == node_mode)
	{
		master_node_loop();
	}
	else if (NODE_SLAVE == node_mode)
	{
		slave_node_loop();
	}
	//val = (val == 1 ? 0 : 1);
	//send_command(0x01, FUNC_CODE_OPERATE, val);
	delay(500);
	wdt_reset();
}

