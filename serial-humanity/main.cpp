/*
 * @Description: 
 * @Author: 
 * @Github: 
 * @Date: 2020-04-24 16:03:35
 * @LastEditors: gabe
 * @LastEditTime: 2022-07-02 11:41:36
 * @FilePath: /ctestcase/tutorial.cpp
 */
// A simple program that computes the square root of a number
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "TutorialConfig.h"
// #ifdef USE_MYMATH
// #include "MathFunctions.h"
// #endif
#include <vector>
#include <csignal>
#include <condition_variable> // std::condition_variable, std::cv_status
#include <functional>
#include <chrono>
#include <iostream>
#include <time.h>
#include "cserial.h"


#define HWMONTER_VERSION "1.0.2"

static std::atomic<bool> main_thread_run(true);
std::shared_ptr<CSerialPort> serial_;
std::condition_variable cv;
std::mutex cv_m;

#define DELEY_TIME 4

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


int32_t do_send_msg(uint16_t to_addr, uint8_t func, uint32_t val, uint8_t rev_len)
{
	int32_t read_len;
	uint8_t try_counts = 0;
	REQUEST_MSG msg_;
	bool res;
	msg_.header = HEAD_MSG;
	msg_.addr = to_addr;
	msg_.func = func;
	msg_.data = val;	
	msg_.crc = CRC((U08*)&msg_, sizeof(REQUEST_MSG) - 2);
#ifdef DEBUG_M
	printf("send Len:%d ,send msg:", sizeof(ACK_MSG));
#endif
	try_counts = 0;
	do  //send message try 4 times
	{
		if(serial_->SendMessage((U08*)&msg_, sizeof(msg_)) > 0)
		{
			break; 
		}
	}while(try_counts++ < MAX_TRY_COUNT);

	ACK_MSG rev_msg_;
	memset(&rev_msg_, 0, sizeof(rev_msg_));
	read_len = serial_->ReadMessage((uint8_t*)&rev_msg_, sizeof(rev_msg_));
	
	if(read_len > 0)
	{
		#ifdef DEBUG_M
		printf("readlen:%d\n", read_len);
		printf("0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n", rev_pool[0], rev_pool[1], rev_pool[2], rev_pool[3], rev_pool[4], rev_pool[5], rev_pool[6]);
		#endif
		uint16_t crc = CRC((uint8_t*)&rev_msg_, read_len - 2);
		printf("receive len:%d, header:%X, addr:%X, func:%X, status:%X, crc:%X scrc:%X\r\n", read_len, rev_msg_.header, rev_msg_.addr, rev_msg_.func, rev_msg_.status, rev_msg_.crc, crc);
		if(crc != rev_msg_.crc)
		{
			read_len = -1;
		}		
	}
	return read_len;	
}


void process_serial(const string& port_name, const int32_t& msgid)
{
	int32_t buf_len = 20;
	uint8_t send_pool[buf_len] = {0};
	struct msg_st data;
	int32_t msgtype = 0;
	int32_t error_count = 0;
	uint32_t value = 0;
	int32_t res{-1};
	spdlog::info("process_serial");
	prctl(PR_SET_NAME, "humanity_serial");  

	serial_ =  std::make_shared<CSerialPort>(port_name.c_str(), SERIAL_BAUDRATE);
	uint16_t relay_addr = 0;
	if(serial_->isConnected())
	{
		while(main_thread_run)
		{
			//read time /luminar and so on

			time_t     now = time(0);
			struct tm  tstruct;
			tstruct = *localtime(&now);
			printf("data: %d-%d-%d %d:%d:%d\r\n", tstruct.tm_year, tstruct.tm_mon, tstruct.tm_mday, tstruct.tm_hour, tstruct.tm_min, tstruct.tm_sec);
			if((tstruct.tm_mon >= 5) && (tstruct.tm_mon <= 8))
			{
				if ((tstruct.tm_hour > 10) && ((tstruct.tm_hour < 14)))
				{
					//open fans
				}
				else
				{
					//close fans;
				}
                  
			}  

			res = do_send_msg(1, FUNC_CODE_OPERATE, value, 10);
			if (res > 0)
			{
				/* code */
				error_count = 0;
			}
			else
			{
				error_count++;
			}
			value = (value == 0 ? 1 : 0);
			res = do_send_msg(0, FUNC_CODE_OPERATE, value, 10);
			error_count = 0;		
			sleep(10);
		}
	}
	else
	{
		spdlog::error("open serial error");
	}
	main_thread_run = false;
}

void on_signal_int(int signum)
{
	(void)signum;
	main_thread_run = false;
	std::unique_lock <std::mutex> lck(cv_m);
	cv.notify_all(); 	
}

int main (int argc, char *argv[])
{
	int32_t msgid = -1;	
	signal(SIGINT, on_signal_int);
	signal(SIGTERM, on_signal_int);
	Log::Init("humanity");
	spdlog::info(HWMONTER_VERSION);
	string seral_name("/dev/ttyUSB0");
    if (argc >=2)
	{
		if(strlen(argv[1]) > 0)
		{
			seral_name.clear();
			seral_name.append(argv[1]);
			spdlog::info("use manual serial");
		}
	}
	spdlog::info("serial path:" + seral_name);	  
	thread process_ser(process_serial, seral_name, msgid);  //open rs485	

	while (main_thread_run)
	{	
		std::unique_lock<std::mutex> lk(cv_m);
		cv_status st = cv.wait_for(lk, std::chrono::seconds(2));
		if(st == cv_status::timeout)
		{
			;
		}
		else
		{
			break;
		}				
	}	
}