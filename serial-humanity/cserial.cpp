/*
 * @Author: gabe
 * @Date: 2022-04-26 23:08:21
 * @LastEditors: gabe
 * @LastEditTime: 2022-07-02 10:52:49
 * @Description: 
 */
#include "cserial.h"
#include <iostream>
using namespace std;

int32_t CSerialPort::SetParity(int32_t fd, int32_t databits, int32_t stopbits, int8_t parity, int32_t speed)
{
    struct termios options;
    if  ( tcgetattr( fd, &options)  !=  0)
    {
        spdlog::error("SetupSerial error");
        return (-1);
    }
    bzero( &options, sizeof(options));
    options.c_cflag |=  CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;

	switch(speed)
	{
	case 9600:
	    cfsetispeed(&options, B9600);
	    cfsetospeed(&options, B9600);
	    break;
	case 115200:
	    cfsetispeed(&options, B115200);
	    cfsetospeed(&options, B115200);
	    break;
	case 460800:
	    cfsetispeed(&options, B460800);
	    cfsetospeed(&options, B460800);
	    break;
	default:
	    cfsetispeed(&options, B9600);
	    cfsetospeed(&options, B9600);
	    break;
	}

    switch (databits)
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        spdlog::error("Unsupported data size");
        return (-1);
    }

    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB;   /* Clear parity enable */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;     /* Enable parity */
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's':  /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        spdlog::error("Unsupported parity");
        return (-1);
    }

    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        spdlog::error("Unsupported stop bits");
        return (-1);
    }

    options.c_cc[VTIME] = 10; // 1 seconds
    options.c_cc[VMIN] = 0;

    tcflush(fd, TCIFLUSH); /* Update the options and do it NOW */
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        spdlog::error("Update the options");
        return (-1);
    }
    return (1);
}


int32_t CSerialPort::UartEpoll(int32_t &efd, int32_t &sfd)
{
    struct epoll_event event;
    efd = epoll_create(10);
    if(efd < 0)
    {
        spdlog::error("uart create epoll error");
        return -1;
    }
    event.data.fd = sfd;
    event.events = EPOLLIN;
    if(epoll_ctl(efd, EPOLL_CTL_ADD, sfd, &event) < 0)
    {
        spdlog::error("epoll add event error");
        return -1;
    }
    return 0;

}
CSerialPort::CSerialPort(const char *PortName,
                         int32_t BaudRate,
                         int32_t ByteSize,
                         int32_t StopBits,
                         int8_t Parity)
{
    if(NULL == PortName)
    {
        spdlog::error("please appoint com port");
        exit(1);
    }
    if(SetUart(serial_fd, PortName, BaudRate, ByteSize, StopBits, Parity) >= 0)
    {
        UartEpoll(epoll_fd, serial_fd);
    }
    else
    {
        spdlog::error("set Serial Port error!\n");
    }
}

int32_t CSerialPort::SetUart(int &fd, const char *dev, int32_t BaudRate, int32_t ByteSize, int32_t StopBits, int8_t Parity)
{
    int32_t res = 0;
    fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);
    if (fd > 0)
    {
        res = SetParity(fd, ByteSize, StopBits, Parity, BaudRate);
        if ( res == -1)
        {
            spdlog::error("Set Parity Error\n");
        }    
    }
    else
    {
        spdlog::error("Can't Open Serial Port!\n");
        res = -1;
    }
    return res;
}
int32_t CSerialPort::SendMessage(string &buffer, uint32_t buf_size)
{
     int32_t wr_len;
     int32_t ret;
     ret = flock(serial_fd, LOCK_EX);
     if(ret < 0)
     {
        spdlog::error("lock serial port error");
        return -1;
     }
     wr_len = write(serial_fd, buffer.c_str(), buf_size);
     ret = flock(serial_fd, LOCK_UN);
     if(ret < 0)
     {
         spdlog::error("unlock serial port error");
     }
     return wr_len;
}

int32_t CSerialPort::SendMessage(uint8_t *buffer, uint32_t buf_size)
{
    int32_t wr_len = -1;
    int32_t ret;
    ret = flock(serial_fd, LOCK_EX);
    if(ret < 0)
    {
        spdlog::error("lock serial port error");
        return -1;
    }
    #ifdef DEBUG_M
    for(int i =0; i < buf_size; i++)
    {
        printf("%X ", buffer[i]);
    }
    
    cout << endl;
    #endif
    wr_len = write(serial_fd, buffer, buf_size);
    ret = flock(serial_fd, LOCK_UN);
    if(ret < 0)
    {
        spdlog::error("unlock serial port error");
    }
    return wr_len;
}

int32_t CSerialPort::ReadMessage(string &buffer, uint32_t buf_size)
{
    struct epoll_event wait_event;
    int32_t epoll_ret = -1;
    int32_t read_len = 0, ret = -1;
    epoll_ret = epoll_wait(epoll_fd, &wait_event, EPOLL_WAITE_MAX, EPOLL_WAITE_TIME);
    if(epoll_ret < 0)
    {
        spdlog::error("uart epoll wait fault");
    }
    else if(epoll_ret > 0)
    {
        if((wait_event.data.fd == serial_fd) && (EPOLLIN == (wait_event.events & EPOLLIN)))
        {
            do{
                ret = read(serial_fd, &buffer[read_len], buf_size);
                if(ret > 0)
                {
                    read_len += ret;
                }
            }while(ret > 0);
        }
    }
    else {
        spdlog::error("serial read:epoll wait time out");
    }
    return read_len;
}

int32_t CSerialPort::ReadMessage(uint8_t *buffer, uint32_t buf_size)
{
    struct epoll_event wait_event;
    int32_t epoll_ret = -1;
    int32_t read_len = 0, ret = -1;
    int32_t timeout = 0;

    do
    {
        epoll_ret = epoll_wait(epoll_fd, &wait_event, EPOLL_WAITE_MAX, EPOLL_WAITE_TIME);
        if(epoll_ret < 0)
        {
            spdlog::error("uart epoll wait fault");
        }
        else if(epoll_ret > 0)
        {
            if((wait_event.data.fd == serial_fd) && (EPOLLIN == (wait_event.events & EPOLLIN)))
            {
                ret = read(serial_fd,&buffer[read_len], buf_size);
                if(ret > 0)
                {
                    read_len += ret;
                }
            }
        }
        else 
        {
            if(++timeout > 5);
            {
            	spdlog::error("serial read: epoll wait time out");
                return -1;
            }

        }
    } while (read_len < buf_size);    
   return read_len;
}



int32_t CSerialPort::GetSerialfd() const
{
    return serial_fd;
}
int32_t CSerialPort::GetEpollfd() const
{
    return epoll_fd;
}

void CSerialPort::CloseSerial()
{
    if(serial_fd > 0)
    {
        close(serial_fd);
    }
    if(epoll_fd > 0)
    {
        close(epoll_fd);
    }
   spdlog::info("closeSerial");

}

bool CSerialPort::isConnected()
{
    return (serial_fd > 0);
}

CSerialPort::~CSerialPort()
{
    CloseSerial();
}


// int32_t serial_msg(int32_t &msgid, uint8_t addr, uint16_t t)
// {
//     struct msg_st data;
//     uint16_t CRC16;
//     uint8_t oper_msg[] = {0x01, 0x10, 0x00, 0x03, 0x00, 0x02, 0x04, 0x00, 0x04, 0x00, 0x32, 0x73, 0xAE};
//     data.msg_type = 1;
// 	data.text[0] = sizeof(oper_msg);
// 	data.text[1] = 8;

// 	if((addr == 3) || (addr == 8) || (addr == 13) || (addr = 18))
// 	{
// 		oper_msg[3] = addr;
// 	}
//     oper_msg[9]  = (t >> 8) & 0XFF;
// 	oper_msg[10] =        t & 0XFF;
	
// 	CRC16 = usMBCRC16(oper_msg, 11);
// 	oper_msg[11] = CRC16 & 0xFF;
// 	oper_msg[12] = (CRC16 >> 8) & 0xFF;
// 	memcpy(&data.text[2], oper_msg, sizeof(oper_msg));
// 	if(msgsnd(msgid, (void*)&data, MAX_TEXT, 0) == -1)
// 	{
// 		spdlog::error("serial_msg msgsnd failed");
//         return -1;			
// 	}
// 	#ifdef DEBUG_M
// 	printf("serial_msg  send msg:\n");
//     printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \n", oper_msg[0], oper_msg[1], oper_msg[2], oper_msg[3], oper_msg[4], oper_msg[5], oper_msg[6], oper_msg[7], oper_msg[8], oper_msg[9], oper_msg[10], oper_msg[11], oper_msg[12]);
	
//     #endif      	
//     return 0;
      
// }

