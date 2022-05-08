
#include "NRF24L01.h"
#include "wiring_private.h"

#define SOFT_SPI_MISO_PIN 16
#define SOFT_SPI_MOSI_PIN 17
#define SOFT_SPI_SCK_PIN 3
#define MY_CE_PIN 5
#define MY_CS_PIN 2
#define IRQ     6

#define  READREGISTER        	0x03
#define  PRESETREGISTER     	0x10
#define  PRESETREGISTER_one  0x06
#define CHECK_COM_DELAY_TIMER 10
#define TRUE     1
#define FALSE    0


typedef struct
{
	bool flag;
	U32 previous_time;
	U32 now_time;
}CHECK_FLAG;



void delay_loc(int i)
{
	int m=50;
	while(i--)
	{
		while(m--)
			asm("nop");
	}
}
//***************************************************
#define TX_ADR_WIDTH    5   // 5 unsigned chars TX(RX) address width
#define TX_PLOAD_WIDTH  32  // 32 unsigned chars TX payload

unsigned char TX_ADDRESS[TX_ADR_WIDTH]  = 
{
  0x34,0x43,0x10,0x10,0x01
}; // Define a static TX address

unsigned char rx_buf[TX_PLOAD_WIDTH];
unsigned char tx_buf[TX_PLOAD_WIDTH];
//***************************************************
void setup() 
{
  pinMode(MY_CE_PIN,  OUTPUT);
  pinMode(SOFT_SPI_SCK_PIN, OUTPUT);
  pinMode(MY_CS_PIN, OUTPUT);
  pinMode(SOFT_SPI_MOSI_PIN,  OUTPUT);
  pinMode(SOFT_SPI_MISO_PIN, INPUT);
  pinMode(IRQ, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //  attachInterrupt(1, _ISR, LOW); // interrupt enable
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  Serial.println("LED blink");
  Serial.println(String(LED_BUILTIN));
  //init_io();                        // Initialize IO port
  digitalWrite(MY_CE_PIN, 0);      // chip enable
  digitalWrite(MY_CS_PIN, 1);                 // Spi disable  
  delay(1000);
//  RX_Mode();                        // set RX mode
//   TX_Mode();                       // set TX mode
}
#define TRANST
char check_test=3;
void loop() 
{
	int k = 0;

	int i;
#ifdef REV
	for(;;)
	{
		RX_Mode();  
		unsigned char status = SPI_Read(STATUS);                         // read register STATUS's value
		if(status&RX_DR)                                                 // if receive data ready (TX_DS) interrupt
		{
			for(int i=0; i<32; i++)
			{
				rx_buf[i]=0;                              // print rx_buf
			}
			SPI_Read_Buf(RD_RX_PLOAD, rx_buf, TX_PLOAD_WIDTH);             // read playload to rx_buf
			SPI_RW_Reg(FLUSH_RX,0);                                        // clear RX_FIFO
 			for(int i=0; i<32; i++)
 			{
 				Serial.print(String(rx_buf[i]));                              // print rx_buf
 			}
			if (rx_buf[10]==4)
			{
				digitalWrite(LED,!digitalRead(LED));
			}
			
		}
		SPI_RW_Reg(WRITE_REG+STATUS,status);                             // clear RX_DR or TX_DS or MAX_RT interrupt flag
		delay_loc(10);
	}
#endif


#ifdef TRANST
 
	for(;;)
	{
		
		if (check_test==3)
		{
			check_test=4;
		}else if (check_test==4)
		{
			check_test=3;
		}
		for( i=0; i<32; i++)
    {
       tx_buf[i] = check_test; 
    }
      TX_Mode();                       // set TX mode 
       //digitalWrite(MY_CE_PIN, LOW);
        //SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
        //digitalWrite(MY_CE_PIN, HIGH);
        delay(100);
         while(digitalRead(IRQ))
        {
          delay(100);       
        }  

		U08 status = SPI_Read(STATUS);                   // read register STATUS's value
        Serial.print("master status: ");
        Serial.println(status, HEX);
        
    U08 statusF = SPI_Read(FIFO_STATUS);                   // read register STATUS's value
        Serial.print("master FIFO_STATUS: ");
        Serial.println(statusF, HEX);        
		if(status&TX_DS)                                    // if receive data ready (TX_DS) interrupt
		{
      Serial.println("success send");
			//SPI_RW_Reg(FLUSH_TX,0); 
			//SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);       // write playload to TX_FIFO
		}
		if(status&MAX_RT)                                        // if receive data ready (MAX_RT) interrupt, this is retransmit than  SETUP_RETR                          
		{
			SPI_RW_Reg(FLUSH_TX,0xFF);
      Serial.println("up to retry times");
			//SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);      // disable standy-mode
		}
		SPI_RW_Reg(WRITE_REG+STATUS,status);                     // clear RX_DR or TX_DS or MAX_RT interrupt flag
    digitalWrite(IRQ, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    //digitalWrite(4,!digitalRead(4));  
	}
#endif

}

//**************************************************
// Function: init_io();
// Description:
// flash led one time,chip enable(ready to TX or RX Mode),
// Spi disable,Spi clock line init high
//**************************************************
void init_io(void)
{
  digitalWrite(IRQ, 0);
  digitalWrite(MY_CE_PIN, 0);			// chip enable
  digitalWrite(MY_CS_PIN, 1);                 // Spi disable	
}

/**************************************************
 * Function: SPI_RW();
 * 
 * Description:
 * Writes one unsigned char to nRF24L01, and return the unsigned char read
 * from nRF24L01 during write, according to SPI protocol
 **************************************************/
unsigned char SPI_RW(unsigned char Byte)
{
  unsigned char i;
  for(i=0;i<8;i++)                      // output 8-bit
  {
    if(Byte&0x80)
    {
      digitalWrite(SOFT_SPI_MOSI_PIN, 1);    // output 'unsigned char', MSB to SOFT_SPI_MOSI_PIN
    }
    else
    {
      digitalWrite(SOFT_SPI_MOSI_PIN, 0);
    }
    digitalWrite(SOFT_SPI_SCK_PIN, 1);                      // Set SOFT_SPI_SCK_PIN high..
    Byte <<= 1;                         // shift next bit into MSB..
    if(digitalRead(SOFT_SPI_MISO_PIN) == 1)
    {
      Byte |= 1;       	                // capture current SOFT_SPI_MISO_PIN bit
    }
    digitalWrite(SOFT_SPI_SCK_PIN, 0);         	// ..then set SOFT_SPI_SCK_PIN low again
  }
  return(Byte);           	        // return read unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_RW_Reg();
 * 
 * Description:
 * Writes value 'value' to register 'reg'
/**************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
  unsigned char status;

  digitalWrite(MY_CS_PIN, 0);                   // MY_CS_PIN low, init SPI transaction
  status = SPI_RW(reg);                   // select register
  SPI_RW(value);                          // ..and write value to it..
  digitalWrite(MY_CS_PIN, 1);                   // MY_CS_PIN high again

  return(status);                   // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Read();
 * 
 * Description:
 * Read one unsigned char from nRF24L01 register, 'reg'
/**************************************************/
unsigned char SPI_Read(unsigned char reg)
{
  unsigned char reg_val;

  digitalWrite(MY_CS_PIN, 0);           // MY_CS_PIN low, initialize SPI communication...
  SPI_RW(reg);                   // Select register to read from..
  reg_val = SPI_RW(0);           // ..then read register value
  digitalWrite(MY_CS_PIN, 1);          // MY_CS_PIN high, terminate SPI communication

  return(reg_val);               // return register value
}
/**************************************************/

/**************************************************
 * Function: SPI_Read_Buf();
 * 
 * Description:
 * Reads 'unsigned chars' #of unsigned chars from register 'reg'
 * Typically used to read RX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(MY_CS_PIN, 0);                  // Set MY_CS_PIN low, init SPI tranaction
  status = SPI_RW(reg);       	    // Select register to write to and read status unsigned char

  for(i=0;i<bytes;i++)
  {
    pBuf[i] = SPI_RW(0);    // Perform SPI_RW to read unsigned char from nRF24L01
  }

  digitalWrite(MY_CS_PIN, 1);                   // Set MY_CS_PIN high again

  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: SPI_Write_Buf();
 * 
 * Description:
 * Writes contents of buffer '*pBuf' to nRF24L01
 * Typically used to write TX payload, Rx/Tx address
/**************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
  unsigned char status,i;

  digitalWrite(MY_CS_PIN, 0);                   // Set MY_CS_PIN low, init SPI tranaction
  status = SPI_RW(reg);             // Select register to write to and read status unsigned char
  for(i=0;i<bytes; i++)             // then write all unsigned char in buffer(*pBuf)
  {
    SPI_RW(*pBuf++);
  }
  digitalWrite(MY_CS_PIN, 1);                  // Set MY_CS_PIN high again
  return(status);                  // return nRF24L01 status unsigned char
}
/**************************************************/

/**************************************************
 * Function: RX_Mode();
 * 
 * Description:
 * This function initializes one nRF24L01 device to
 * RX Mode, set RX address, writes RX payload width,
 * select RF channel, datarate & LNA HCURR.
 * After init, MY_CE_PIN is toggled high, which means that
 * this device is now ready to receive a datapacket.
/**************************************************/
void RX_Mode(void)
{
  //digitalWrite(MY_CE_PIN, 0);
  SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // Use the same address on the RX device as the TX device
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
  SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); // Select same RX payload width as TX Payload width
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
  SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:RX. RX_DR enabled..
  delay_loc(10);
  digitalWrite(MY_CE_PIN, 1);                             // Set MY_CE_PIN pin high to enable RX device
  delay_loc(10);
  
  //  This device is now ready to receive one packet of 16 unsigned chars payload from a TX device sending to address
  //  '3443101001', with auto acknowledgment, retransmit count of 10, RF channel 40 and datarate = 2Mbps.
}
/**************************************************/
/**/
void TX_Mode(void)
{
  digitalWrite(MY_CE_PIN, 0);
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
  SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x0a); // 500us + 86us, 10 retrans...
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
	digitalWrite(MY_CE_PIN, 1);
}

void TX_Mode1(void)

{
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
	SPI_Write_Buf(WR_TX_PLOAD,tx_buf,TX_PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...
	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:2Mbps, LNA:HCURR
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     // Set PWR_UP bit, enable CRC(2 unsigned chars) & Prim:TX. MAX_RT & TX_DS enabled..
	delay_loc(10);
	digitalWrite(MY_CE_PIN, 1);
  delay_loc(10);
}
