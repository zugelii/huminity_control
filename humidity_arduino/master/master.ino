/*
 * See documentation at https://nRF24.github.io/RF24
 * See License information at root directory of this library
 * Author: Brendan Doherty (2bndy5)
 */

/**
 * A simple example of sending data from 1 nRF24L01 transceiver to another.
 *
 * This example was written to be used on 2 devices acting as "nodes".
 * Use the Serial Monitor to change each node's behavior.
 */
#include <SPI.h>
#include "printf.h"
#include <RF24.h>
#include <RF24Network.h>

#define MY_CE_PIN       5
#define MY_CS_PIN       2
#define MY_RELAY_PIN    4
// instantiate an object for the nRF24L01 transceiver
RF24 radio(MY_CE_PIN, MY_CS_PIN); // using pin 5 for the CE pin, and pin 2 for the CSN pin

RF24Network network(radio);       // Network uses that radio
const uint16_t this_node = 00;    // Address of our node in Octal format ( 04,031, etc)
const uint16_t other_node = 02;   // Address of the other node in Octal format

const unsigned long interval = 1000;  //ms  // How often to send 'hello world to the other unit

unsigned long last_sent;             // When did we last send?

/**** Create a large array for data to be received ****
* MAX_PAYLOAD_SIZE is defined in RF24Network_config.h
* Payload sizes of ~1-2 KBytes or more are practical when radio conditions are good
*/
uint8_t dataBuffer[MAX_PAYLOAD_SIZE]; //MAX_PAYLOAD_SIZE is defined in RF24Network_config.h
void setup() 
{
    pinMode(MY_RELAY_PIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial) 
    {
    // some boards need to wait to ensure access to serial over USB
    }

    // initialize the transceiver on the SPI bus
    if (!radio.begin()) 
    {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {} // hold in infinite loop
    }

    // print example's introductory prompt
    Serial.println(F("master RF24/examples/GettingStarted"));
    radio.setChannel(90);
    network.begin(/*node address*/ this_node);

    // For debugging info
    printf_begin();             // needed only once for printing details
    radio.printPrettyDetails(); // (larger) function that prints human readable data

    // Load our data buffer with numbered data
    for (uint16_t i = 0; i < MAX_PAYLOAD_SIZE; i++) 
    {
        dataBuffer[i] = i % 256; //Ensure the max value is 255
    }
} // setup

char flag = HIGH;

uint16_t sizeofSend = 25; //Variable to indicate how much data to send
bool stopSending = 0;    //Used to stop/start sending of data
uint8_t dataBuffers[MAX_PAYLOAD_SIZE]; //MAX_PAYLOAD_SIZE is defined in RF24Network_config.h
uint32_t timeBetweenPackets = 0;
void loop() 
{
    //User input anything via Serial to stop/start data transmission
    if (Serial.available()) 
    {
        Serial.read();
        stopSending = !stopSending;
    }
    network.update();                          // Check the network regularly
    unsigned long now = millis();              // If it's time to send a message, send it!
    if ( now - last_sent >= interval && !stopSending ) 
    {
        last_sent = now;
        Serial.print(F("Sending size "));
        Serial.print(sizeofSend);

        // Fragmentation/reassembly is transparent. Just send payloads as usual.
        RF24NetworkHeader header(/*to node*/ other_node);
        bool ok = network.write(header, &dataBuffer, sizeofSend++);

        // If the size of data to be sent is larger than max payload size, reset at 0
        if (sizeofSend  > MAX_PAYLOAD_SIZE) 
        {
            sizeofSend  = 0;
        }

        Serial.println(ok ? F(" ok.") : F(" failed."));
        //digitalWrite(MY_RELAY_PIN, flag);
        //flag = flag == HIGH ? LOW : HIGH;    
    }
    //read data from slave
    RF24NetworkHeader headers;                       // If so, grab it and print it out
    uint16_t payloadSize = network.peek(headers);    // Use peek() to get the size of the payload
    if(payloadSize > 0)
    {
        network.read(headers, &dataBuffers, payloadSize); // Get the data
        Serial.print("Received packet, size ");         // Print info about received data
        Serial.print(payloadSize);
        Serial.print("(");
        Serial.print(millis() - timeBetweenPackets);
        Serial.println("ms since last)");
        timeBetweenPackets = millis();
        // Uncomment below to print the entire payload  
        for(uint32_t i = 0; i < payloadSize; i++) 
        {
            Serial.print(dataBuffers[i]);
            Serial.print(F(": "));
            if(i % 50 == 49) 
            {
                //Add a line break every 50 characters
                Serial.println();
            }
        }
        Serial.println();
    }
} // loop
