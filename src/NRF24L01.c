/*	NRF24L01.c
 *
 *	CSE 4352 - IOT
 *  Project 2
 *  Team 11 - Glass Break Sensor
 *  Members:
 *      Omar Elsaghir
 *      John (Reade) Corr
 * 	Created: April 29, 2025
 */


/*************************************************************************************************/
/**************************** INCLUDES, DEFINES, ASSEMBLER DIRECTIVES ****************************/

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "gpio.h"
#include "uart0.h"
#include "spi0.h"
#include "wait.h"
#include "battery_level.h"
#include "glass_break_main.h"
#include "NRF24L01.h"
#include "tm4c123gh6pm.h"


uint8_t address[5] = {0xA0, 0xB0, 0xA0, 0xB0, 0xA0};
nrf24lo1 sndPacket;


/*************************************************************************************************/
/***************************************** SUB-ROUTINES ******************************************/

uint8_t readRegister(uint8_t reg)
{
    uint8_t result;

    setPinValue(CS, 0); 	// Begin SPI transaction

    writeSpi0Data(R_REGISTER | (reg & 0x1F)); 	// Send register read command
    while (SSI0_SR_R & SSI_SR_BSY);          	// Wait for SPI
    readSpi0Data();                          	// Dummy read (status)

    writeSpi0Data(NOP);                      	// Send NOP to receive register value
    while (SSI0_SR_R & SSI_SR_BSY);          	// Wait for SPI
    result = readSpi0Data();                 	// Actual register value

    setPinValue(CS, 1);   	// End SPI transaction

    return result;
}

void writeRegisterMulti(uint8_t reg, uint8_t* data, uint8_t len)
{
	uint8_t i;
    setPinValue(CS, 0);
    writeSpi0Data(W_REGISTER | (reg & 0x1F));
    for (i = 0; i < len; i++)
        writeSpi0Data(data[i]);
    setPinValue(CS, 1);
}

void writeRegister(uint8_t reg, uint8_t value)
{
    setPinValue(CS, 0);
    writeSpi0Data(W_REGISTER | (reg & 0x1F));
    writeSpi0Data(value);
    setPinValue(CS, 1);
}

void writePayload(uint8_t* data, uint8_t len)
{
	int i;
    setPinValue(CS, 0);
    writeSpi0Data(W_TX_PAYLOAD);
    for (i = 0; i < len; i++)
        writeSpi0Data(data[i]);
    setPinValue(CS, 1);
}

uint8_t crc8_ccitt(const uint8_t *data, uint8_t length)
{
	uint8_t i;
	uint8_t j;
    uint8_t crc = 0x00;
    for (i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    //return crc;
    return 0xAA;
}

void readPayload(uint8_t* buffer, uint8_t len)
{
	int i;
    setPinValue(CS, 0);
    writeSpi0Data(0x61);  // R_RX_PAYLOAD
    while (SSI0_SR_R & SSI_SR_BSY);
    for (i = 0; i < len; i++)
    {
        writeSpi0Data(0);
        while (SSI0_SR_R & SSI_SR_BSY);
        buffer[i] = readSpi0Data();
    }
    setPinValue(CS, 1);
}

void flushRx(void)
{
    setPinValue(CS, 0);
    writeSpi0Data(0xE2);  // FLUSH_TX command
    setPinValue(CS, 1);
}

void initNrf24l01Receiver(void)
{
    writeRegisterMulti(TX_ADDR, address, 5);    // Set TX address
	writeRegisterMulti(RX_ADDR_P0, address, 5); // Set RX address
    setPinValue(CE, 0); // Disable while configuring

    writeRegister(CONFIG, 0x03);       // PRIM_RX=1, PWR_UP=1, CRC=1 byte
    writeRegister(EN_AA, 0x00);        // Enable auto-ack on pipe 0
    writeRegister(EN_RXADDR, 0x01);    // Enable data pipe 0
    //writeRegister(SETUP_RETR, 0x3F);   // 15 retries, 1250us delay
    writeRegister(SETUP_RETR, 0x00);   // 15 retries, 1250us delay
    writeRegister(RF_CH, 10);          // Same channel as TX
    writeRegister(RF_SETUP, 0x06);     // 1 Mbps, 0 dBm
    //writeRegister(STATUS, 0x70);       // Clear all IRQ flags
    writeRegister(STATUS, 0x70);       // Clear all IRQ flags
    writeRegister(DYNPD, 0x00);           // Disable dynamic payload
    writeRegister(FEATURE, 0x00);         // Disable features
    writeRegister(RX_PW_P0, 32);       // 32-byte fixed payload size

    //writeRegisterMulti(TX_ADDR, address, 5);

    //flushRx();                         // Clean RX FIFO
    setPinValue(CE, 1);                // Enable RX mode (CE high)
    waitMicrosecond(2000);  // startup delay
}

void enableReceiver(void)
{
	setPinValue(CE, 0);
    writeRegister(CONFIG, 0x03);   // PWR_UP + PRIM_RX=1 => RX mode
    writeRegister(STATUS, 0x70);
    waitMicrosecond(150);         // ~130us startup for RX mode
    setPinValue(CE, 1);           // CE high to start listening
}

void nrf24_init(void)
{
	writeRegisterMulti(TX_ADDR, address, 5);
	writeRegisterMulti(RX_ADDR_P0, address, 5);
    //setPinValue(CE, 0);
    //writeRegister(CONFIG, 0x0E);        // Power up, CRC on, PRIM_RX=0 (TX)
    writeRegister(CONFIG, 0x00);
    //writeRegister(EN_AA, 0x01);         // Auto ack on pipe 0
    writeRegister(EN_AA, 0x00);         // Auto ack on pipe 0
    writeRegister(EN_RXADDR, 0x01);     // Enable data pipe 0
    //writeRegister(SETUP_RETR, 0x2F);    // 750us delay, 15 retries
    writeRegister(SETUP_RETR, 0x00);
    writeRegister(RF_CH, 10);           // Channel 10
    writeRegister(RF_SETUP, 0x06);      // 1 Mbps, 0 dBm
    //writeRegister(RF_SETUP, 0x27);
    writeRegister(STATUS, 0x70);        // Clear IRQs
    //writeRegister(STATUS, 0x40);        // Clear IRQs
    writeRegister(RX_PW_P0, 32);        // Fixed payload size
    writeRegister(DYNPD, 0x00);  // Disable dynamic payloads
    writeRegister(FEATURE, 0x00); // Disable features (like ACKs) just in case

    // Set TX and RX address for pipe 0
	//writeRegisterMulti(TX_ADDR, address, 5);
	//writeRegisterMulti(RX_ADDR_P0, address, 5);

    waitMicrosecond(1500);
    //setPinValue(CE, 1);
}

void enableTransmitter(void)
{
    setPinValue(CE, 0);
    writeRegister(CONFIG, 0x0E);  // PWR_UP + PRIM_RX=0
    //writeRegister(STATUS, 0x40);  // Clear IRQs
    waitMicrosecond(150);
}

void flushTx(void)
{
    setPinValue(CS, 0);
    writeSpi0Data(0xE1);  // FLUSH_TX command
    setPinValue(CS, 1);
}

void sendPacket(nrf24lo1* pkt)
{
    uint8_t i;

	pkt->data[28-2] = 0; // reserved byte 0
	pkt->data[28-1] = 0; // reserved byte 1

	// Set checksum field temporarily to 0
	pkt->checksum = 0;

	// Calculate CRC over entire 32 bytes
	pkt->checksum = crc8_ccitt((uint8_t*)pkt, 32);
	printHex8(pkt->checksum);
    writeRegister(STATUS, 0x70);  // Clear TX_DS, MAX_RT, RX_DR

    flushTx(); // Always flush before send

    writeRegister(CONFIG, 0x0E);        //TODO: from Sleep Code
	waitMicrosecond(2000);      // Wait for power-up

    setPinValue(CE, 0);
    writePayload((uint8_t*)pkt, 32);
    setPinValue(CE, 1);
    waitMicrosecond(15);
    setPinValue(CE, 0);          // Must go low after TX
    waitMicrosecond(130);        // Allow some time for TX to complete

    putsUart0("Frame ID: ");
	printHex8(pkt->frame_id);
	putsUart0("\n");
	putsUart0("Type: ");
	printHex8(pkt->type);
	putsUart0("\n");
	putsUart0("Data Length: ");
	printHex8(pkt->dataLength);
	putsUart0("\n");
	putsUart0("Data: ");
	for(i = 0; i < pkt->dataLength; i++) {
		printHex8(pkt->data[i]);
	}
	putsUart0("\n");
}

uint8_t computeChecksum(nrf24lo1* pkt)
{
	uint8_t i;
    uint8_t checksum = 0;
    checksum += pkt->type;
    checksum += pkt->dataLength;
    for (i = 0; i < pkt->dataLength; i++)
        checksum += pkt->data[i];
    return checksum;
}


void serverJoin(void)
{
	int i;

    while (1) {

        enableReceiver();
        BLUE_LED = 1;       // indicates waiting for host message

        while (!(readRegister(STATUS) & (1 << 6))); // Wait for RX_DR flag to be set
        writeRegister(STATUS, (1 << 6));            // Clear RX_DR flag
        BLUE_LED = 0;       // indicates host message received

        // Read in packet and store to packet struct
        uint8_t rxBuf[32];
        readPayload(rxBuf, 32);
        nrf24lo1* pkt = (nrf24lo1*)(rxBuf+7);

        //enableTransmitter();

        // Print received packet details
        putsUart0("Received Packet:\n");
        putsUart0("Frame ID: ");
        printHex8(pkt->frame_id);
        putsUart0("\nType: ");
        printHex8(pkt->type);
        putsUart0("\nData Length: ");
        printHex8(pkt->dataLength);
        putsUart0("\n");
        for (i = 0; i < pkt->dataLength; i++) {
            putsUart0("Data[");
            printHex8(i);
            putsUart0("]: ");
            printHex8(pkt->data[i]);
            putsUart0("\n");
        }

        //enableTransmitter();

        // Received SYNC packet from Wireless Hub
        if (pkt->type == 0x04) {
            enableTransmitter();
            putsUart0("SYNC packet received!\n");
            sndPacket.frame_id = 0x00;
            sndPacket.type = 0x02;
            sndPacket.dataLength = 0x01;
            sndPacket.data[0] = 0x81;
            sndPacket.checksum =  crc8_ccitt((uint8_t*)&sndPacket, 32);
            sendPacket(&sndPacket);
            putsUart0("Sending JOIN_REQUEST\n");
        }
        // else {
        //     putsUart0("Waiting for SYNC packet\n");
        // }

        // waitMicrosecond(2000000);
        // enableReceiver();

        // Join Response Received
        else if (pkt->type == 0x03) {
            GREEN_LED = 1;

            // Print JOIN_RESPONSE packet details
            putsUart0("JOIN_RESPONSE received!\n");
            putsUart0("Frame ID: ");
            printHex8(pkt->frame_id);
            putsUart0("\nType: ");
            printHex8(pkt->type);
            putsUart0("\nData Length: ");
            printHex8(pkt->dataLength);
            putsUart0("\n");
            for (i = 0; i < pkt->dataLength; i++) {
                putsUart0("Data[");
                printHex8(i);
                putsUart0("]: ");
                printHex8(pkt->data[i]);
                putsUart0("\n");
            }

            // Sample Battery Level
            uint8_t battery_level = check_bat_level();       // will sample for 2 seconds

            // Send Battery Level Message
            enableTransmitter();
            //writeRegister(STATUS, (1 << 6)); // Clear RX_DR flag
            sndPacket.frame_id = pkt->frame_id;
            sndPacket.type = 0x0B;
            sndPacket.dataLength = 0x01;
            sndPacket.data[0] = battery_level;
            sndPacket.checksum =  crc8_ccitt((uint8_t*)&sndPacket, 32);
            sendPacket(&sndPacket);

            return;
        }
    }
}
