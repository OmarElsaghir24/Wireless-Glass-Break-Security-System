/*	NRF24L01.h
 *
 *	CSE 4352 - IOT
 *  Project 2
 *  Team 11 - Glass Break Sensor
 *  Members:
 *      Omar Elsaghir
 *      John (Reade) Corr
 * 	Created: April 29, 2025
 */


#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#ifndef NRF24L01_H_
#define NRF24L01_H_


/*************************************************************************************************/
/**************************** INCLUDES, DEFINES, ASSEMBLER DIRECTIVES ****************************/

typedef struct _nrf24lo1
{
	uint8_t checksum;
	uint8_t frame_id;
	uint8_t type;
	uint8_t dataLength;
	uint8_t data[28];
} nrf24lo1;

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define RX_ADDR_P0  0x0A
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define FIFO_STATUS 0x17
#define DYNPD       0x1C
#define FEATURE     0x1D

#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define NOP           0xFF

#define MSG_RESET           0x00
#define MSG_DISCONNECT      0x01
#define MSG_JOIN_REQUEST    0x02
#define MSG_JOIN_RESPONSE   0x03
#define MSG_SYNC            0x04
#define MSG_KEEP_ALIVE      0x05
/*************************************************************************************************/
/***************************************** SUB-ROUTINES ******************************************/

uint8_t readRegister(uint8_t reg);
void writeRegisterMulti(uint8_t reg, uint8_t* data, uint8_t len);
void writeRegister(uint8_t reg, uint8_t value);
void writePayload(uint8_t* data, uint8_t len);
uint8_t crc8_ccitt(const uint8_t *data, uint8_t length);
void readPayload(uint8_t* buffer, uint8_t len);
void flushRx(void);
void initNrf24l01Receiver(void);
void enableReceiver(void);
void nrf24_init(void);
void enableTransmitter(void);
void flushTx(void);
void sendPacket(nrf24lo1* pkt);
uint8_t computeChecksum(nrf24lo1* pkt);
void serverJoin(void);

#endif /* NRF24L01_H_ */
