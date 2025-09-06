/*	glass_break_main.h
 *
 *	CSE 4352 - IOT
 *  Project 2
 *  Team 11 - Glass Break Sensor
 *  Members:
 *      Omar Elsaghir
 *      John (Reade) Corr
 * 	Submitted: April 29, 2025
 */


#ifndef GLASS_BREAK_MAIN_H_
#define GLASS_BREAK_MAIN_H_


/*************************************************************************************************/
/**************************** INCLUDES, DEFINES, ASSEMBLER DIRECTIVES ****************************/

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

#define RED_LED           (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED          (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PIEZOELECTRIC     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define SW2               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))
#define SW1               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define SW2_MASK 1
#define SW1_MASK 16
#define PIEZOELECTRIC_MASK 16
#define IRQ_MASK 4

#define IRQ          PORTB,2
#define CE           PORTA,7
#define CS           PORTA,3
#define BAT_VOLT     PORTE,0


#endif
