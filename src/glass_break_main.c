/*	glass_break_main.c
 *
 *	CSE 4352 - IOT
 *  Project 2
 *  Team 11 - Glass Break Sensor
 *  Members:
 *      Omar Elsaghir
 *      John (Reade) Corr
 * 	Submitted: April 29, 2025
 */


#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "spi0.h"
#include "wait.h"
#include "adc0.h"
#include "wd0.h"
#include "battery_level.h"
#include "glass_break_main.h"
#include "NRF24L01.h"


char string[100];
uint32_t HOUR, MINUTE, realTime;
bool alarmTrig = false;
nrf24lo1 packet;
uint16_t volt_raw;
uint16_t x[16];
uint16_t volt_inst, fir_volt, iir_volt = 0;
uint8_t index = 0;
uint16_t sum = 0; // total fits in 16b since 12b adc output x 16 samples


/*************************************************************************************************/
/***************************************** SUB-ROUTINES ******************************************/

// Initialize Hardware
void initHw(void)
{
	// Initialize system clock to 40 MHz
	initSystemClockTo40Mhz();

	// Enable clocks
	enablePort(PORTA);
	enablePort(PORTB);
    enablePort(PORTE);
	enablePort(PORTF);
	_delay_cycles(3);

	// Enable Hibernation Module
    SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;
	_delay_cycles(3);

	// Setup LED's
	GPIO_PORTF_DIR_R |= (RED_LED_MASK | BLUE_LED_MASK| GREEN_LED_MASK); // Set as output
    GPIO_PORTF_DEN_R |= (RED_LED_MASK | BLUE_LED_MASK| GREEN_LED_MASK); // Digitally enable

    selectPinPushPullOutput(CS);
    selectPinPushPullOutput(CE);
	// selectPinPushPullOutput(IRQ);
	setPinValue(CE, 0);

    // Set IRQ pin as input with pull-up resistor
    GPIO_PORTB_DIR_R &= ~IRQ_MASK;     // Set IRQ as input
	GPIO_PORTB_DEN_R |= IRQ_MASK;

	GPIO_PORTB_IS_R &= ~IRQ_MASK;     // Edge-sensitive
	GPIO_PORTB_IBE_R &= ~IRQ_MASK;    // Not both edges
	GPIO_PORTB_IEV_R &= ~IRQ_MASK;    // Falling edge

	GPIO_PORTB_ICR_R = IRQ_MASK;      // Clear flag
	GPIO_PORTB_IM_R |= IRQ_MASK;      // Unmask PB2

	NVIC_EN0_R |= (1 << (INT_GPIOB - 16));

    // Configure PF4 (SW1) as input with pull-up resistor
    GPIO_PORTF_DIR_R &= ~SW1_MASK;       // Set PF4 as input
	GPIO_PORTF_PUR_R |= SW1_MASK;        // Enable pull-up resistor on PF4
	GPIO_PORTF_DEN_R |= SW1_MASK;        // Enable digital function on PF4

	// Unlock PF0 (SW2 is on a locked pin)
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;   // Unlock Port F (standard unlock key)
	GPIO_PORTF_CR_R |= SW2_MASK;         // Allow changes to PF0

	// Configure PF0 as an input with a pull-up resistor
	GPIO_PORTF_DIR_R &= ~SW2_MASK;       // Set PF0 as input
	GPIO_PORTF_PUR_R |= SW2_MASK;        // Enable pull-up resistor on PF0
	GPIO_PORTF_DEN_R |= SW2_MASK;        // Enable digital function on PF0

	// Enable interrupt on falling edge (button press)
	GPIO_PORTF_IS_R &= ~SW2_MASK;        // Make PF0 edge-sensitive
	GPIO_PORTF_IBE_R &= ~SW2_MASK;       // Not both edges (only falling edge)
	GPIO_PORTF_IEV_R &= ~SW2_MASK;       // Falling edge trigger
	GPIO_PORTF_ICR_R = SW2_MASK;         // Clear any prior interrupt
	GPIO_PORTF_IM_R |= SW2_MASK;         // Unmask interrupt*/

	// Enable Port F interrupt in NVIC
	NVIC_EN0_R = (1 << (INT_GPIOF - 16)); // Enable interrupt for Port F

	// Configure RTC for Wake Up and Hibernation
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_CTL_R |= HIB_CTL_CLK32EN;            // Enables Hibernation mode clock source
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_CTL_R |= HIB_CTL_PINWEN | HIB_CTL_RTCWEN;        // Enable external wake-up*/
	while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_CTL_R |= HIB_CTL_RTCEN;             // Enables RTC timer (RTC time counting up in seconds)
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_CTL_R |= HIB_CTL_VBATSEL_2_3V | HIB_CTL_VABORT | HIB_CTL_BATWKEN;
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_IM_R |= (HIB_IM_RTCALT0 | HIB_IM_EXTW | HIB_IM_LOWBAT);
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	NVIC_EN1_R = 1 << (INT_HIBERNATE-16-32);
}

// Blinks Green LED for indicated number of times.
void blinkLed(int times)
{
	int i;
    for (i = 0; i < times; i++)
    {
        GREEN_LED = 1;  // Toggle Green LED
        waitMicrosecond(100000);
        GREEN_LED = 0;
        waitMicrosecond(100000);
    }
}

// Tests LED Function:
void test_leds(void)
{
	RED_LED = 1;
	waitMicrosecond(330000);
	RED_LED = 0;
	BLUE_LED = 1;
	waitMicrosecond(330000);
	BLUE_LED = 0;
	GREEN_LED = 1;
	waitMicrosecond(330000);
	GREEN_LED = 0;
}

void disableWatchdogInterrupt()
{
    WATCHDOG0_LOCK_R = WDT_LOCK_UNLOCK;
    WATCHDOG0_CTL_R &= ~WDT_CTL_INTEN;
    WATCHDOG0_LOCK_R = 0;
}

void enableWatchdogInterrupt()
{
    WATCHDOG0_LOCK_R = WDT_LOCK_UNLOCK;
    WATCHDOG0_CTL_R |= WDT_CTL_INTEN;
    WATCHDOG0_LOCK_R = 0;
}

void watchdog_Isr(void)
{
	//resetWatchdog0();
	WATCHDOG0_ICR_R = 0;
	GREEN_LED = 1;
    waitMicrosecond(1000000);
    GREEN_LED = 0;
	/*nrf24lo1 packet;
	packet.frame_id = 1;
	packet.type = 0x04;
	packet.dataLength = 2;
	packet.data[0] = 0x01;
	packet.data[1] = 0x50;*/
	sendPacket(&packet);
	disableWatchdogInterrupt();
	//while(!(HIB_CTL_R & HIB_CTL_WRC));
	//HIB_CTL_R |= HIB_CTL_HIBREQ;
}

void checkBatteryLevel()
{
    // Ensure write complete before modifying HIB_CTL_R
    //while (!(HIB_CTL_R & HIB_CTL_WRC));

    // Trigger a manual battery voltage check
    //HIB_CTL_R |= HIB_CTL_BATCHK;
    //while (!(HIB_CTL_R & HIB_CTL_WRC));

    // Check if the battery voltage is below VBATSEL threshold (VBATSEL is used only if not using default voltage level, which is 2.1 Volts)
    if (HIB_RIS_R & HIB_RIS_LOWBAT) {
        putsUart0("Battery voltage is LOW!\n");
    } else {
        putsUart0("Battery voltage is OK.\n");
    }
}

void setAlarm(void)
{
	// uint32_t nextDay = 86400;
	uint32_t currentTime;
	uint32_t alarmTime, time;
	uint32_t num_seconds;
	// uint32_t seconds;

	while(!(HIB_CTL_R & HIB_CTL_WRC));
	//while((~HIB_CTL_R & HIB_CTL_WRC));
	currentTime = HIB_RTCC_R % 86400;
	uint32_t hh1, mm1, sec1;
	hh1 = currentTime / 3600;
	num_seconds = currentTime % 3600;
	mm1 = (currentTime % 3600) / 60;
	sec1 = num_seconds % 60;
	snprintf(string, sizeof(string), "Time: %d:%02d:%02d\n", hh1, mm1, sec1);
	putsUart0(string);

	//alarmTime = currentTime + 10;

	if(currentTime < 43200) {
		//alarmTime = 43200;
		alarmTime = currentTime + 60;
		/*GREEN_LED = 1;
		waitMicrosecond(1000000);
		GREEN_LED = 0;
		waitMicrosecond(1000000);*/
	}
	else {
		//alarmTime = 0 + 86400;
		alarmTime = currentTime + (86400-currentTime);
		//alarmTime = 43260;
	}
	//alarmTime = 43200;
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_RTCM0_R = alarmTime;
	while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_DATA_R = currentTime;  // Store current time in HIB_DATA0
    while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_DATA_R = (HIB_DATA_R & 0xFFFF0000) | alarmTime;  // Store alarm time in HIB_DATA0 lower 16 bits
	HIB_RIS_R |= HIB_RIS_RTCALT0;
	time = HIB_RTCM0_R;
	uint32_t hh, mm;
	hh = time / 3600;
	mm = (time % 3600) / 60;
	snprintf(string, sizeof(string), "Alarm Time: %02d:%02d\n", hh, mm);
	putsUart0(string);
}

// RTC Interrupt Handler
void RTCIsr(void)
{
	// Wake from RTC Timer
	if (HIB_MIS_R & HIB_MIS_RTCALT0) {
	   	while(!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_IC_R |= HIB_IC_RTCALT0;
		putsUart0("Alarm Time Reached.\n");
		// Retrieve the stored alarm time from HIB_DATA_R
		uint32_t storedData = HIB_DATA_R;
		// uint32_t storedTime = storedData >> 16;  // Upper 16 bits store current time
		uint32_t storedAlarm = storedData & 0xFFFF; // Lower 16 bits store alarm time

		/*while (!(HIB_CTL_R & HIB_CTL_WRC));
			HIB_RTCLD_R = storedTime;  // Restore time

		uint32_t h = storedTime / 3600;
		uint32_t m = (storedTime % 3600) / 60;
		snprintf(string, sizeof(string), "Stored Time: %d:%02d\n", h, m);
		putsUart0(string);*/

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCM0_R = storedAlarm; // Restore alarm

		uint32_t hh = storedAlarm / 3600;
		uint32_t mm = (storedAlarm % 3600) / 60;
		snprintf(string, sizeof(string), "Stored Alarm Time: %02d:%02d\n", hh, mm);
		putsUart0(string);

		// Function check board LED's on Startup
        test_leds();

        // Join the server
        serverJoin();

        setAlarm();

		// alarmTrig = true;
		// GREEN_LED = 1;
		// waitMicrosecond(1000000);
		// GREEN_LED = 0;
		// waitMicrosecond(1000000);

		/*// Restore clock time
		while(!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCLD_R = storedTime;
		uint32_t h = storedTime / 3600;
		uint32_t m = (storedTime % 3600) / 60;
		snprintf(string, sizeof(string), "Stored Time: %d:%02d\n", h, m);
		putsUart0(string);*/
		//checkBatteryLevel();
		//nrf24lo1 packet;
       	packet.frame_id = 0;
       	packet.type = 0x0B;
       	packet.dataLength = 1;
       	packet.data[0] = 0x63;
       	//sendPacket(&packet);
       	//WATCHDOG0_CTL_R |= WDT_CTL_INTEN;
       	//enableWatchdogInterrupt();

        // setAlarm();

		//waitMicrosecond(3000000);
        //while(!(HIB_CTL_R & HIB_CTL_WRC));
        //HIB_CTL_R |= HIB_CTL_HIBREQ;
	}

	// Wake from Glass Break Sensor (Wake Pin)
	if (HIB_MIS_R & HIB_MIS_EXTW) {
		while(!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_IC_R |= HIB_IC_EXTW;
		putsUart0("Woke up from hibernation due glass struck with object or button press\n");

		// Restore stored time and alarm
		uint32_t storedData = HIB_DATA_R;
		uint32_t storedTime = storedData >> 16;  // Upper 16 bits store current time
		uint32_t storedAlarm = storedData & 0xFFFF; // Lower 16 bits store alarm time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCLD_R = storedTime;  // Restore time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCM0_R = storedAlarm; // Restore alarm

		alarmTrig = false;

		snprintf(string, sizeof(string), "Restored Time: %02d:%02d\n", storedTime / 3600, (storedTime % 3600) / 60);
		putsUart0(string);
		snprintf(string, sizeof(string), "Restored Alarm Time: %02d:%02d\n", storedAlarm / 3600, (storedAlarm % 3600) / 60);
		putsUart0(string);

		// Function check board LED's on Startup
        test_leds();

        // Join the server
        serverJoin();

        setAlarm();

		//checkBatteryLevel();

		//while (!(HIB_CTL_R & HIB_CTL_WRC));  // Wait for write complete
		//HIB_IM_R |= HIB_IM_EXTW;  // Re-enable External Wake Interrupt
	}

	// Wake from Low Battery Warning
	if (HIB_MIS_R & HIB_MIS_LOWBAT) {
		while(!(HIB_CTL_R & HIB_CTL_WRC));
	    HIB_IC_R |= HIB_IC_LOWBAT;
	    putsUart0("Battery Level Low!\n");

	    // Restore stored time and alarm
		uint32_t storedData = HIB_DATA_R;
		uint32_t storedTime = storedData >> 16;  // Upper 16 bits store current time
		uint32_t storedAlarm = storedData & 0xFFFF; // Lower 16 bits store alarm time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCLD_R = storedTime;  // Restore time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCM0_R = storedAlarm; // Restore alarm

		alarmTrig = false;

		snprintf(string, sizeof(string), "Restored Time: %02d:%02d\n", storedTime / 3600, (storedTime % 3600) / 60);
		putsUart0(string);
		snprintf(string, sizeof(string), "Restored Alarm Time: %02d:%02d\n", storedAlarm / 3600, (storedAlarm % 3600) / 60);
		putsUart0(string);
	}
	else {
		putsUart0("Battery Level is Ok\n");
	}
}

void GPIOPortB_Handler(void)
{
    GPIO_PORTB_ICR_R = IRQ_MASK;  // Clear PB2 interrupt flag

    uint8_t i;
    uint8_t status = readRegister(STATUS);
    putsUart0("Status: ");
    printHex8(status);

    if (status & (1 << 6)) // RX_DR: Packet received
    {
        putsUart0("IRQ: Packet Received\n");

        uint8_t buffer[32];
        readPayload(buffer, 32);
        nrf24lo1 *recPacket = (nrf24lo1*)(buffer + 7);

        uint8_t computedChecksum = 0;
	   for (i = 1; i < recPacket->dataLength + 4; i++) {
		   computedChecksum += ((uint8_t*)recPacket)[i];
	   }

	   if (computedChecksum != recPacket->checksum)
	   {
		   putsUart0("Invalid message\n");
		   return;
	   }
	   else
	   {
		 if(recPacket->type == 0x04) {
		   putsUart0("SYNC packet received\n");
		   putsUart0("Received Packet Info:\n ");
		   putsUart0("Frame ID: ");
		   printHex8(recPacket->frame_id);
		   putsUart0("Type: ");
		   printHex8(recPacket->type);
		   putsUart0("Data Length: ");
		   printHex8(recPacket->dataLength);
		   for(i = 0; i < recPacket->dataLength; i++) {
			putsUart0("Data: ");
			printHex8(recPacket->data[i]);
		   }
		   //enableTransmitter();
		 }
		   //nrf24lo1 sndPkt;
		   //sndPkt.type = 0x02;
		   //sndPkt.frame_id = pkt->frame_id;
		   //sndPkt.dataLength = 1;
		   //sndPkt.data[0] = 0x02;
		   //sendPacket(&sndPkt);
	   }

        putsUart0("Received Packet Info:\n ");
        putsUart0("Frame ID: ");
        printHex8(recPacket->frame_id);
        putsUart0("Type: ");
        printHex8(recPacket->type);
        putsUart0("Data Length: ");
        printHex8(recPacket->dataLength);
        for(i = 0; i < recPacket->dataLength; i++) {
        	putsUart0("Data: ");
        	printHex8(recPacket->data[i]);
        }

        // You can process your packet here (cast to struct, etc.)
        // Save last received
       //memcpy(&lastReceivedPacket, recPacket, sizeof(nrf24lo1));
        writeRegister(STATUS, (1 << 6)); // Clear RX_DR
    }

    uint8_t fifo_status = readRegister(FIFO_STATUS);
    putsUart0("FIFO: ");
    printHex8(fifo_status);  // Should show TX_EMPTY = 1 after each send
}

void GPIOC_Handler(void)
{
	if(GPIO_PORTC_RIS_R & PIEZOELECTRIC_MASK) {
	     GPIO_PORTC_ICR_R = PIEZOELECTRIC_MASK;
	     putsUart0("Glass struck with object\n");

	     uint32_t currentTime;
		 while (!(HIB_CTL_R & HIB_CTL_WRC));
		 currentTime = HIB_RTCC_R;  // Get current time

		 // Store current time and alarm time in HIB_DATA
		 while (!(HIB_CTL_R & HIB_CTL_WRC));
		 uint32_t alarmTime = HIB_RTCM0_R;
		 HIB_DATA_R = (currentTime << 16) | (alarmTime & 0xFFFF);

		 //while (!(HIB_CTL_R & HIB_CTL_WRC));
		 //HIB_CTL_R |= HIB_CTL_VDD3ON;  // Keep power ON for HIB_DATA_R
		 while (!(HIB_CTL_R & HIB_CTL_WRC));
		 HIB_RIS_R |= HIB_RIS_EXTW;

		 while (!(HIB_CTL_R & HIB_CTL_WRC));
		 HIB_IC_R |= HIB_IC_EXTW;   // Clear External Wake flag

		 waitMicrosecond(100000);
	}
}

// GPIOF Interrupt Handler. For testing external wake-up from Hibernation
void GPIOF_Handler(void)
{
    // Check if PF0 caused the interrupt by pressing button
    if (GPIO_PORTF_RIS_R & SW2_MASK) {
        // Handle button press (PF0)
        GPIO_PORTF_ICR_R = SW2_MASK; // Clear interrupt flag
        //putsUart0("Entering Hibernation\n");

        uint32_t currentTime;
		while (!(HIB_CTL_R & HIB_CTL_WRC));
		currentTime = HIB_RTCC_R;  // Get current time

		// Store current time and alarm time in HIB_DATA
		while (!(HIB_CTL_R & HIB_CTL_WRC));
		uint32_t alarmTime = HIB_RTCM0_R;
		HIB_DATA_R = (currentTime << 16) | (alarmTime & 0xFFFF);

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RIS_R |= HIB_RIS_EXTW;

        //while (!(HIB_CTL_R & HIB_CTL_WRC));  // Wait for write complete
        //HIB_IM_R &= ~(HIB_IM_RTCALT0 | HIB_IM_EXTW | HIB_IM_LOWBAT);            // Disable External Wake Interrupt
        // Clear any pending wake-up flags before hibernation
		//while (!(HIB_CTL_R & HIB_CTL_WRC));
		//HIB_IC_R |= HIB_IC_EXTW;   // Clear External Wake flag
		waitMicrosecond(100000);             // Small delay to debounce button

        //while (!(HIB_CTL_R & HIB_CTL_WRC));  // Wait for write control ready
        //HIB_CTL_R |= HIB_CTL_HIBREQ;         // Enter Hibernation mode
        // This should NEVER print if hibernation is working
        //putsUart0("Entered Hibernation\n");
    }
}


/*************************************************************************************************/
/***************************************** MAIN FUNCTION *****************************************/

int main(void)
{
	USER_DATA data;

	// Initialize Hardware
	initHw();

	// Initialize SPI0
	initSpi0(USE_SSI0_RX);
	setSpi0BaudRate(10e6, 40e6);
	setSpi0Mode(0, 0);

    // Initialize Wireless Module
	initNrf24l01Receiver();

	// Initialize UART
	initUart0();
	setUart0BaudRate(115200, 40e6);

	// Initialize Battery Level Indicator
	init_battery_level();

    // Initialize Watchdog Timer
	initWatchdog0(2e6, 40e6);

	uint32_t hours, seconds;
    uint32_t minutes, time_local;
	uint32_t remaining_seconds;
	uint32_t realTime;
	//while(true) {
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_RTCLD_R = 10800;
	waitMicrosecond(2000000);
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	realTime = HIB_RTCC_R % 86400;                    // After time is set in load register, the clock counter register will start counting up in seconds at time set in load register
	hours = realTime / 3600;                  // Convert seconds to hours
	remaining_seconds = realTime % 3600;
	minutes = remaining_seconds / 60;         // Convert seconds to minutes
    seconds = remaining_seconds % 60;

    if (HIB_RIS_R & HIB_RIS_RTCALT0) {
		while(!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_IC_R |= HIB_IC_RTCALT0;
		putsUart0("Alarm Time Reached.\n");
		// Retrieve the stored alarm time from HIB_DATA_R
		uint32_t storedData = HIB_DATA_R;
		// uint32_t storedTime = storedData >> 16;  // Upper 16 bits store current time
		uint32_t storedAlarm = storedData & 0xFFFF; // Lower 16 bits store alarm time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCM0_R = storedAlarm; // Restore alarm

		// Initialize Hardware
		initHw();

		alarmTrig = true;

		uint32_t hh = storedAlarm / 3600;
		uint32_t mm = (storedAlarm % 3600) / 60;
		snprintf(string, sizeof(string), "Stored Alarm Time: %02d:%02d\n", hh, mm);
		putsUart0(string);

		// Function check board LED's on Startup
        test_leds();

        // Join the server
        serverJoin();

        setAlarm();
    }

    if (HIB_RIS_R & HIB_RIS_EXTW) {
		while(!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_IC_R |= HIB_IC_EXTW;
		putsUart0("Woke up from hibernation due glass struck with object or button press\n");

		// Restore stored time and alarm
		uint32_t storedData = HIB_DATA_R;
		uint32_t storedTime = storedData >> 16;  // Upper 16 bits store current time
		uint32_t storedAlarm = storedData & 0xFFFF; // Lower 16 bits store alarm time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCLD_R = storedTime;  // Restore time

		while (!(HIB_CTL_R & HIB_CTL_WRC));
		HIB_RTCM0_R = storedAlarm; // Restore alarm

		// Initialize Hardware
		initHw();

		alarmTrig = false;

		snprintf(string, sizeof(string), "Restored Time: %02d:%02d\n", storedTime / 3600, (storedTime % 3600) / 60);
		putsUart0(string);
		snprintf(string, sizeof(string), "Restored Alarm Time: %02d:%02d\n", storedAlarm / 3600, (storedAlarm % 3600) / 60);
		putsUart0(string);

		// Function check board LED's on Startup
        test_leds();

        // Join the server
        serverJoin();

        setAlarm();

    }

	// Function check board LED's on Startup
	test_leds();

	// Join the server
	serverJoin();

	setAlarm();

	GREEN_LED = 0;

	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_CTL_R |= HIB_CTL_VDD3ON;
	while(!(HIB_CTL_R & HIB_CTL_WRC));
	HIB_CTL_R |= HIB_CTL_HIBREQ;
	// }

	while(true);
}
