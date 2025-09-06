/*	battery_level.c
 *
 *	CSE 4352 - IOT
 *  Project 2
 *  Team 11 - Glass Break Sensor
 *  Members:
 *      Omar Elsaghir
 *      John (Reade) Corr
 * 	Date: April 29, 2025
 */

#include <stdio.h>
#include <stdint.h>
#include "gpio.h"
#include "clock.h"
#include "wait.h"
#include "adc0.h"
#include "uart0.h"

#include "battery_level.h"
#include "tm4c123gh6pm.h"

void init_battery_level(void)
{
    enablePort(PORTE);
    selectPinAnalogInput(BAT_VOLT);

    initAdc0Ss3();
    setAdc0Ss3Mux(3);                          // Use AIN3 input with N=4 hardware sampling
    setAdc0Ss3Log2AverageCount(2);
}

uint8_t check_bat_level(void)
{
    uint16_t x[16];
    uint8_t index = 0;

    uint16_t volt_raw;
    uint16_t fir_volt = 0;
    uint16_t sum = 0; // total fits in 16b since 12b adc output x 16 samples

    uint8_t i;

    // Clear FIR filter taps
    for (i = 0; i < 16; i++) {
        x[i] = 0;
    }

    // Sample battery level 20 times (2 seconds) to get a good average.
    for (i = 0; i < 20; ++i) {
        volt_raw = readAdc0Ss3();

        // FIR sliding average filter with circular addressing
        sum -= x[index];
        sum += volt_raw;
        x[index] = volt_raw;
        index = (index + 1) & 15;
        fir_volt = (uint16_t) (((sum >> 4) * 0.000598048 - 1.37026) * 100 + 0.5);

        waitMicrosecond(100000);    // 100 milisecond delay
    }

    return fir_volt;    // return final average battery level
}
