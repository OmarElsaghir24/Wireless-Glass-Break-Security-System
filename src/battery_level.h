/*	battery_level.h
 *
 *	CSE 4352 - IOT
 *  Project 2
 *  Team 11 - Glass Break Sensor
 *  Members:
 *      Omar Elsaghir
 *      John (Reade) Corr
 * 	Date: April 29, 2025
 */


// #include <inttypes.h>
#include <stdint.h>
// #include <stdbool.h>
// #include <stdio.h>
// #include <string.h>
// #include <ctype.h>


#ifndef BATTERY_LEVEL_H_
#define BATTERY_LEVEL_H_


/*************************************************************************************************/
/**************************** INCLUDES, DEFINES, ASSEMBLER DIRECTIVES ****************************/

#define BAT_VOLT     PORTE,0


/*************************************************************************************************/
/***************************************** SUB-ROUTINES ******************************************/

void init_battery_level(void);
uint8_t check_bat_level(void);


#endif
