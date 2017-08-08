/*
* line_sens_interface.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 11/07/2017 10:07:14 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Prototypes for functions for driving line following sensors
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* QRE1113 Sensor Datasheet:http://www.onsemi.com/pub/Collateral/QRE1113-D.pdf
*
* Functions:
* void lfLedState(uint8_t ledState)
* uint8_t lfLineDetected(uint8_t lfSensor)
*
*/

#ifndef LINE_SENS_INTERFACE_H_
#define LINE_SENS_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
#define ON				1
#define OFF				0

#define LINE			1
#define NO_LINE			0
#define NO_CHANGE		2

//#define LF_THRESHOLD_H	545				//Upper Threshold above which line is no longer detected
//#define LF_THRESHOLD_L	235				//Lower threshold below which line is detected
//Thresholds for insensitive sensor on Red V2
#define LF_THRESHOLD_H	475				//Upper Threshold above which line is no longer detected
#define LF_THRESHOLD_L	375				//Lower threshold below which line is detected
#define LF_OUTER_L		(LF0_ADC_CH)	//Line follower 0 ADC channel
#define LF_INNER_L		(LF1_ADC_CH)	//Line follower 1 ADC channel
#define LF_INNER_R		(LF2_ADC_CH)	//Line follower 2 ADC channel
#define LF_OUTER_R		(LF3_ADC_CH)	//Line follower 3 ADC channel
#define LFC_PORT		(PIOA)			//PIO Port def for the line follower LED control
#define LFC				(PIO_PA8)		//PIO pin def for the line follower LED control

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void lfInit(void)
*
* Initialises Line sensors on V2 robots
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void lfInit(void);

/*
* Function:
* void lfLedState(uint8_t ledState);
*
* Allows line follower LEDs to be switched on and off on the V2+ robots.
*
* Inputs:
* uint8_t lfSensor:
*	1 means turn on LEDs and 0 means turn off.
*
* Returns:
* none
*
*/
void lfLedState(uint8_t ledState);

/*
* Function:
* uint8_t lfLineDetected(uint8_t lfSensor)
*
* Will read value of given light sensor and return 1 or 0 depending if line is detected or not
*
* Inputs:
* uint8_t lfSensor:
*	Sensor to read (LF0-3)
*
* Returns:
* Returns 1 if line detected, otherwise returns 0.
*
*/
uint8_t lfLineDetected(uint8_t lfSensor);

#endif /* LINE_SENS_INTERFACE_H_ */