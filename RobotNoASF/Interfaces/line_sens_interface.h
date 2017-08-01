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

#if defined ROBOT_TARGET_V1
//TODO: Layout defines for the PIO pins that the line followers are connected to. They aren't
//connected to ADC channels, but they may still be able to work by setting the threshold through
//their pull up resistors.
#define LF0_PORT		PIOA			//Line follower 0 Port
#define LF1_PORT		PIOC			//Line follower 1 Port
#define LF2_PORT		PIOC			//Line follower 2 Port
#define LF3_PORT		PIOA			//Line follower 3 Port
#define LF0				PIO_PA5			//Line follower 0 PIO pin
#define LF1				PIO_PC28		//Line follower 1 PIO pin
#define LF2				PIO_PC10		//Line follower 2 PIO pin
#define LF3				PIO_PA2			//Line follower 3 PIO pin
#endif

#if defined ROBOT_TARGET_V2
#define LF_THRESHOLD_H	2730			//Upper Threshold above which line is no longer detected
#define LF_THRESHOLD_L	1365			//Lower threshold below which line is detected
#define LF0				LF0_ADC_CH		//Line follower 0 ADC channel
#define LF1				LF1_ADC_CH		//Line follower 1 ADC channel
#define LF2				LF2_ADC_CH		//Line follower 2 ADC channel
#define LF3				LF3_ADC_CH		//Line follower 3 ADC channel
#define LFC_PORT		PIOA			//PIO Port def for the line follower LED control
#define LFC				PIO_PA8			//PIO pin def for the line follower LED control
#endif

///////////////Functions////////////////////////////////////////////////////////////////////////////
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