/*
 * line_sens_interface.c
 *
 * Created: 
 *  Author: Matthew
 */ 
/*
* line_sens_interface.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 1/08/2017 10:47:55 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for reading line sensors and detecting the presence of lines.
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

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "line_sens_interface.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void lfLedState(uint8_t ledState)
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
* Implementation:
* TODO: Implementation description for lfLedState()
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void lfLedState(uint8_t ledState)
{
	
}

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
* Implementation:
* TODO: Implementation description for lfLineDetected()
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
uint8_t lfLineDetected(uint8_t lfSensor)
{
	
}