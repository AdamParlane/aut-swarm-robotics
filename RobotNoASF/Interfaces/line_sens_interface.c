/*
* line_sens_interface.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 1/08/2017 10:47:55 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
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

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "line_sens_interface.h"
#include "adc_interface.h"
#include "component/pio.h"		//Allows pin assignment macros

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
* Implementation:
* Configures the LED control pin for output so that the line sensor LEDs can be turned on
*
*/
void lfInit(void)
//Line followers are incorrectly wired on the V1, and are therefore unavailable.
{
	//Initialise PA8 (LFC) for output so LEDs can be turned on and off
	LFC_PORT->PIO_PER
	|=	LFC;
	LFC_PORT->PIO_OER
	|=	LFC;
	
	lfLedState(ON);
}

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
	if (ledState == OFF)
		LFC_PORT->PIO_SODR |= LFC;	//Turn LEDs off
	if (ledState == ON)
		LFC_PORT->PIO_CODR |= LFC;	//Turn LEDs on
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
* Returns 1 above upper threshold, and 0 if below lower threshold. If in between then returns 2.
*
* Implementation:
* TODO: Implementation description for lfLineDetected()
* 
* Read ADC for given sensor.
* Compare to threshold levels set in header file.
* If greater than upper limit, then no line detected. If less than lower limit then line is detected
*
* Improvements:
* TODO: Tuning of thresholds will be neccessary for line followers
* In the switch statement, the pin defines might not be exclusive. ie, PIO_PA5 = PIO_PC5 by
* definition, meaning they yield the same number, which would lead to an incorrect selection in
* the switch statement below. Not likely to ever be a problem for us but should be documented
* nonetheless.
*
*/
uint8_t lfLineDetected(uint8_t lfSensor)
{
	uint16_t sensorData = 0;
	sensorData = adcRead(lfSensor);
	
	if (sensorData > LF_THRESHOLD_H)	//if above threshold then line detected
		return LINE;						
	if (sensorData < LF_THRESHOLD_L)	//if below threshold then white floor detected.
		return NO_LINE;
	return NO_CHANGE;
}
