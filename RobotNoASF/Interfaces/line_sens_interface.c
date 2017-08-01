/*
* line_sens_interface.c
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
#include "component/pio.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////

void lfInit(void)
#if defined ROBOT_TARGET_V1
{
	//Setup LF sensor pins for (binary) input
	LF0_PORT->PIO_PER			//Enable LF0 sensor pin
	|=	LF0;
	LF1_PORT->PIO_PER			//Enable LF1 sensor pin
	|=	LF1;
	LF2_PORT->PIO_PER			//Enable LF2 sensor pin
	|=	LF2;
	LF3_PORT->PIO_PER			//Enable LF3 sensor pin
	|=	LF3;
}
#endif

#if defined ROBOT_TARGET_V2
{
	//Initialise PA8 (LFC) for output so LEDs can be turned on and off
	LFC_PORT->PIO_PER
	|=	LFC;
	LFC_PORT->PIO_OER
	|=	LFC;
	
	lfLedState(ON);
}
#endif

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
#if defined ROBOT_TARGET_V1
{
	//There is no ability to turn the LEDs on or off on the V1 (Hard wired on)
}
#endif

#if defined ROBOT_TARGET_V2
{
	if (ledState == OFF)
		LFC_PORT->PIO_CODR |= LFC;	//Turn LEDs off
	if (ledState == ON)
		LFC_PORT->PIO_SODR |= LFC;	//Turn LEDs on
}
#endif

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
#if defined ROBOT_TARGET_V1
{
	//Thresholds are set in hardware by pull up resistors
	switch (lfSensor)			//Pick the selected sensor and output it's value.
	{
		case LF0:
			return LF0_PORT[PIO_IDR] & LF0;
		
		case LF1:
			return LF1_PORT[PIO_IDR] & LF1;
		
		case LF2:
			return LF2_PORT[PIO_IDR] & LF2;
		
		case LF3:
			return LF3_PORT[PIO_IDR] & LF3;
	}
	return 0;
}
#endif

#if defined ROBOT_TARGET_V2
{
	uint16_t sensorData = 0;
	sensorData = adcRead(lfSensor);
	
	if (sensorData > LF_THRESHOLD_H)	//if above threshold then line detected
		return LINE;						
	if (sensorData < LF_THRESHOLD_L)	//if below threshold then white floor detected.
		return NO_LINE;
		
	return NO_CHANGE;
}
#endif