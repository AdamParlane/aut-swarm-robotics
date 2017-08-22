/*
* light_sens_interface.h
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:06:37 AM
*
* Project Repository:https://github.com/AdamParlane/aut-swarm-robotics
*
* All functions and defines relating to accessing the light/colour sensors go here
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* VEML6040 colour sensor Datasheet:https://www.vishay.com/docs/84276/veml6040.pdf
*
* Functions:
* void lightSensInit(uint8_t channel)
* uint16_t lightSensRead(uint8_t channel, uint8_t colour)
* uint8_t lightSensCapture(uint8_t channel, struct ColourSensorData *colours)
* void lightSensRGB2HSV(struct ColourSensorData *colours)
*
*/

#ifndef LIGHT_SENS_INTERFACE_H_
#define LIGHT_SENS_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "sam.h"

//////////////[Type Definitions]////////////////////////////////////////////////////////////////////
struct ColourSensorData
{
	uint16_t red;
	uint16_t green;
	uint16_t blue;
	uint16_t white;
	float hue;
	uint16_t saturation;
	uint16_t value;
};

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Register addresses for reading Colour
#define LS_RED_REG			0x08
#define LS_GREEN_REG		0x09
#define LS_BLUE_REG			0x0A
#define LS_WHITE_REG		0x0B	//White light is detected at the tower for docking alignment
//Register address for configuration
#define LS_CONFIG_REG		0x00
//Config register settings (0x00)
////Lower nibble settings (Trigger or Auto mode)
#define LS_DETECT			0x06		//Lower nibble settings Trigger one time/ force mode
#define LS_AUTO				0x00		//Auto mode, no triggering
////Upper nibble settings (Light sensitivity)
#define LS_40MS				0x06		//40ms integration time (High sensitivity)
#define LS_80MS				0x10		//80ms integration time
#define LS_160MS			0x20		//160ms integration time
#define LS_320MS			0x30		//3200ms integration time
#define LS_640MS			0x40		//640ms integration time
#define LS_1280MS			0x50		//1280ms integration time, (Low sensitivity)
////Maximum light sensor value
#define MAX_LIGHT_CHANNEL_VAL	65535

////Hue angle colour ranges


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void lightSensInit(uint8_t channel)
*
* Sets initial settings for given light sensor
*
* Inputs:
* uint8_t channel:
*   The I2C multiplexer channel for the desired light sensor (see twimux_interface.h). Only sets up
*   one light sensor at a time, not both
*
* Returns:
* none
*
*/
void lightSensInit(uint8_t channel);

/*
* Function:
* uint16_t lightSensRead(uint8_t channel, uint8_t colour)
*
* Retrieves the (16-bit) light data of the given colour from the selected Light Sensor
*
* Inputs:
* uint8_t channel:
*   The I2C mulitplexer channel of the light sensor to read from.
*   MUX_LIGHTSENS_R for the right sensor or MUX_LIGHTSENS_L for the left
* uint8_t colour:
*   The colour channel to read from.
*   LS_RED_REG for red
*   LS_GREEN_REG for green
*   LS_BLUE_REG for blue
*   LS_WHITE_REG for white
*
* Returns:
* a 16bit unsigned integer containing the light level value from the sensor
*
*/
uint16_t lightSensRead(uint8_t channel, uint8_t colour);

/*
* Function:
* uint8_t lightSensCapture(uint8_t channel, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
*
* Retrieves the (16-bit) light data of all colours from the selected Light Sensor
*
* Inputs:
* uint8_t channel:
*   The I2C mulitplexer channel of the light sensor to read from.
*   MUX_LIGHTSENS_R for the right sensor or MUX_LIGHTSENS_L for the left
* uint16_t *r
*   Pointer to a 16bit integer to store red channel data
* uint16_t *g
*   Pointer to a 16bit integer to store green channel data
* uint16_t *b
*   Pointer to a 16bit integer to store blue channel data
* uint16_t *w
*   Pointer to a 16bit integer to store white channel data
*
* Returns:
* 0 on success, or non-zero when TWI error occurred.
*
*/
uint8_t lightSensCapture(uint8_t channel, struct ColourSensorData *colours);

/*
* Function:
* void lightSensRGB2HSV(struct ColourSensorData *colours)
*
* Converts RGB to HSV and stores them in a ColourSensorData structure
*
* Inputs:
* struct ColourSensorData *colours
*   Pointer to a ColourSensorData structure to store the calculated HSV values
*
* Returns:
* none
*
*/
void lightSensRGB2HSV(struct ColourSensorData *colours);

#endif /* LIGHT_SENS_INTERFACE_H_ */