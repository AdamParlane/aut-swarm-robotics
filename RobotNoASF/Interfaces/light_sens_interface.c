/*
* light_sens_interface.c
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:06:37 AM
*
* Project Repository:https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions relating to accessing the light/colour sensors
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* VEML6040 colour sensor Datasheet:https://www.vishay.com/docs/84276/veml6040.pdf
*
* Functions:
* void lightSensInit(uint8_t channel)
* uint16_t lightSensRead(uint8_t channel, uint8_t colour)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "light_sens_interface.h"
#include "twimux_interface.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
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
* Implementation:
* - Sets I2C multiplexer to the channel that the desired light sensor is connected to.
* - Writes out initial settings defined in LS_AUTO_LOW_LUX; (White Light Detection Enabled
*   with low lux (40ms integration time), No Trigger, Auto Mode)
*
* Improvements:
* Perhaps could just setup both sensors at once?
*
*/

void lightSensInit(uint8_t channel)
{
	twi0MuxSwitch(channel); //Set multiplexer address to correct device
	twi0Write(TWI0_LIGHTSENS_ADDR, LS_CONFIG_REG, 1, &LS_AUTO_LOW_LUX);
}

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
* Implementation:
* First, the multiplexer on TWI0 is set to the channel for the desired light sensor.
* Next, the desired colour value from the selected sensor is read and returned by twi0Read().
* TWI0_LIGHTSENS_ADDR is the I2C address of the light sensors, and colour is an internal register
* address.
*
*/
uint16_t lightSensRead(uint8_t channel, uint8_t colour)
{
	uint16_t data;
	twi0MuxSwitch(channel);	//Set multiplexer address to a light sensor device
	twi0Read(TWI0_LIGHTSENS_ADDR, colour, 2, &data);
	return data;
}
