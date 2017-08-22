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
* Approx light ranges
* Ambient light in WS217 0x00DE - 0x0158
* LED @ 10cm straight on 0x0abe - 0x12ea
* LED @ 20cm straight on 0x05da - 0x06c6
* LED @ 30cm straight on 0x0443 - 0x04a9
* LED @ 30cm 30dg offset High ~0x04a9 Low ~0x0440
* LED @ 30cm 60dg offset High ~0x033a Low ~0x02b7
* LED @ 30cm 90dg offset High ~0x00ec Low ~0x00d6
*
* Functions:
* void lightSensInit(uint8_t channel)
* uint16_t lightSensRead(uint8_t channel, uint8_t colour)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "light_sens_interface.h"
#include "twimux_interface.h"

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
	uint8_t writeBuffer = LS_AUTO|LS_80MS;	//Auto trigger, 80ms integration time
	twi0MuxSwitch(channel); //Set multiplexer address to correct device
	twi0Write(TWI0_LIGHTSENS_ADDR, LS_CONFIG_REG, 1, &writeBuffer);
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
	uint8_t data[2];
	twi0MuxSwitch(channel);	//Set multiplexer address to a light sensor device
	twi0Read(TWI0_LIGHTSENS_ADDR, colour, 2, data);
	return (data[1]<<8)|(data[0]);
}

/*
* Function:
* uint8_t lightSensReadAll(uint8_t channel, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
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
* Implementation:
* First, the multiplexer on TWI0 is set to the channel for the desired light sensor.
* Next, all colour channel data is read in a single TWI read operation and stored in the data array.
* After that, the colour data is separated from the data array and stored at the colour pointer
* locations. TWI0_LIGHTSENS_ADDR is the I2C address of the light sensors, and colour is an internal 
* register address.
*
*/
uint8_t lightSensReadAll(uint8_t channel, struct ColourSensorData *colours)
{
	uint8_t returnVal = 0;

	colours->red; = lightSensRead(channel, LS_RED_REG);		//Read red channel
	colours->green; = lightSensRead(channel, LS_GREEN_REG);	//Read green channel
	colours->blue; = lightSensRead(channel, LS_BLUE_REG);	//Read blue channel
	colours->white; = lightSensRead(channel, LS_WHITE_REG);	//Read white channel
	
	lightSensRGB2HSV(colours);
	
	return returnVal;	
}

void lightSensRGB2HSV(struct ColourSensorData *colours)
{
	uint16_t cMax = 0x0000;
	uint16_t cMin = 0xFFFF;
	
	if(colours->red > cMax)
		cMax = colours->red;
	if(colours->green > cMax)
		cMax = colours->green;
	if(colours->blue > cMax)
		cMax = colours->blue;

	if(colours->red < cMin)
		cMax = colours->red;
	if(colours->green < cMin)
		cMax = colours->green;
	if(colours->blue < cMin)
		cMax = colours->blue;
	
	colours->hue = atan2(sqrt(3)*(colours->green - colours->blue),
									2*colours->red - colours->green - colours->blue);
	
	//Get Saturation (0-65535)
	if(cMax == 0)
		colours->saturation = 0;
	else
		colours->saturation = (cMax - cMin)/cMax;
	
	//Get Value										
	colours->value = cMax;
}