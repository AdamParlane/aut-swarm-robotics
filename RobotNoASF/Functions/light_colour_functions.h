/*
* light_colour_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 5/09/2017 6:50:27 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Header for light_colour_functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void lcfRetrieveLightData(uint8_t convertToHSV)
* uint8_t lcfCapture(uint8_t channel, struct ColourSensorData *colours)
* void lcfRGB2HSV(struct ColourSensorData *colours)
*
*/

#ifndef LIGHT_COLOUR_FUNCTIONS_H_
#define LIGHT_COLOUR_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Parameter definitions for lcfRetrieveLightData()
#define LCF_RGB_ONLY		0
#define LCF_RGB_AND_HSV		1

////Hue angle constants (Should help speed up maths)
#define LCF_MAX_HUE_ANGLE	360
#define LCF_HUE_ANGLE_DIV3	120
#define LCF_HUE_ANGLE_DIV6	60

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void lcfRetrieveLightData(uint8_t convertToHSV)
*
* Will retrieve light sensor data and store it in the data structures. Can also perform RGB to HSV
* conversions if desired
*
* Inputs:
* uint8_t convertToHSV:
*   If LCF_RGB_ONLY (0) is passed as the parameter, then this function will capture data from the
*   sensors and store it, but will not perform HSV conversion. if LCF_RGB_AND_HSV (1) is passed then
*   RGB to HSV conversion will be performed as well.
*
* Returns:
* None
*
*/
void lcfRetrieveLightData(uint8_t convertToHSV);

/*
* Function:
* uint8_t lcfCapture(uint8_t channel, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
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
uint8_t lcfCapture(uint8_t channel, ColourSensorData *colours);

/*
* Function:
* void lcfRGB2HSV(struct ColourSensorData *colours)
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
void lcfRGB2HSV(ColourSensorData *colours);

#endif /* LIGHT_COLOUR_FUNCTIONS_H_ */