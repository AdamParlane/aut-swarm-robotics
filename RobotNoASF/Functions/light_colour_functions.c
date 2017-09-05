/*
* light_colour_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 5/09/2017 6:50:27 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for polling data from the colour sensors and converting RGB colour values to HSV
* for colour detection
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

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "light_colour_functions.h"
#include <tgmath.h>				//Used for round in RGB2HSV function

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//Colour sensor data structures. Should be passed as pointer to functions that requires them
struct ColourSensorData lcfLeftSensor;
struct ColourSensorData lcfRightSensor;

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
* Implementation:
* Calls lcfCapture for each sensor, and then if the parameter is given, will convert captured RGB
* data to HSV for both sensors as well.
*
*/
void lcfRetrieveLightData(uint8_t convertToHSV)
{
	lcfCapture(MUX_LIGHTSENS_L, &lcfLeftSensor);
	lcfCapture(MUX_LIGHTSENS_R, &lcfRightSensor);
	
	if(convertToHSV)
	{
		lcfRGB2HSV(&lcfLeftSensor);						//Derive HSV figures from RGB
		lcfRGB2HSV(&lcfRightSensor);						//Derive HSV figures from RGB
	}
}

/*
* Function:
* uint8_t lcfCapture(uint8_t channel, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
*
* Retrieves the (16-bit) light data of all colours from the selected Light Sensor (RGB and HSV)
*
* Inputs:
* struct ColourSensorData *colours
*   Pointer to a ColourSensorData structure, which is where the output will be stored
*
* Returns:
* Always 0
*
* Implementation:
* Uses lightSensRead() to read the values from each colour channel on the given colour sensor and
* loads the data into the ColourSensorData structure provided in the parameters.
* After that, the ColourSensorData structure is passed to lcfRGB2HSV to find the hue,
* saturation and value figures for the retrieved RGB values and stores them in ColourSensorData
* structure.
*/
uint8_t lcfCapture(uint8_t channel, struct ColourSensorData *colours)
{
	uint8_t returnVal = 0;

	colours->red = lightSensRead(channel, LS_RED_REG);		//Read red channel
	colours->green = lightSensRead(channel, LS_GREEN_REG);	//Read green channel
	colours->blue = lightSensRead(channel, LS_BLUE_REG);	//Read blue channel
	colours->white = lightSensRead(channel, LS_WHITE_REG);	//Read white channel
	
	return returnVal;
}

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
* Implementation:
* See
* http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
* https://en.wikipedia.org/wiki/Hue#Computing_hue_from_RGB
*
*/
void lcfRGB2HSV(struct ColourSensorData *colours)
{
	//RGB minimum and maximum
	unsigned short rgbMin = MAX_LIGHT_CHANNEL_VAL;
	unsigned short rgbMax = 0;
	
	//used for hue angle calculation
	int rawHue = 0;
	
	//Find maximum colour channel value (rgbMax)
	if(colours->red > rgbMax)
		rgbMax = colours->red;
	if(colours->green > rgbMax)
		rgbMax = colours->green;
	if(colours->blue > rgbMax)
		rgbMax = colours->blue;

	//Find minimum colour channel value (rgbMin)
	if(colours->red < rgbMin)
		rgbMin = colours->red;
	if(colours->green < rgbMin)
		rgbMin = colours->green;
	if(colours->blue < rgbMin)
		rgbMin = colours->blue;
	
	//Set Value figure to maximum rgb channel figure
	colours->value = rgbMax;
	
	//If HSV value equals 0 then we are looking at pure black (no hue or saturation)
	if (colours->value == 0)
	{
		colours->hue = 0;
		colours->saturation = 0;
		return;
	}
	
	//Calculate saturation
	colours->saturation = MAX_LIGHT_CHANNEL_VAL*(short)((rgbMax - rgbMin)/colours->value);
	
	//If no saturation then we are looking at a perfectly grey item (no hue)
	if (colours->saturation == 0)
	{
		colours->hue = 0;
		return;
	}

	//Calculate Hue angle
	if (rgbMax == colours->red)
		rawHue = 0 + LCF_HUE_ANGLE_DIV6*(colours->green - colours->blue)/(rgbMax - rgbMin);
	else if (rgbMax == colours->green)
		rawHue = LCF_HUE_ANGLE_DIV3 + LCF_HUE_ANGLE_DIV6
					*(colours->blue - colours->red)/(rgbMax - rgbMin);
	else
		rawHue = 2*LCF_HUE_ANGLE_DIV3 + LCF_HUE_ANGLE_DIV6
					*(colours->red - colours->green)/(rgbMax - rgbMin);

	//Wrap rawHue to the range 0-360 and store in colours.hue
	while(rawHue < 0)
		rawHue += 360;
	while(rawHue > 360)
		rawHue -= 360;
	colours->hue = rawHue;

	return;
}