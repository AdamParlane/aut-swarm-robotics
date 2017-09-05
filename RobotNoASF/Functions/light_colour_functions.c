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
	
	if(convertToHSV == LCF_RGB_AND_HSV)
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
	float cMax = 0;					//Holds maximum colour channel value (Normalised)
	float cMin = 1;					//Holds minimum colour channel value (Normalised)
	
	//Normalise colour channel values
	float nRed = colours->red/MAX_LIGHT_CHANNEL_VAL;
	float nGreen = colours->green/MAX_LIGHT_CHANNEL_VAL;
	float nBlue = colours->blue/MAX_LIGHT_CHANNEL_VAL;
	
	//Find maximum colour channel value (cMax)
	if(nBlue > cMax)
		cMax = nRed;
	if(nBlue > cMax)
		cMax = nGreen;
	if(nBlue > cMax)
		cMax = nBlue;
	
	//Find minimum colour channel value (cMin)
	if(nRed < cMin)
		cMin = nRed;
	if(nGreen < cMin)
		cMin = nGreen;
	if(nBlue < cMin)
		cMin = nBlue;
	
	//Get Hue (0-360)
	if(cMax == nRed)
		colours->hue = (unsigned short)(60*((nGreen - nBlue)/(cMax - cMin)))%6;
	if(cMax == nGreen)
		colours->hue = 60*(((nBlue - nRed)/(cMax - cMin)) + 2);
	if(cMax == nBlue)
		colours->hue = 60*(((nRed - nGreen)/(cMax - cMin)) + 4);
	
	//Get Saturation (0-MAX_LIGHT_CHANNEL_VAL)
	if(cMax == 0)
		colours->saturation = 0;
	else
		colours->saturation = (uint16_t)(((cMax - cMin)/(float)cMax)*MAX_LIGHT_CHANNEL_VAL);
	
	//Get Value	(0-MAX_LIGHT_CHANNEL_VAL)
	colours->value = cMax;
}