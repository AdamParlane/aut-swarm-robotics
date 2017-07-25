/*
* docking_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:07:50 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* [WIP] Contains functions for docking... [WIP]
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "docking_functions.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void dockRobot(void)
*
* Function to guide the robot to the dock.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* assume max brightness is 0-100 (scale it to make this work)
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void dockRobot(void)
{
	//************Approx light ranges**************//
	//Ambient light in WS217 0x00DE - 0x0158
	//LED @ 10cm straight on 0x0abe - 0x12ea
	//LED @ 20cm straight on 0x05da - 0x06c6
	//LED @ 30cm straight on 0x0443 - 0x04a9
	//LED @ 30cm 30dg offset High ~0x04a9 Low ~0x0440
	//LED @ 30cm 60dg offset High ~0x033a Low ~0x02b7
	//LED @ 30cm 90dg offset High ~0x00ec Low ~0x00d6
	
	//***********Approx Promity Values*************//
	//Using my hand as an object, testing on side A
	//touching - ~5cm = 0x03ff (max)
	//5 cm away 0x0150 - 0x01ff
	//10cm away 0x0070 - 0x0100
	//20cm away 0x0030 - 0x003f
	//30cm away 0x0020 - 0x0029
	
	uint16_t rightBrightness, leftBrightness;
	float diff = 0;
	float rightBrightnessScaled, leftBrightnessScaled;
	
	leftBrightness = lightSensRead(MUX_LIGHTSENS_L, LS_WHITE_REG);
	rightBrightness = lightSensRead(MUX_LIGHTSENS_R, LS_WHITE_REG);
	
	//frontProximity = proxSensRead(MUX_PROXSENS_A); //need to test this
	
	if(rightBrightness > 0x0200 && leftBrightness > 0x0200)//if there is more light than ambient
	{
		//Scale brightness to calculate required position
		rightBrightnessScaled = (rightBrightness / 0xFFFF) * 100;
		leftBrightnessScaled = (leftBrightness / 0xFFFF) * 100;
		//Zero Justified Normalized Differential Shade Calculation
		diff = 2 * (((rightBrightnessScaled * 100)/(rightBrightnessScaled + leftBrightnessScaled)) - 50);
		//Convert to degrees
		moveRobot(diff/2, 50);
	}
	else if((leftBrightness > 0x1000 || rightBrightness >  0x1000))// && frontProximity > 0x0300)
	{
		stopRobot();
	}
	else if((rightBrightness - leftBrightness) > 0x009F)
	{
		rotateRobot(CW, 30); //turn right
	}
	else if((leftBrightness - rightBrightness) > 0x009F)
	{
		rotateRobot(CCW, 30);//turn left
	}
}