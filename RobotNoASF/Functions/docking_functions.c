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
* void dockRobot(void)
* void updateLineSensorStates(void)
* int8_t getLineDirection(void)
* void followLine(void)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "docking_functions.h"

///////////////Global variables/////////////////////////////////////////////////////////////////////
//Light follower sensor states.
struct LineSensorArray lf;

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

/*
* Function:
* void updateLineSensorStates(void)
*
* Sees if any sensors have made a definite state change and loads the states into the line sensor
* state structure for use by other functions in this module.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Read state value of sensor.
* - if a state other than NO_CHANGE is returned then update the value stored in the line sensor
*   data structure for the given sensor.
* - Repeat until all four sensors have been read.
*
*/
void updateLineSensorStates(void)
{
	uint8_t sensorValue;
	
	sensorValue = lfLineDetected(LF_OUTER_L);
	if (sensorValue != NO_CHANGE)
		lf.outerLeft = sensorValue;
		
	sensorValue = lfLineDetected(LF_INNER_L);
	if (sensorValue != NO_CHANGE)
		lf.innerLeft = sensorValue;
		
	sensorValue = lfLineDetected(LF_OUTER_R);
	if (sensorValue != NO_CHANGE)
		lf.outerRight = sensorValue;
		
	sensorValue = lfLineDetected(LF_INNER_R);
	if (sensorValue != NO_CHANGE)
		lf.innerRight = sensorValue;
}

/*
* Function:
* int8_t getLineDirection(void)
*
* This function examines the states of the line follower sensors and determines the direction and
* urgency factor by which the robot should move to find its way to the centre of the line.
*
* Inputs:
* none
*
* Returns:
* returns a signed integer between -3 and 3 that determines the direction and speed magnitude that 
* the robot should move to find the centre of the line.
* A negative output means that the robot should move left to find the line and a positive output
* means that the robot should move right. 0 means keep going straight because no direction data is
* able to be derived from sensor array.
*
* Implementation:
* All the line sensor states are loaded into a single byte so that they can easily managed by a
* switch statement rather than a series of unwieldy if statements. A table describing the states
* and there digital values can be seen below:
*
* Sensor array state descriptions:
* _______________________________________________________________________________________
*| State  | Outer left | Inner left | Inner right | Outer right | Function Output value: |
*|________|_____8______|_____4______|______2______|______1______|________________________|
*|__0x0___|____________|____________|_____________|_____________|__________0_____________|
*|__0x8___|_____X______|____________|_____________|_____________|_________-3_____________|
*|__0xC___|_____X______|_____X______|_____________|_____________|_________-2_____________|
*|__0xE___|_____X______|_____X______|______X______|_____________|_________-1_____________|
*|__0xF___|_____X______|_____X______|______X______|______X______|__________0_____________|
*|__0x7___|____________|_____X______|______X______|______X______|__________1_____________|
*|__0x3___|____________|____________|______X______|______X______|__________2_____________|
*|__0x1___|____________|____________|_____________|______X______|__________3_____________|
*|__0x2___|____________|____________|______X______|_____________|__________1_____________|
*|__0x4___|____________|_____X______|_____________|_____________|_________-1_____________|
*|__0x6___|____________|_____X______|______X______|_____________|__________0_____________|     
*
*/
int8_t getLineDirection(void)
{
	//Get updated sensor data
	updateLineSensorStates();
	//Combine sensor states from sensor structure into single byte that can be used by a switch
	//statement. See above for description of each state.
	uint8_t sensorStates
	=	(lf.outerLeft<<3)	//Outer left has binary weighting 8
	|	(lf.innerLeft<<2)	//Inner left has binary weighting 4
	|	(lf.innerRight<<1)	//Inner right has binary weighting 2
	|	(lf.outerRight<<0);	//Outer right has binary weighting 1
	
	switch(sensorStates)
	{
		case 0x0:
			return 0;		//Straight
		case 0x8:
			return -3;		//Move left by factor 3
		case 0xC:
			return -2;		//Move left by factor 2
		case 0xE:
			return -1;		//Move left by factor 1
		case 0xF:
			return 0;		//Straight
		case 0x7:
			return 1;		//Move right by factor 1
		case 0x3:
			return 2;		//Move right by factor 2
		case 0x1:
			return 3;		//Move right by factor 3
		case 0x2:
			return 1;		//Move right by factor 1
		case 0x4:
			return -1;		//Move left by factor 1
		case 0x6:
			return 0;		//Straight
	}
	return 0;
}

/*
* Function:
* void followLine(void)
*
* A basic function to follow a line that seems to work ok
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Get the direction of the detected line. If line is to the left, rotate left until it isn't. If
* line is to the right then rotate to the right until it isn't. If no line detected or line is in
* middle of sensor array then move straight.
*
* Improvements:
* Would be better with the wiggleForward function once its working.
*
*/
void followLine(void)
{
	int8_t lineDirection = getLineDirection();
	if (lineDirection > 0)			//Turn right
		rotateRobot(CW, abs(lineDirection)*10);
	else if (lineDirection < 0)			//Turn left
		rotateRobot(CCW, abs(lineDirection)*10);
	else
	{
		// moveRobot(0, 25) didn't do anything, hence this block of code (move forward at 25%)
		//TODO: Want to replace this now Matt?
		//moveRobot(0, 25);
		RIN_1_H;
		FIN_1_L;
		RIN_3_L;
		FIN_3_H;
		REG_PWM_CUPD3 = 35;			//Left Front
		REG_PWM_CUPD1 = 35;			//Right front
		
		RIN_2_L;
		FIN_2_L;
		REG_PWM_CUPD2 = 0;			//rear
	}
}