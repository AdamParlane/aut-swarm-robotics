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
* uint8_t scanBrightestLightSource(int16_t *brightestHeading)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "docking_functions.h"
#include <stdlib.h>				//abs() function in followLine()

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//Light follower sensor states.
struct LineSensorArray lf;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void dockRobot(void)
*
* Function to guide the robot to the dock.
*
* Inputs:
* struct Position *imuData
*   Pointer to the robotPosition structure
*
* Returns:
* 0 when docking complete, otherwise non-zero
*
* Implementation:
* The docking function is a state machine that will change states after each step that is required
* for docking is performed. More to come [WIP]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
uint8_t dockRobot(struct Position *imuData)
{
	float bHeading = 0;	//Brightest Heading
	enum {FINISHED, START, FACE_BRIGHTEST, MOVE_FORWARD, RESCAN_BRIGHTEST, FOLLOW_LINE};
	static uint8_t dockingState = START;
	uint8_t returnVal;
	///////////////[WIP]///////////////
	switch(dockingState)
	{
		case START:
//			rotateToHeading(0, imuData);
			ledOff1;
			ledOff2;
			ledOff3;
			//returnVal = updateLineSensorStates();
			if(!scanBrightestLightSource(&bHeading, 359, imuData))
				dockingState = FACE_BRIGHTEST;
		break;
		
		case FACE_BRIGHTEST:
			ledOn1;
			ledOff2;
			ledOff3;
			if(!rotateToHeading(bHeading, imuData))
				dockingState = MOVE_FORWARD;
		break; 
		
		case MOVE_FORWARD:
			ledOn1;
			ledOn2;
			ledOff3;
			moveRobot(0, 50);
			//RIN_3_H;
			//FIN_3_L;
			//RIN_2_L;
			//FIN_2_H;
			//REG_PWM_CUPD2 = 30;			//Left Front
			//REG_PWM_CUPD3 = 45;			//Right front
			//
			//RIN_1_L;
			//FIN_1_L;
			//REG_PWM_CUPD1 = 0;			//rear
			if(!fdelay_ms(3000))			//After five seconds, look for LEDs again
			{
				stopRobot();
				dockingState= RESCAN_BRIGHTEST;
			}
			if(updateLineSensorStates())	//If line found then follow it
			{
				stopRobot();
				dockingState = FOLLOW_LINE;
			}
		break;
		
		case RESCAN_BRIGHTEST:
			ledOff1;
			ledOff2;
			ledOn3;
			//Only look in front, because we should still be roughly in the right direction
			if(!scanBrightestLightSource(&bHeading, 180, imuData))
				dockingState = FACE_BRIGHTEST;
		break;
		
		case FOLLOW_LINE:
			ledOn1;
			ledOff2;
			ledOn3;		
			followLine();
		break;
		
		case FINISHED:
			ledOn1;
			ledOn2;
			ledOn3;
			//trackLight(imuData);
			return 0;
		break;
	}
	
	return dockingState;
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
* 1 if line state change detected, otherwise 0
*
* Implementation:
* - Read state value of sensor.
* - if a state other than NO_CHANGE is returned then update the value stored in the line sensor
*   data structure for the given sensor.
* - Repeat until all four sensors have been read.
*
*/
uint8_t updateLineSensorStates(void)
{
#if defined ROBOT_TARGET_V2
	uint8_t sensorValue;						//Temporarily stores state of a single sensor
	uint8_t returnVal = 0;						//Returns non 0 if any sensor has changed state
	
	sensorValue = lfLineDetected(LF_OUTER_L);	//Look for line on outer left sensor
	if (sensorValue != NO_CHANGE)				//If this sensor has changed state
		lf.outerLeft = sensorValue;				//Update line follower data structure with new data
	if(sensorValue == LINE)
		returnVal = 1;
	
	sensorValue = lfLineDetected(LF_INNER_L);	//Look for line on inner left sensor
	if (sensorValue != NO_CHANGE)				//If this sensor has changed state
		lf.innerLeft = sensorValue;				//Update line follower data structure with new data
	if(sensorValue == LINE)
		returnVal = 1;
	
	sensorValue = lfLineDetected(LF_OUTER_R);	//Look for line on outer right sensor
	if (sensorValue != NO_CHANGE)				//If this sensor has changed state
		lf.outerRight = sensorValue;			//Update line follower data structure with new data
	if(sensorValue == LINE)
		returnVal = 1;
	
	sensorValue = lfLineDetected(LF_INNER_R);	//Look for line on inner right sensor
	if (sensorValue != NO_CHANGE)				//If this sensor has changed state
		lf.innerRight = sensorValue;			//Update line follower data structure with new data
	if(sensorValue == LINE)
		returnVal = 1;
		
	return returnVal;
#endif
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
#if defined ROBOT_TARGET_V2
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
#endif
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
#if defined ROBOT_TARGET_V2
	int8_t lineDirection = getLineDirection();
	if (lineDirection > 0)			//Turn right
		rotateRobot(CCW, abs(lineDirection)*10);
	else if (lineDirection < 0)			//Turn left
		rotateRobot(CW, abs(lineDirection)*10);
	else
	{
		// moveRobot(0, 25) didn't do anything, hence this block of code (move forward at 25%)
		//TODO: Want to replace this now Matt?
		moveRobot(0, 35);
		//RIN_1_H;
		//FIN_1_L;
		//RIN_3_L;
		//FIN_3_H;
		//REG_PWM_CUPD3 = 35;			//Left Front
		//REG_PWM_CUPD1 = 35;			//Right front
		//
		//RIN_2_L;
		//FIN_2_L;
		//REG_PWM_CUPD2 = 0;			//rear
	}
#endif
}

/*
* Function:
* uint8_t scanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle,
*									struct Position *imuData);
*
* The robot will scan from -180 degrees to 180 degrees and record the heading with the brightest
* source of light (which hopefully is the charging station)
*
* Inputs:
* int16_t *brightestHeading
*   A pointer to a variable that contains a heading to the brightest detected light source so far.
*
* Returns:
* Returns a 1 if the function hasn't completed yet, or a 0 if it has. When the function returns a 0
* it means the heading stored at *breightestHeading points to the brightest light source.
*
* Implementation:
* heading is a static variable that stores the heading that the robot is currently moving to.
* brightestVal is a static variable that stored the brightest detected light value so far.
* avgBrightness is a temporary variable that stores the average brightness between the two light
* sensors.
* First up the function calls the rotateToHeading function. If that function has completed (ie the
* robot is facing in the heading we want) then an average white light brightness reading is taken
* from the light sensors. If the average brightness just read is greater than the last stored
* brightness value then update brightestHeading with the current heading and update brightestVal
* with the current avgBrightness. Once the robot has rotated 360 degrees, return 0 and reset the
* static variable to their starting states to indicate that the scan is complete. The heading
* left behind in brightest heading is the heading with the greatest amount of light.
*
* Improvements:
* Have the robot scan in one continuous sweep rather than stopping at each heading to read light 
* values
* Currently it seems to not lock dead on with the light source.
*
*/
uint8_t scanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle, 
								struct Position *imuData)
{
	const float ROTATE_STEP_SZ = 2;
	enum {FUNCTION_INIT, GOTO_START_POSITION, SWEEP, END};
	static uint8_t functionState = FUNCTION_INIT;
	static float startHeading;
	static float endHeading;
	static float sHeading;
	static uint16_t brightestVal;
	float rotateError;
	uint16_t avgBrightness = 0;
	
	switch(functionState)
	{
		case FUNCTION_INIT:
			//Calculate where to start sweep from
			brightestVal = 0;								//Reset brightestValue
			startHeading = imuData->imuYaw - (sweepAngle/2);//Calculate start heading
			endHeading = startHeading + sweepAngle;
			sHeading = startHeading + sweepAngle/3;
			functionState = GOTO_START_POSITION;			//Angles set up, lets start
			return 1;
		break;

		case GOTO_START_POSITION:
			if(!rotateToHeading(startHeading, imuData))
				functionState = SWEEP;						//In position, now perform sweep
			return 1;
		break;
		
		case SWEEP:
			rotateError = rotateToHeading(sHeading, imuData);
			if(abs(rotateError) < 170 && sHeading < endHeading)//Keep sHeading only 25 degrees ahead
															//of current heading so that robot will
															//always take the long way around
			{
				sHeading += ROTATE_STEP_SZ;
				if(sHeading > endHeading)
					sHeading = endHeading;
			}
			if(!rotateError)
				functionState = END;
			else
			{
				avgBrightness = (lightSensRead(MUX_LIGHTSENS_L, LS_WHITE_REG) +
										lightSensRead(MUX_LIGHTSENS_R, LS_WHITE_REG))/2;
				if(avgBrightness > brightestVal)
				{
					brightestVal = avgBrightness;
					*brightestHeading = imuData->imuYaw;
				}
			}
			return 1;
		break;
		
		case END:
			functionState = FUNCTION_INIT;
			return 0;
	}
	return 1;
}

//TODO:Header
float scanBrightestLightSourceProx(void)
{
	uint16_t sensor[6];
	uint16_t brightestVal;
	int brightestSensor = 0;
	//Enable Ambient light mode on the prox sensors
	proxAmbModeEnabled();

	//Read light sensor values
	sensor[0] = proxAmbRead(MUX_PROXSENS_A);		//0
	sensor[1] = proxAmbRead(MUX_PROXSENS_B);		//60
	sensor[3] = proxAmbRead(MUX_PROXSENS_C);		//120
	sensor[4] = proxAmbRead(MUX_PROXSENS_D);		//180
	sensor[5] = proxAmbRead(MUX_PROXSENS_E);		//-120
	sensor[6] = proxAmbRead(MUX_PROXSENS_F);		//-60
	//Revert to proximity mode
	proxModeEnabled();
	
	//Find largest
	for (int i = 0; i < 6; i++)
	{
		if(sensor[i] > brightestVal)
		{
			brightestVal = sensor[i];
			brightestSensor = i;
		}
	}
	
	return imuWrapAngle(60*brightestSensor);
}