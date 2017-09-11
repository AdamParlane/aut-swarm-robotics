/*
* docking_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:07:50 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains the docking routine state machine and functions that are useful for docking...
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void dfDockRobot(void)
* void dfUpdateLineSensorStates(void)
* int8_t dfGetLineDirection(void)
* uint8_t dfFollowLine(void)
* uint8_t dfScanBrightestLightSource(int16_t *brightestHeading)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "docking_functions.h"
#include <stdlib.h>				//abs() function in dfFollowLine()

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//Line follower sensor states.
struct LineSensorArray lf;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void dfDockRobot(void)
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
uint8_t dfDockRobot(struct Position *imuData)
{
	static float bHeading = 0;	//Brightest Heading
	enum {FINISHED, START, FACE_BRIGHTEST, MOVE_FORWARD, RESCAN_BRIGHTEST, FOLLOW_LINE};
	static uint8_t dockingState = START;
	///////////////[WIP]///////////////
	switch(dockingState)
	{
		case START:
			//pioLedNumber(0);
			if(!dfScanBrightestLightSource(&bHeading, 359, imuData))
				dockingState = FACE_BRIGHTEST;
		break;
		
		case FACE_BRIGHTEST:
			if(!mfRotateToHeading(bHeading, imuData))
				dockingState = MOVE_FORWARD;
		break;
		
		case MOVE_FORWARD:
			//pioLedNumber(2);
			//mfMoveToHeading(bHeading, 40, imuData);
			mfTrackLight(imuData);
			if(!fdelay_ms(3700))			//After 3.7 seconds, look for LEDs again
			{
				stopRobot();
				dockingState= RESCAN_BRIGHTEST;
			}
			if(dfUpdateLineSensorStates())	//If line found then follow it
			{
				stopRobot();
				dockingState = FOLLOW_LINE;
			}
		break;
		
		case RESCAN_BRIGHTEST:
			//pioLedNumber(3);
			//Only look in front, because we should still be roughly in the right direction
			if(!dfScanBrightestLightSource(&bHeading, 180, imuData))
				dockingState = FACE_BRIGHTEST;
		break;
		
		case FOLLOW_LINE:
			//pioLedNumber(4);
			if(!dfFollowLine(35, imuData))
				dockingState = FINISHED;
		break;
		
		case FINISHED:
			//pioLedNumber(7);
			dockingState = START;
			return 0;
		break;
	}
	return dockingState;
}

/*
* Function:
* void dfUpdateLineSensorStates(void)
*
* Sees if any sensors have made a definite state change and loads the states into the line sensor
* state structure for use by other functions in this module.
*
* Inputs:
* none
*
* Returns:
* 1 if line state change detected, otherwise 0
* 2 if wrong robot selected
*
* Implementation:
* - Read state value of sensor.
* - if a state other than NO_CHANGE is returned then update the value stored in the line sensor
*   data structure for the given sensor.
* - Repeat until all four sensors have been read.
*
*/
uint8_t dfUpdateLineSensorStates(void)
{
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
}

/*
* Function:
* int8_t dfGetLineDirection(void)
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
int8_t dfGetLineDirection(void)
{
	//Get updated sensor data
	dfUpdateLineSensorStates();
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
			return 0;		//Straight, no line
		case 0x8:
			return -3;		//Move left by factor 3
		case 0xC:
			return -2;		//Move left by factor 2
		case 0xE:
			return -1;		//Move left by factor 1
		case 0xF:
			return 0;		//Straight, line
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
			return 0;		//Straight, line
	}
	return 0;
}

/*
* Function:
* uint8_t dfFollowLine(void)
*
* A basic function to follow a line
*
* Inputs:
* uint8_t speed:
*   Speed at which to follow the line (0-100%)
* struct Position *imuData
*   Pointer to the global robotPosition data structure
*
* Returns:
* 0 when finished, otherwise current state
*
* Implementation:
* TODO:Update implementation description here -Matt
* Get the direction of the detected line. Multiply this by 15 and apply as a corrective heading
* to mfMoveToHeading.
*
* Improvements:
* Need to find a way to make it smoother.
*
*/
uint8_t dfFollowLine(uint8_t speed, struct Position *imuData)
{
	movingFlag = 1;
	enum {START, FIRST_CONTACT, ALIGN, FOLLOW, FINISH};
	int8_t lineDirection = dfGetLineDirection();
	uint16_t forwardProxSens = proxSensRead(MUX_PROXSENS_A);//Will use obstacle data structure once
															//implemented.
	static uint8_t lineFollowerState = FIRST_CONTACT;
	static float lineHeading = 0;
	
	switch(lineFollowerState)
	{
		//Starting state. Has value of 0 so when line following is fineshed will return 0
		case START:
			lineFollowerState = FIRST_CONTACT;
		break;
		
		//On first contact, drive forward slowly until line is detected on the middle two sensors.
		//Once that is the case, then we must be over the line properly, so move to the FOLLOW
		//state.
		case FIRST_CONTACT:
			//pioLedNumber(1);
			if(!lineDirection)
				lineFollowerState = FOLLOW;
			else 
				steerRobot(25, 0);
		break;
		
		//Given the position of the line sensors relative to the wheels on the underside of the 
		//robot, the robot must stop and rotate on the spot in order to accurately locate the 
		//position of the line relative to the robot. The robot will rotate clockwise or anti-
		//clockwise depending on the directional data from the line sensors, and if the line is
		//detected as being directly underneath the robot, then that heading is recorded and the
		//function switches to the FOLLOW state.
		case ALIGN:
			//pioLedNumber(2);
			if(lineDirection)
			{
				if(lineDirection < 0)
					rotateRobot(-10 + lineDirection*2);
				if(lineDirection > 0)
					rotateRobot(10 + lineDirection*2);
			}
			else 
			{
				lineHeading = imuData->imuYaw;
				lineFollowerState = FOLLOW;
			}
		break;
		
		//If the directional data from the line sensors suggests that we are centred over the line
		//then drive along the heading established (in the ALIGN state) as the heading of the line.
		//Otherwise, switch to the ALIGN state and re-centre over the line. If the forward prox
		//sensor has been triggered, then slow the robot down proportional to the value of the
		//sensor, and if maximum value is reached on the proximity sensor, then stop line following.
		case FOLLOW:
			//pioLedNumber(3);
			if(abs(lineDirection) < 2)
				mfMoveToHeading(lineHeading, speed - (forwardProxSens*speed/1023) + 5, imuData);
			else
				lineFollowerState = ALIGN;
			if(forwardProxSens >= PS_CLOSEST)
				lineFollowerState = FINISH;
		break;
		
		//If finished, reset the state machine for next time and return a 0.
		case FINISH:
			//pioLedNumber(7);
			lineFollowerState = START;
		break;
	}
	return lineFollowerState;	
}

/*
* Function:
* uint8_t dfScanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle,
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
* First up the function calls the mfRotateToHeading function. If that function has completed (ie the
* robot is facing in the heading we want) then an average white light brightness reading is taken
* from the light sensors. If the average brightness just read is greater than the last stored
* brightness value then update brightestHeading with the current heading and update brightestVal
* with the current avgBrightness. Once the robot has rotated 360 degrees, return 0 and reset the
* static variable to their starting states to indicate that the scan is complete. The heading
* left behind in brightest heading is the heading with the greatest amount of light.
*
* Improvements:
* TODO: more comments
*
*/
uint8_t dfScanBrightestLightSource(float *brightestHeading, uint16_t sweepAngle, 
								struct Position *imuData)
{
	const float ROTATE_STEP_SZ = 3;
	enum {FUNCTION_INIT, GOTO_START_POSITION, SWEEP, END};
	static uint8_t functionState = FUNCTION_INIT;
	static float startHeading;
	static float endHeading;
	static float sHeading;
	static uint32_t brightestVal;
	float rotateError;
	uint32_t avgBrightness = 0;
	
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
			if(!mfRotateToHeading(startHeading, imuData))
				functionState = SWEEP;						//In position, now perform sweep
			return 1;
		break;
		
		case SWEEP:
			rotateError = mfRotateToHeading(sHeading, imuData);
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

/*
* Function:
* float dfScanBrightestLightSourceProx(void)
*
* Uses all of the proximity sensors simultaneously to find the brightest source of light.
*
* Inputs:
* none
*
* Returns:
* Heading angle at which the brightest light source was detected.
*
* Implementation:
* The sensor array holds the values retrieved from each sensor. brightestVal holds the brightest
* detected value from any of the sensors. brightest sensor holds the index number of the sensor
* with the brightest ambient light detected.
* First the function enables ambient light detection on the proximity sensors. Then it reads the 
* light reading from each one into the sensor array. After this, proximity mode is re enabled on the
* proximity sensors. After that a for loop is used to see which sensor contained the brightes value.
* Finally, the number of the sensor multiplied by 60 (the angle in degrees that each sensor is
* apart) is returned from the function, indicating the direction of the brightest light source.
*
* Improvements:
* [NOT WORKING]: When proxAmbModeEnabled() is called, the IMU stops updating. I think its todo with
* the delay function that waits 50ms for data to be ready. Need to do more experimentation. -Matt
*
*/
float dfScanBrightestLightSourceProx(void)
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
	
	return nfWrapAngle(60*brightestSensor);
}