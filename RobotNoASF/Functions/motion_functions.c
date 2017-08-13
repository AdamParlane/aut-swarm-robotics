/*
* motion_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/08/2017 12:41:58 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* High level motion functions for moving the robot (ie PID controlled navigation)
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
#include "motion_functions.h"

///////////////Global variables/////////////////////////////////////////////////////////////////////
extern struct Position robotPosition;

///////////////Functions////////////////////////////////////////////////////////////////////////////

/*
* Function:
* [function declaration]
*
* [brief purpose of function]
*
* Inputs:
* [input arguments and any relevant explanation]
*
* Returns:
* [return values and any relevant explanation]
*
* Implementation:
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
float rotateToHeading(float heading, struct Position *imuData)
{
	static float pErr, pErrOld, iErr, dErr;
	uint32_t motorSpeed;
	//Make sure heading is in range
	while(heading > 180.0)
		heading -= 360.0;
	while(heading <= -180.0)
		heading += 360.0;
		
	//Calculate proportional, integral and differential error values
	pErrOld = pErr;
	pErr = heading - imuData->imuYaw;
	iErr += pErr*imuData->imuDeltaTime*0.001;
	dErr = (pErr - pErrOld)/(imuData->imuDeltaTime*0.001);
	
	motorSpeed = abs(RTH_KP*pErr + RTH_KI*iErr + RTH_KD*dErr); 
	if(motorSpeed > 100)
		motorSpeed = 100;

	if((abs(pErr) < 3) && (motorSpeed < 13))	//If error is less than 3 deg and motorSpeed is less
												//than 5% then we must be pretty close so stop and
												//return a 0. Prevents oscillation about target
	{
		stopRobot();
		iErr = 0;
		pErr = 0;
		dErr = 0;
		return 0;
	} else {
	
		if(pErr > 0.0 )
			rotateRobot(CW, (unsigned char)motorSpeed);
		else
			rotateRobot(CCW, (unsigned char)motorSpeed);
	

		return pErr;						//If not, return pErr
	}
}