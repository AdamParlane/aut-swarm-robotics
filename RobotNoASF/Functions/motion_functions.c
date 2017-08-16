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
* float rotateToHeading(float heading, struct Position *imuData)
*
* Will rotate the robot to face the given heading
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* struct Position *imuData:
*   A pointer to the robotPosition structure so we can get imuYaw
*
* Returns:
* Will return 0 if the robot has settled at the desired heading, otherwise will return the signed
* error
*
* Implementation:
* pErr and iErr store the proportional and integral error values. They are declared as static as
* they need to retain their values between function calls. pErrOld and dErr are the old
* proportional error value and the delta or change in error value. motorSpeed stores the duty cycle
* (%) that will be sent to the rotateRobot() function.
* First, the function checks that heading is within the required range (between -180 and 180 
* degrees). If it is out of range, the number is scaled down by a while function until the heading
* is in range. This works because the heading is periodic ie, 540 deg = 180 deg and so on.
* Next, the proportional (or signed), integral and derivative (or delta) error values are 
* calculated. The resulting error values are multiplied by their respective constants and summed.
* The result of this is the corrective speed and direction to be applied to the motors. The
* absolute value of this is stored in motorSpeed and is then checked to make sure that its value
* is no greater than 100 (the maximum speed of the motors). After that, the proportional error is
* checked to see if it is with 1 degree of the desired heading. If it is, AND the motorSpeed is less
* than 15%, then this is deemed close enough to end seeking the desired heading. The static error
* variables are cleared, the robot is stopped and the function exits with a 0 value. If the prior
* conditions are not met, then pErr is checked for signedness (which determines which direction
* the motors should spin) and rotates the robot in the speed and direction necessary to correct the
* error.
*
* Improvements:
* the PID controller functionality might be able to be moved to its own function to be used by
* more than one motion function. Haven't implemented yet because not sure if this would create 
* problems later when there is the potential for to PID controllers to be running at once. If using
* static vars between calls they could crosstalk.
*
*/
float rotateToHeading(float heading, struct Position *imuData)
{
	static float pErr;				//Proportional (signed) error
	uint32_t motorSpeed;			//Stores motorSpeed calculated by PID sum
	
	//Make sure heading is in range
	heading = imuWrapAngle(heading);
		
	//Calculate proportional error values
	pErr = heading - imuData->imuYaw;				//Signed Error
	
	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.	
	if(pErr > 180)
		pErr -= 360;
	if(pErr < -180)
		pErr += 360;
		
	//If motorSpeed ends up being out of range, then dial it back
	motorSpeed = abs(RTH_KP*pErr);
	if(motorSpeed > 100)
		motorSpeed = 100;
		
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(pErr) < 0.5) && (abs(imuData->imuGyroZ) < 0.5))	
	{
		stopRobot();
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		return 0;
	} else {
		if(pErr > 0.0 )	//If heading is less than IMU heading then rotate clockwise to correct
			rotateRobot(CW, (unsigned char)motorSpeed);
		else			//Otherwise rotate anti-clockwise
			rotateRobot(CCW, (unsigned char)motorSpeed);
		return pErr;	//If not, return pErr
	}
}
