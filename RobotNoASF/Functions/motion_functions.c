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
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "motion_functions.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* float mfRotateToHeading(float heading, struct Position *imuData)
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
float mfRotateToHeading(float heading, struct Position *imuData)
{
	static float pErr;				//Proportional (signed) error
	uint32_t motorSpeed;			//Stores motorSpeed calculated by PID sum
	
	//Make sure heading is in range (-180 to 180)
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
	motorSpeed = RTH_KP*pErr;
	if(motorSpeed > 100)
		motorSpeed = 100;
	if(motorSpeed < -100)
		motorSpeed = -100;
	
		
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(pErr) < 0.5) && (abs(imuData->imuGyroZ) < 0.5))	
	{
		stopRobot();
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		return 0;
	} else {
		rotateRobot(motorSpeed);
		return pErr;	//If not, return pErr
	}
}

float mfMoveForwardByDistance(uint16_t distance, struct Position *posData)
{
	enum {START, MOVING, STOP};
	static uint8_t movingState = START;
	uint16_t distanceTravelled = 0;
	
	switch(movingState)
	{
		case START:
			movingState = MOVING;
			moveRobot(0, 30);
		break;
		
		case MOVING:
			distanceTravelled += posData->opticalDY;
			if(distanceTravelled > distance)
				movingState = STOP;
		break;
						
		case STOP:
			stopRobot();
			distanceTravelled = 0;
			movingState = START;
			return 0;
		break;
	}
	return distance - distanceTravelled;
}

/*
* Function:
* float mfTrackLight(struct Position *imuData)
*
* Robot while attempt to aim itself at a light source
*
* Inputs:
* struct Position *imuData:
*   A pointer to the robotPosition structure
*
* Returns:
* 0 if equilibrium is reached, otherwise will return the proportional error value
*
* Implementation:
* Works similarly to mfRotateToHeading except that a normalised difference between the light sensors
* is used for feedback. This generates a heading delta that can be applied to the current heading
* by the mfRotateToHeading() function which tries to correct the imbalance between the sensors.
*
* Improvements:
* Possibility for integral run away if something goes wrong at the moment
*
*/
float mfTrackLight(struct Position *imuData)
{
	static float pErr;			//Proportional error
	static float iErr = 0;		//Integral error
	float dHeading;				//Delta heading to adjust by
	//Read light sensor values
	uint16_t leftSensor = lightSensRead(MUX_LIGHTSENS_L, LS_WHITE_REG);
	uint16_t rightSensor = lightSensRead(MUX_LIGHTSENS_R, LS_WHITE_REG);
	
	//Calculate errors
	//Proportional error is normalised by the average value of both sensors
	pErr = (float)(rightSensor - leftSensor)/((float)(leftSensor + rightSensor)/2.0);
	iErr += pErr;
			
	//If dHeading ends up being out of range, then dial it back
	dHeading = TL_KP*pErr + TL_KI*iErr;
	if(dHeading > 90)
		dHeading = 90;
	if(dHeading < -90)
		dHeading = -90;
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(dHeading) < 0.5) && (abs(imuData->imuGyroZ) < 0.5))
	{
		led1On;
		stopRobot();
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		iErr = 0;
		return 0;
	} else {
		mfRotateToHeading(imuData->imuYaw + dHeading, imuData);
		led1Off;
		return pErr;	//If not, return pErr
	}
}

float mfTrackLightProx(struct Position *imuData)
{
	static float pErr;			//Proportional error
	static float iErr = 0;		//Integral error
	float dHeading;				//Delta heading to adjust by
	
	//Enable Ambient light mode on the prox sensors
	proxAmbModeEnabled();
	
	//Read light sensor values
	uint16_t leftSensor = proxAmbRead(MUX_PROXSENS_F);
	uint16_t frontSensor = proxAmbRead(MUX_PROXSENS_A);
	uint16_t rightSensor = proxAmbRead(MUX_PROXSENS_B);
	
	//Revert to proximity mode
	proxModeEnabled();
	
	leftSensor = (leftSensor + frontSensor)/2;
	rightSensor = (rightSensor + frontSensor)/2;
	
	//Calculate errors
	//Proportional error is normalised by the average value of both sensors
	pErr = (float)(rightSensor - leftSensor)/((float)(leftSensor + rightSensor)/2.0);
	iErr += pErr;
	
	//If dHeading ends up being out of range, then dial it back
	dHeading = TL_KP*pErr + TL_KI*iErr;
	if(dHeading > 90)
		dHeading = 90;
	if(dHeading < -90)
		dHeading = -90;
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(dHeading) < 0.5) && (abs(imuData->imuGyroZ) < 0.5))
	{
		stopRobot();
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
		//function
		iErr = 0;
		return 0;
	} else {
		mfRotateToHeading(imuData->imuYaw + dHeading, imuData);
		return dHeading;	//If not, return pErr
	}
	return 1;
}

/*
* Function:
* char mfRandomMovementGenerator(void)
*
* Will make the robot move around psuedo-randomly
*
* Inputs:
* No Inputs
*
* Returns:
* No return values
*
* Implementation:
* rand is seeded using srand and the streamIntervalFlag
* streamIntervalFlag is used becaue it has a high chance of being unique each time this is called
* This is important because rand is puesdo-random and the same seed will produce the same set
* of 'random' numbers therefore must be seeded with a unique value
* PC applications use the time but this is not available
*
* Once seeded rand() is called with a modulo making the output within the desired range
* Desired range
*	Direction		0-360	degrees
*	Speed			0-100	percent
*	Delay			0-5		seconds
*
* Next these values are used to call the moveRobot function and allow it to run for Delay amount
* of seconds
*
*
* Improvements:
* Could use a return value to indicate success, or delay time
* 
* TODO: AP
* Need to change to use the timer interrupt NOT delay so the program doesnt hang for a few seconds
* each time the function is called
*
*/
char mfRandomMovementGenerator(void)
{
	srand(streamIntervalFlag);		//Seed rand() to give unique random numbers
	int direction = rand() % 360;	//get random direction range: 0 - 360 degrees
	char speed = rand() % 100;		//get random speed:up to 100%
	char runTime = rand() % 5;		//get random delay time: up to 5 seconds
	moveRobot(direction, speed);	//moveRobot at random speed and direction
	delay_ms(runTime * 1000);		//Delay for random milliseconds
	return 0;
}