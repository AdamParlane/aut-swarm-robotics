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
* float mfRotateToHeading(float heading, RobotGlobalStructure *sys)
* float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys)
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, uint32_t distance,
*                                  RobotGlobalStructure *sys)
* float mfTrackLight(RobotGlobalStructure *sys)
* float mfTrackLightProx(RobotGlobalStructure *sys)
* char mfRandomMovementGenerator(void)
* void mfStopRobot(RobotGlobalStructure *sys)
*
*/
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "motion_functions.h"
#include "navigation_functions.h"
#include "../Interfaces/motor_driver.h"
#include "../Interfaces/light_sens_interface.h"
#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/timer_interface.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* float mfRotateToHeading(float heading, RobotGlobalStructure *sys)
*
* Will rotate the robot to face the given heading
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure so we can get IMU.yaw
*
* Returns:
* Will return 0 if the robot has settled at the desired heading, otherwise will return the signed
* error
*
* Implementation:
* pErr stores the proportional error value. It is declared as static as they need to retain their 
* values between function calls. motorSpeed stores the duty cycle (%) that will be sent to the 
* moveRobot() function. First, the function checks that heading is within the required range 
* (between -180 and 180 degrees). If it is out of range, the number is scaled down by nfWrapAngle.
* Next, the proportional (or signed) error value is calculated. It is simply the difference between 
* the desired heading and the current actual heading. The resulting error value is multiplied by a
* tuning constant and summed. The result of this is the corrective speed and direction to be applied
* to the motors. The absolute value of this is stored in motorSpeed and is then checked to make sure
* that its value is no greater than 100 (the maximum speed of the motors). After that, the
* proportional error is checked to see if it is with 0.5 degrees of the desired heading. If it is, 
* and delta yaw is less than 0.5dps, then this is deemed close enough to end seeking the desired
* heading. The static error variable is cleared, the robot is stopped and the function exits with a
* 0 value. If the prior conditions are not met, then the motorSpeed value is passed to the
* moveRobot function in order to correct the error.
*
* Improvements:
* the PID controller functionality might be able to be moved to its own function to be used by
* more than one motion function. Haven't implemented yet because not sure if this would create 
* problems later when there is the potential for to PID controllers to be running at once. If using
* static vars between calls they could crosstalk.
*
*/
float mfRotateToHeading(float heading, RobotGlobalStructure *sys)
{
	static float pErr;				//Proportional (signed) error
	int32_t motorSpeed;				//Stores motorSpeed calculated by PID sum
	
	//Make sure heading is in range (-180 to 180)
	heading = nfWrapAngle(heading);
		
	//Calculate proportional error values
	pErr = heading - sys->pos.facing;				//Signed Error
	
	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.	
	if(pErr > 180)
		pErr -= 360;
	if(pErr < -180)
		pErr += 360;
		
	//If motorSpeed ends up being out of range, then dial it back
	motorSpeed = RTH_KP*pErr;
	motorSpeed = capToRangeInt(motorSpeed, -100, 100);

	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(pErr) < 0.5) && (abs(sys->pos.IMU.gyroZ) < 0.5))	
	{
		mfStopRobot(sys);
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		return 0;
	} else {
		moveRobot(0, motorSpeed, 100);
		return pErr;	//If not, return pErr
	}
}

/*
* Function:
* float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys)
*
* Will rotate and then move the robot along the given heading at the given speed.
*
* Inputs:
* float heading:
*   The heading in degrees that we wish the robot to face (-180 < heading < 180)
* uint8_t speed:
*   Absolute speed as a percentage of maximum speed (0-100)
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure so we can get IMU.yaw
*
* Returns:
* Will return 0 if the robot moving along the desired heading, otherwise will return the signed
* error
*
* Implementation:
* Works similarly to mfRotateToHeading() except that robot will move along heading at the speed
* specified in the parameters. Direction of travel is controlled by closed loop system with the IMU.
* pErr stores the proportional error value. It is declared as static as they need to retain their
* values between function calls. motorSpeed stores the duty cycle (%) that will be sent to the
* moveRobot() function. First, the function checks that heading is within the required range
* (between -180 and 180 degrees). If it is out of range, the number is scaled down by nfWrapAngle.
* Next, the proportional (or signed) error value is calculated. It is simply the difference between
* the desired heading and the current actual heading. The resulting error value is multiplied by a
* tuning constant and summed. The result of this is the corrective speed and direction to be applied
* to the motors. The absolute value of this is stored in motorSpeed and is then checked to make sure
* that its value is no greater than 100 (the maximum speed of the motors). After that, the
* proportional error is checked to see if it is with 0.5 degrees of the desired heading. If it is,
* and delta yaw is less than 0.5dps, then this is deemed close enough to end seeking the desired
* heading. The static error variable is cleared, the robot is stopped and the function exits with a
* 0 value. If the prior conditions are not met, then the motorSpeed value is passed to the
* moveRobot function in order to correct the error.
*
*/
float mfMoveToHeading(float heading, uint8_t speed, RobotGlobalStructure *sys)
{
	static float pErr;				//Proportional (signed) error
	int32_t rotationSpeed = 0;		//Stores turn ratio calculated by PID sum
	
	//Make sure heading is in range (-180 to 180)
	heading = nfWrapAngle(heading);
	
	//Make sure speed is in range
	speed = capToRangeUint(speed, 0, 100);
		
	//Calculate proportional error values
	pErr = heading - sys->pos.facing;				//Signed Error
	
	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.
	if(pErr > 180)
		pErr -= 360;
	if(pErr < -180)
		pErr += 360;
	
	//If motorSpeed ends up being out of range, then dial it back
	rotationSpeed = MTH_KP*pErr;
	rotationSpeed = capToRangeInt(rotationSpeed, -100, 100);
	
	moveRobot(0, speed, rotationSpeed);
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//return 0 (ie robot is more or less on correct heading)
	if((abs(pErr) < 0.5) && (abs(sys->pos.IMU.gyroZ) < 0.5))
		return 0;
	else
		return pErr;	//If not, return pErr	
}

/*
* Function:
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, uint32_t distance,
*									 RobotGlobalStructure *sys);
*
* Will allow robot to move along the given heading a given distance.
*
* Inputs:
* float heading:
*   Heading to move along (-180 to 180 degrees)
* uint8_t speed:
*   Percentage of max speed to move at (0-100%)
* uint32_t distance:
*   Distance to travel before stopping.
* struct SystemStates *state
*   Pointer to the sys.states data structure
* RobotGlobalStructure *sys:
*   Pointer to the sys->pos. global structure.
*
* Returns:
* 0 when maneuver is complete, otherwise returns distance remaining before maneuver complete.
*
* Implementation:
* TODO:[[Currently not working as we don't seem able to retrieve data from the mouse yet.]]
* This function consists of a state machine. In the start state, the robot rotates to face the 
* correct heading. When it is facing the right way it moves to the move state. This is so the robot
* doesn't start measuring distance while the robot is rotating as this will through off the final
* distance traveled. In the move state, the robot moves along the given heading, summing the delta
* Y values from the mouse sensor to track the total distance traveled. When the traveled distance is
* found to be greater than the desired distance, then move to the stop state. In the stop state,
* the robot stops the motors and returns a 0 value. Also, the distance variable is reset to 0 and
* the function state reset to start, ready for the next call. If the function hasn't moved the
* desired distance then the function returns the distance remaining to be traveled.
*
*/
float mfMoveToHeadingByDistance(float heading, uint8_t speed, uint32_t distance, RobotGlobalStructure *sys)
{
	static float distanceTravelled = 0;
	
	switch(sys->states.moveHeadingDistance)
	{
		case MHD_START:
			if(!mfRotateToHeading(heading, sys))//Face the right direction
				sys->states.moveHeadingDistance = MHD_MOVING;
		break;
		
		case MHD_MOVING:
			mfMoveToHeading(heading, speed, sys);
			distanceTravelled += sys->pos.Optical.dy;//Once we are facing the right direction we can
													//start keeping track of the distance traveled.
			if(distanceTravelled > distance)		//If we have gone the distance
				sys->states.moveHeadingDistance = MHD_STOP;//Time to stop.
		break;
						
		case MHD_STOP:
			mfStopRobot(sys);							//Stop robot
			distanceTravelled = 0;					//Reset static distance variable
			sys->states.moveHeadingDistance = MHD_START;	//Reset function state.
			return 0;								//Indicate that maneuver is complete
		break;
	}
	return distance - distanceTravelled;	//If not complete return how far we have to go.
}

/*
* Function:
* float mfTrackLight(RobotGlobalStructure *sys)
*
* Robot will attempt to aim itself at a light source using colour sensors
*
* Inputs:
* RobotGlobalStructure *sys:
*   A pointer to the sys->pos. structure
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
* Possibility for integral run away if something goes wrong at the moment.
* Add speed parameter
*
*/
float mfTrackLight(RobotGlobalStructure *sys)
{
	sys->flags.obaMoving = 1;	
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
	dHeading = capToRangeInt(dHeading, -90, 90);
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(dHeading) < 0.5) && (abs(sys->pos.IMU.gyroZ) < 0.5))
	{
		mfStopRobot(sys);
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		iErr = 0;
		return 0;
	} else {
		mfMoveToHeading(sys->pos.facing + dHeading, 40, sys);
		return pErr;	//If not, return pErr
	}
}

/*
* Function:
* float mfTrackLightProx(RobotGlobalStructure *sys)
*
* Function to track a light source using the proximity sensors. [WIP]
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global sys->pos. data structure.
*
* Returns:
* 0 if facing light source, otherwise will return heading error value
*
* Implementation:
* Works similarly to mfRotateToHeading except that a difference between the light sensors
* is used for feedback. This generates a heading delta that can be applied to the current heading
* by the mfRotateToHeading() function which tries to correct the imbalance between the sensors.
*
* Improvements:
* TODO:Switching the prox sensors to ambient mode makes the IMU bug out. No solution yet.
*
*/
float mfTrackLightProx(RobotGlobalStructure *sys)
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
	dHeading = TLP_KP*pErr + TLP_KI*iErr;
	dHeading = capToRangeInt(dHeading, -90, 90);
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(dHeading) < 0.5) && (abs(sys->pos.IMU.gyroZ) < 0.5))
	{
		mfStopRobot(sys);
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
		//function
		iErr = 0;
		return 0;
	} else {
		mfRotateToHeading(sys->pos.facing + dHeading, sys);
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
* rand is seeded using srand and the sys->flags.tfStream
* sys->flags.tfStream is used becaue it has a high chance of being unique each time this is called
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
	int direction = rand() % 360;	//get random direction range: 0 - 360 degrees
	char speed = rand() % 100;		//get random speed:up to 100%
	char runTime = rand() % 5;		//get random delay time: up to 5 seconds
	moveRobot(direction, speed, 0);	//moveRobot at random speed and direction
	delay_ms(runTime * 1000);		//Delay for random milliseconds
	return 0;
}

void mfStopRobot(RobotGlobalStructure *sys)
{
	mdStopMotors();
	sys->flags.obaMoving = 0;
}