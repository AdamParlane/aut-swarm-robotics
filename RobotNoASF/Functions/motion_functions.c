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
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, float distance,
*                                  RobotGlobalStructure *sys)
* float mfTrackLight(RobotGlobalStructure *sys)
* float mfTrackLightProx(RobotGlobalStructure *sys)
* char mfRandomMovementGenerator(void)
* void mfStopRobot(RobotGlobalStructure *sys)
* char mfAdvancedMove(float heading, float facing, uint8_t speed,
* 							uint8_t maxTurnRatio, RobotGlobalStructure *sys)
* int32_t mfMoveToPosition(int32_t x, int32_t y, uint8_t speed, float facing,
*							uint8_t maxTurnRatio, RobotGlobalStructure *sys)
*
*/
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include <math.h>		//For round()

#include "../Interfaces/motor_driver.h"
#include "../Interfaces/light_sens_interface.h"
#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/timer_interface.h"

#include "navigation_functions.h"
#include "motion_functions.h"

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
	if((abs(pErr) < MF_FACING_ERR) && (abs(sys->pos.IMU.gyroZ) < MF_DELTA_GYRO_ERR))	
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
	if((abs(pErr) < MF_FACING_ERR) && (abs(sys->pos.IMU.gyroZ) < MF_DELTA_GYRO_ERR))
		return 0;
	else
		return pErr;	//If not, return pErr	
}

/*
* Function:
* float mfMoveToHeadingByDistance(float heading, uint8_t speed, float distance,
*									 RobotGlobalStructure *sys);
*
* Will allow robot to move along the given heading a given distance.
*
* Inputs:
* float heading:
*   Heading to move along (-180 to 180 degrees)
* uint8_t speed:
*   Percentage of max speed to move at (0-100%)
* float distance:
*   Distance to travel before stopping.
* struct SystemStatesGroup *state
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
float mfMoveToHeadingByDistance(float heading, uint8_t speed, float distance, 
									RobotGlobalStructure *sys)
{
	static float distanceTravelled = 0;
	
	switch(sys->states.moveHeadingDistance)
	{
		case MHD_START:
			if(!mfRotateToHeading(heading, sys) && sys->pos.dy == 0)//Face the right direction
				sys->states.moveHeadingDistance = MHD_MOVING;
			break;
		
		case MHD_MOVING:
			//Once we are facing the right direction we can start keeping track of the distance 
			//traveled.
			//speed = capToRangeUint(round((distance - distanceTravelled)*MTHD_KP + 25), 0, 100);
			mfMoveToHeading(heading, speed, sys);
			distanceTravelled += sqrt(sys->pos.dy*sys->pos.dy + sys->pos.dx*sys->pos.dx);
			if(distanceTravelled > distance)		//If we have gone the distance
				sys->states.moveHeadingDistance = MHD_STOP;//Time to stop.
			break;
						
		case MHD_STOP:
			mfStopRobot(sys);						//Stop robot
			distanceTravelled = 0;					//Reset static distance variable
			sys->states.moveHeadingDistance = MHD_START;//Reset function state.
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
float mfTrackLight(uint8_t speed, RobotGlobalStructure *sys)
{
	sys->flags.obaMoving = 1;	
	static float pErr;			//Proportional error
	static float iErr = 0;		//Integral error
	float dHeading;				//Delta heading to adjust by
	
	//Calculate errors
	//Proportional error is normalised by the average value of both sensors
	pErr = (float)(sys->sensors.colour.right.green - sys->sensors.colour.left.green)/
			((float)(sys->sensors.colour.left.green + sys->sensors.colour.right.green)/2.0);
	iErr += pErr;
			
	//If dHeading ends up being out of range, then dial it back
	dHeading = TL_KP*pErr + TL_KI*iErr;
	dHeading = capToRangeInt(dHeading, -60, 60);
	
	mfMoveToHeading(sys->pos.facing + dHeading, 60, sys);
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//stop
	if((abs(dHeading) < MF_FACING_ERR) && (abs(sys->pos.IMU.gyroZ) < MF_DELTA_GYRO_ERR))
	{
		pErr = 0;			//Clear the static vars so they don't interfere next time we call this
							//function
		iErr = 0;
		return 0;
	} else {
		mfMoveToHeading(sys->pos.facing + dHeading, speed, sys);
		return pErr;		//If not, return pErr
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
	if((abs(dHeading) < MF_FACING_ERR) && (abs(sys->pos.IMU.gyroZ) < MF_DELTA_GYRO_ERR))
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
char mfRandomMovementGenerator(RobotGlobalStructure *sys)
{
	static char restart = 1;
	int direction = rand() % 360;	//get random direction range: 0 - 360 degrees
	char speed = rand() % 100;		//get random speed:up to 100%
	//char runTime = rand() % 5;		//get random delay time: up to 5 seconds
	if(restart)
	{
		sys->pos.targetHeading = direction;
		moveRobot(direction, speed, 0);	//moveRobot at random speed and direction
	}
	if(!fdelay_ms(5000))				//Delay for 5 secondss
		restart = 1;
	else
		restart = 0;
	return 0;
}

/*
* Function:
* void mfStopRobot(RobotGlobalStructure *sys)
*
* Stops the robot (High level function to avoid direct access to the motor_driver from other 
* high level functions
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure
*
* Returns:
* none
*
* Implementation:
*   Stops all the motors and sets the obstacle avoidances system moving flag to 0
*
*/
void mfStopRobot(RobotGlobalStructure *sys)
{
	mdStopMotors();
	sys->flags.obaMoving = 0;
}

/*
* Function:
* char mfAdvancedMove(float heading, float facing, uint8_t speed,
* 							uint8_t maxTurnRatio, RobotGlobalStructure *sys)
*
* Will move the robot along a heading, and also rotate the robot to face the given facing, 
* using closed loop control from the mouse and IMU to achieve it.
*
* Inputs:
* float heading:
*   The absolute (arena) heading that the robot should travel along
* float facing:
*   The absolute (arena) facing that the robot should face
* uint8_t speed:
*   The maximum motor speed (0-100%)
* uint8_t maxTurnRatio:
*   The percentage of rotational speed to be applied to the motors (ie how fast the robot will
*   rotate towards the desired facing). 0% means the robot will not rotate at all. %100 means that
*   The robot will only rotate on the spot until the desired facing is achieve before setting off
*   on the desired heading. Anything in between will have the robot gradually rotate while
*   travelling along the desired heading.
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* 0 when the robot has achieved the desired facing, otherwise the proportional error between the
* desired facing and the current facing of the robot.
*
* Implementation:
* See mfMoveToHeading() above as this function is based on that. The difference is that this
* two control mechanisms, one from the IMU and one from the optical sensor for the facing control
* and heading control respectively.
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
char mfAdvancedMove(float heading, float facing, uint8_t speed,
						uint8_t maxTurnRatio, RobotGlobalStructure *sys)
{	
	//float pErrH;					//Proportional (signed) heading error
	float pErrF;					//Proportional (signed) facing error
	float facingCorrection = 0;		//Stores turn ratio calculated by PID sum
	//float headingCorrection = 0;	//Stores heading correction calculated by PID sum
	
	//Make sure heading and facing is in range (-180 to 180)
	heading = nfWrapAngle(heading);
	facing = nfWrapAngle(facing);
	
	//Make sure speed and maxTurnRatio is in range
	speed = capToRangeUint(speed, 0, 100);
	maxTurnRatio = capToRangeUint(maxTurnRatio, 0, 100);
	
	//Calculate proportional error values
	//pErrH = heading - sys->pos.heading;				//Signed Error (heading)
	pErrF = facing - sys->pos.facing;				//Signed Error (facing)
	
	//Force the P controller to always take the shortest path to the destination.
	//For example if the robot was currently facing at -120 degrees and the target was 130 degrees,
	//instead of going right around from -120 to 130, it will go to -180 and down to 130.
	if(pErrF > 180)
		pErrF -= 360;
	if(pErrF < -180)
		pErrF += 360;

	//if(pErrH > 180 || pErrH < -180)
	//	pErrH *= -1;
	
	//Calculate the correction figures
	//headingCorrection = AMH_KP*pErrH;
	facingCorrection = AMF_KP*pErrF;
	//If the correction values end up being out of range, then dial them back
	facingCorrection = capToRangeFlt(facingCorrection, -maxTurnRatio, maxTurnRatio);
	//headingCorrection = capToRangeFlt(headingCorrection, -180, 180);
	
	//Get the robot moving to correct the errors.
	moveRobot((heading - sys->pos.facing) , speed, facingCorrection);
	
	//If error is less than 0.5 deg and delta yaw is less than 0.5 degrees per second then we can
	//return 0 (ie robot is more or less on correct heading)
	if((abs(sys->pos.IMU.gyroZ) < MF_DELTA_GYRO_ERR) && (abs(pErrF) < MF_FACING_ERR))
		return 0;
	else
		return pErrF;	//If not, return pErrF
}

/*
* Function:
* int32_t mfMoveToPosition(int32_t x, int32_t y, uint8_t speed, float facing,
*							uint8_t maxTurnRatio, RobotGlobalStructure *sys)
*
* Will move the robot to the given position in the arena
*
* Inputs:
* int32_t x:
*   The absolute (arena) x position (mm) that the robot should drive to
* int32_t y:
*   The absolute (arena) y position (mm) that the robot should drive to
* uint8_t speed:
*   The maximum motor speed (0-100%)
* float facing:
*   The absolute (arena) facing that the robot should face
* uint8_t maxTurnRatio:
*   The percentage of rotational speed to be applied to the motors (ie how fast the robot will
*   rotate towards the desired facing). 0% means the robot will not rotate at all. %100 means that
*   The robot will only rotate on the spot until the desired facing is achieve before setting off
*   on the desired heading. Anything in between will have the robot gradually rotate while
*   travelling along the desired heading.
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* current state
*
* Implementation:
* [[[[WIP]]]]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
int32_t mfMoveToPosition(int32_t x, int32_t y, uint8_t speed, float facing, 
						uint8_t maxTurnRatio, RobotGlobalStructure *sys)
{
	enum {START, CALC_HEADING, MOVE_TO_POS, FINISHED};
	static uint8_t state = START;
	static float currentHeading = 0;
	//static uint32_t distTravelled = 0;
	
	switch(state)
	{
		case START:
			//initialDistance = sqrt((x - sys->pos.x)*(x - sys->pos.x) + 
			//						(y - sys->pos.y)*(y - sys->pos.y));
			state = CALC_HEADING;
			break;
		
		case CALC_HEADING:
			//distTravelled = 0;
			currentHeading = atan2((x - sys->pos.x), (y - sys->pos.y))*180/M_PI;
			state = MOVE_TO_POS;
			break;
			
		case MOVE_TO_POS:
			//if(!fdelay_ms(1000))
			//	state = CALC_HEADING;
			
			currentHeading = atan2((x - sys->pos.x), (y - sys->pos.y))*180/M_PI;
			//distTravelled += sqrt(sys->pos.dx*sys->pos.dx + sys->pos.dy*sys->pos.dy);
			
			mfAdvancedMove(currentHeading, facing, speed, maxTurnRatio, sys);
			if((abs(x - sys->pos.x) < 50) && (abs(y - sys->pos.y) < 50))
				state = FINISHED;
			break;
			
		case FINISHED:
			mfStopRobot(sys);
			state = START;
			break;
	}
	
	return state;
}