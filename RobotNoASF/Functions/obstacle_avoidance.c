/*
* obstacle_avoidance.c
*
* Author : Adam Parlane
* Created: 8/08/2017 3:08:20 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Functions that perform obstacle avoidance
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t scanProximity(void)
* void dodgeObstacle(RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/motor_driver.h"
#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/timer_interface.h"
#include "../Functions/motion_functions.h"
#include "obstacle_avoidance.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////

/*
* Function:
* char void scanProximity(void)
*
* Updates latest proximity values in an array
*
* Inputs:
* No Inputs
*
* Returns:
* No return values
*
* Implementation:
* Loops through proximity sensors using a for loop
* CW starting at A, F, E, D, C, B
* Reads each proximity sensor and fills the proximity array with their values
*
*/
void scanProximity(RobotGlobalStructure *sys)
{
	uint8_t index = 0;
	static uint32_t pollProxTime = 0;
	//If fast charge chip polling is enabled
	if(sys->sensors.prox.pollEnabled && pollProxTime <= sys->timeStamp)
	{
		//Store the time at which the charger status will next be polled
		pollProxTime = sys->timeStamp + sys->sensors.prox.pollInterval;
		for(uint16_t i = MUX_PROXSENS_A; i <= MUX_PROXSENS_B; i++)
		{
			proximity[index] = proxSensRead(i);
			index++;
			delay_ms(1);
		}
		if(!checkProximity())
		{
			//error so reset sensors
			proxSensInit();		//Initialise proximity sensors
			//delay_ms(50);
			sys->sensors.prox.errorCount ++;
			if(sys->sensors.prox.errorCount < 2)
			scanProximity(sys);	//rescan the sensors
		}
		else
		sys->sensors.prox.errorCount = 0;
	}
}

uint8_t checkProximity(void)
{
	char matches = 0; //counts how many prox sensors have the same value
	for(uint8_t i = 0; i < 5; i++)
	{
		if(proximity[i] == proximity[i+1])//compare each proximity sensor with the one after it
		matches ++; //count the number of matches
	}
	if(matches <= 4)//if they are not all the same
	return 1;//return 1 sensors are okay
	else
	return 0;//return 0 problem with sensors
}


/*
* Function:
* void dodgeObstacle(RobotGlobalStructure *sys)
* Will make the robot avoid obstacles while (hopefully remain on the current track
*
* Inputs:
* Aim, the direction robot was heading irrespective of obstacles
* Speed, the speed the robot was heading irrespective of obstacles
*
* Returns:
* No return values
*
* Implementation:
* [WIP] - AP
*
* Improvements:
* [WIP]
*
*/
uint8_t dodgeObstacle(RobotGlobalStructure *sys)
{
	scanProximity(sys);// updates proximity sensors
	signed int proxRange = 0, proxRangeHigh, proxRangeLow;// value assigned by index, can be 0, 60, 120, 180, 240, 300
	static char direction;
	static char firstLoop = 1;
	static char obs = 0;
	static uint8_t original;
	static float facing = 0;
	if(firstLoop)
	original = sys->pos.targetHeading % 60;
	uint8_t indexLeft, indexRight;//follows and leads index for the sake of checking proximity of nearby sensors
	//When to next send update
	static uint32_t updateNextTime = 0;
	if((sys->timeStamp > updateNextTime))
	{
		updateNextTime = sys->timeStamp + 100;
		for(uint8_t index = 0; index <= 5 ; index++)//0, 1, 2, 3, 4, 5
		{
			proxRange = index * 60; //convert angle to degrees
			proxRangeHigh = proxRange + 30;
			proxRangeLow = proxRange - 30;
			indexLeft = index + 1;
			indexRight = index - 1;
			//keep left and right indexes in range
			if(indexLeft > 5)
			indexLeft = 0;
			if(indexRight > 5)
			indexRight = 5;
			if(sys->pos.targetHeading > proxRangeLow && sys->pos.targetHeading <= proxRangeHigh) // if the prox is on the FACE we care about
			{
				if((proximity[index] > OBSTACLE_THRESHOLD) || (proximity[indexLeft] > 1000) || (proximity[indexRight] > 1000))
				{
					if(firstLoop) //choose a direction to dodge on first attempt
					{
						if((proximity[indexLeft] > proximity[indexRight]))
						{
							sys->pos.targetHeading +=90;
							direction = RIGHT;
						}
						else if ((proximity[indexLeft] < proximity[indexRight]))
						{
							sys->pos.targetHeading -= 90;
							direction = LEFT;
						}
						firstLoop = 0;
						obs = 1;
					}
					//stuck in a corner
					else if((proximity[index] > OBSTACLE_THRESHOLD) && (proximity[indexLeft] > OBSTACLE_THRESHOLD) && (proximity[indexRight] > OBSTACLE_THRESHOLD))
					{
						sys->pos.targetHeading += 120;
						original = sys->pos.targetHeading % 60;
						firstLoop = 1;
						//facing +=120;
					}
					//moving left but its getting worse
					else if ((direction == LEFT) && (proximity[indexLeft] > (proximity[indexRight] + 100)) && (proximity[indexLeft] > 600 || obs))
					{
						sys->pos.targetHeading += 90;
						if((abs(sys->pos.targetHeading) - 5) <= 5)
						sys->pos.targetHeading += 90;
						direction = RIGHT;
						obs = 0;
					}
					//moving right but its getting worse
					else if ((direction == RIGHT) && (proximity[indexRight] > (proximity[indexLeft] + 100)) && (proximity[indexRight] > 600 || obs))
					{
						sys->pos.targetHeading -= 90;
						if((abs(sys->pos.targetHeading) - 5) <= 5)
						sys->pos.targetHeading -= 90;
						direction = LEFT;
						obs = 0;
					}
				}
				else if (proximity[original] < (OBSTACLE_THRESHOLD-100)) //obstacle has been avoided
				{
					firstLoop = 1;
					obs = 0;
					delay_ms(250);
					return 0;
				}
			}
		}
		if(firstLoop)
		facing = sys->pos.facing;
		while(sys->pos.targetHeading >= 360)
		sys->pos.targetHeading -= 360;
		while(sys->pos.targetHeading < 0)
		sys->pos.targetHeading +=360;
	}

	mfAdvancedMove( sys->pos.targetHeading + facing, facing, sys->pos.targetSpeed, 100, sys);
	//moveRobot(sys->pos.targetHeading, sys->pos.targetSpeed, 0);
	//mfAdvancedMove( sys->pos.targetHeading, facing, sys->pos.targetSpeed, 100, sys);
	return 1;
}

void checkForObstacles(RobotGlobalStructure *sys)
{
	scanProximity(sys);// updates proximity sensors
	signed int proxRange = 0, proxRangeHigh, proxRangeLow;// value assigned by index, can be 0, 60, 120, 180, 240, 300
	uint8_t indexLeft, indexRight;//follows and leads index for the sake of checking proximity of nearby sensors
	for(uint8_t index = 0; index <= 5 ; index++)//0, 1, 2, 3, 4, 5
	{
		proxRange = index * 60; //convert angle to degrees
		proxRangeHigh = proxRange + 30;
		proxRangeLow = proxRange - 30;
		indexLeft = index + 1;
		indexRight = index - 1;
		//keep left and right indexes in range
		if(indexLeft > 5)
		indexLeft = 0;
		if(indexRight > 5)
		indexRight = 5;
		if(sys->pos.targetHeading > proxRangeLow && sys->pos.targetHeading <= proxRangeHigh) // if the prox is on the FACE we care about
		{
			if((proximity[index] > OBSTACLE_THRESHOLD) || (proximity[indexLeft] > 1000) || (proximity[indexRight] > 1000))
			{
				//if there is obstacles
				sys->states.mainfPrev = sys->states.mainf;
				sys->states.mainf = M_TEST_ALL;
			}
		}
	}
}

void dodgeObstacleByFacing(RobotGlobalStructure *sys)
{
	scanProximity(sys);// updates proximity sensors
	static float facing = 0;
	if(sys->flags.obaEnabled)
	{
		facing = sys->pos.facing;
		sys->flags.obaEnabled = 0;
	}
	if(abs(mfMoveToHeading(facing, sys->pos.targetSpeed, sys)) < 10)
	{
		if(proximity[0] > OBSTACLE_THRESHOLD)
		{
			if((proximity[5] >= proximity[1]) && (proximity[5] < OBSTACLE_THRESHOLD && proximity[1] < OBSTACLE_THRESHOLD))
				facing +=90;
			else if ((proximity[5] < proximity[1]) && (proximity[5] < OBSTACLE_THRESHOLD && proximity[1] < OBSTACLE_THRESHOLD))
				facing -= 90;
			else if(abs(proximity[5] - proximity[1]) < 50)	//side are relatively the same so stuck in corner
				facing += 180;
		}
		else if((abs(proximity[5] - proximity[1]) < 100) && (proximity[5] > OBSTACLE_THRESHOLD) && (proximity[1] > OBSTACLE_THRESHOLD))
			//side are relatively the same so in a "tunnel"
			facing += 0;
		else if(proximity[5] > OBSTACLE_THRESHOLD)
			facing -= 45;
		else if(proximity[1] > OBSTACLE_THRESHOLD)
			facing += 45;
	}
}

