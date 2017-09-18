/*
* obstacle_avoidance.c
*
* Author : Adam Parlane
* Created: 8/08/2017 3:08:20 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions that perform obstacle avoidance
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t scanProximity(void)
* void dodgeObstacle(struct Position *robotPosition)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
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
void scanProximity(void)
{
	uint8_t index = 0;
	for(uint16_t i = MUX_PROXSENS_A; i <= MUX_PROXSENS_B; i++)
	{
		proximity[index] = proxSensRead(i);
		index++;
	}		
}


/*
* Function:
* void dodgeObstacle(struct Position *robotPosition)
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
void dodgeObstacle(struct Position *robotPosition)
{
	scanProximity();	//update proximity readings
	uint16_t proxRange = 0;
	static uint8_t save = 0;// 0 for not set, 1 for moving left, 2 for moving right
	static uint8_t firstTime = 1; // 1 on firstTime, 0 on other times
	static signed int movingDirection = 0;
	if(firstTime)
		movingDirection = robotPosition -> targetHeading;
	uint8_t indexLeft, indexRight;
	for(uint8_t index = 0; index < 6 ; index++)
	{
		proxRange = index * 60; //convert angle to degrees
		indexLeft = index + 1;
		indexRight = index - 1;
		//keep left and right indexes in range
		if(indexLeft > 5)
			indexLeft = 0;
		if(indexRight > 5)
			indexRight = 5;
		if((robotPosition->targetHeading > (proxRange - 30) && 
		(robotPosition->targetHeading <= (proxRange + 30))) || ((index == 0) && 
		(robotPosition->targetHeading > 330) && (robotPosition->targetHeading <= 30)))
		{
			if((proximity[index] > OBSTACLE_THRESHOLD) || (proximity[indexLeft] > 800) || (proximity[indexRight] > 800))
			{
				if(firstTime)// || (proximity[indexLeft] > 1000) || (proximity[indexRight] > 1000))
				{
					if((proximity[indexLeft] > proximity[indexRight]) && (proximity[indexLeft] < 800) && (proximity[indexRight] < 800))
					{
						//robotPosition ->targetHeading +=90;
						save = 2;
						firstTime = 0;
						movingDirection =+ 90;
						//moveRobot(movingDirection, robotPosition -> targetSpeed);//move right
						
					}
					else if ((proximity[indexLeft] < proximity[indexRight]) && (proximity[indexLeft] < 800) && (proximity[indexRight] < 800))
					{
						//robotPosition -> targetHeading -= 90;
						firstTime = 0;
						save = 1;
						movingDirection -= 90;
						//moveRobot(movingDirection, robotPosition -> targetSpeed);//move left
					}				
				}
				//else if(!firstTime)
				//{
					//if(save == 2)//already moving right
					//{
						////robotposition ->targetheading +=90;
						////save = 2;
						//moveRobot(movingDirection, robotPosition -> targetSpeed);//move right
					//}
					//if (save == 1)//already moving left
					//{
						////robotposition -> targetheading -= 90;
						//moveRobot(movingDirection, robotPosition -> targetSpeed);//move left
					//}
				//}
				if(proximity[indexLeft] > 800)
				{
					movingDirection += 90;
				}
				if(proximity[indexRight] > 800)
				{
					movingDirection -=90;
				}
				moveRobot(movingDirection, robotPosition -> targetSpeed);
			}
			else
			{ 
				moveRobot(robotPosition->targetHeading, robotPosition -> targetSpeed);
				firstTime = 1;
				save = 0;
			}
		}	
	}	
	if(movingDirection > 360)
		movingDirection -= 360;	
	if(movingDirection < 0)
		movingDirection += 360;
}

void obstacleAvoidance(signed int aim)
{
	scanProximity();// updates proximity sensors
	signed int proxRange = 0, proxRangeHigh, proxRangeLow;// value assigned by index, can be 0, 60, 120, 180, 240, 300
	static char obstacleFlag = 0;
	static char direction;
	static signed int newAim = 0;
	static char firstLoop = 1;
	if(firstLoop)//on the first run through assign aim to newAim
	{
		newAim = aim;
		//firstLoop = 0;
	}
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
		if(newAim > proxRangeLow && newAim <= proxRangeHigh) // if the prox is on the FACE we care about
		{
			if((proximity[index] > OBSTACLE_THRESHOLD) || (proximity[indexLeft] > 1000) || (proximity[indexRight] > 1000))
			{
				if(firstLoop)
				{
					if((proximity[indexLeft] > proximity[indexRight]))
					{
						aim +=90;
						moveRobot(aim, 50);//move right 
						direction = RIGHT;
					}
					else if ((proximity[indexLeft] < proximity[indexRight]))
					{
						aim -= 90;
						moveRobot(aim, 50);//move left
						direction = LEFT;
					}
					firstLoop = 0;
				}
				else if ((direction == LEFT) && (proximity[indexLeft] > (proximity[indexRight] + 100)) && proximity[indexLeft] > 600)
				{
					aim += 90;
					moveRobot(aim, 50);
					direction = RIGHT;
				}
				else if ((direction == RIGHT) && (proximity[indexRight] > (proximity[indexLeft] + 100)) && proximity[indexRight] > 600)
				{
					aim -= 90;
					moveRobot(aim, 50);
					direction = LEFT;
				}
				else if((proximity[index] > OBSTACLE_THRESHOLD) && (proximity[indexLeft] > 800) && (proximity[indexRight] > 800))
				{
					aim -= 120;
					moveRobot(aim, 50);
					//delay_ms(1000);
				}
			}
			else
			{
				moveRobot(aim, 50);	
				firstLoop = 1;
			}

		}
			//if((proximity[index] > OBSTACLE_THRESHOLD) && !obstacleFlag)//obstacle straight(ish) ahead
			//{
				////if(proximity[indexLeft] > 800 && proximity[indexRight] > 800)
				////{
					//////if robot is surrounded by obstacles
					////newAim = aim - 180;
					////moveRobot(newAim, 50);
					////obstacleFlag = 1;
				////}
				//if(proximity[indexLeft] > proximity[indexRight])//obstacle to left
				//{
					////move RIGHT away from obstacle
					//newAim = aim + 90;
					//moveRobot(newAim, 50);//move right
					//obstacleFlag = 1;				
				//}
				//if(proximity[indexLeft] < proximity[indexRight])//obstacle to right
				//{
					////move LEFT away from obstacle
					//newAim = aim - 90;
					//moveRobot(newAim, 50);//move left
					//obstacleFlag = 1;
				//}
			//}
			//else if(proximity[indexLeft] > 800)
			//{
				////Something close on left side
				////move RIGHT away from obstacle
				//newAim = aim + 90;
				//moveRobot(newAim, 50);//move right
				//obstacleFlag = 1;				
			//}
			//else if(proximity[indexRight] > 800)
			//{
				////Something close on right side
				////move LEFT away from obstacle
				//newAim = aim - 90;
				//moveRobot(newAim, 50);//move left
				//obstacleFlag = 1;
			//}
			//else if((proximity[index] < OBSTACLE_THRESHOLD) && obstacleFlag)
			//{
				////timer to ensure robot passes edge of object not just field of view of sensor
				//delay_ms(300); //has potential will need to change to non blocking
				//obstacleFlag = 0;
			//}
			//else if((proximity[index] < OBSTACLE_THRESHOLD) && !obstacleFlag)
			//{
				//moveRobot(aim, 50);//continue straight
				//firstLoop = 1;
			//}		
		//}
		//else if(aim == proxRangeHigh) //aiming on a corner
		//{
			////different strategy involving averaging the 2 adjacent prox sensors
			////to work without this, make if(aim > proxRangeLow && aim <= proxRangeHigh)
		//}
	}
	//if(newAim > 360)
		//newAim -= 360;
	//if(newAim < 0)
		//newAim += 360;	
}


//signed int dodgeObstacle(signed int aim, char speed)
//{
	//scanProximity();	//update proximity readings
	//static int oAim = 0;
	//uint16_t proxRange = 0;
	//uint8_t indexLeft, indexRight;
	//for(uint8_t index = 0; index < 6 ; index++)
	//{
		//proxRange = index * 60; //convert angle to degrees
		//indexLeft = index + 1;
		//indexRight = index - 1;
		////keep left and right indexes in range
		//if(indexLeft > 5)
		//indexLeft = 0;
		//if(indexRight > 5)
		//indexRight = 5;
		//if((aim >= (proxRange - 30) && (aim < (proxRange + 30))) || ((index == 0) && (aim > 330) && (aim < 30)))
		//{
			//if((proximity[index] > OBSTACLE_THRESHOLD) || (proximity[indexLeft] > 1000) || (proximity[indexRight] > 1000))
			//{
				//if(proximity[indexLeft] > proximity[indexRight])
				//{
					//aim +=90;
					//moveRobot(aim, speed);//moveLeft
				//}
				//else if (proximity[indexLeft] < (proximity[indexRight]-50))
				//{
					//aim -= 90;
					//moveRobot(aim, speed);//move right
				//}
			//}
			//else
			//moveRobot(aim, speed);
		//}
	//}
	//return aim;
//}