/*
* obstacle_avoidance.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
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
* uint8_t scanProxSensors(uint8_t obstacleDetected)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "obstacle_avoidance.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t scanProxSensors(uint8_t obstacleDetected)
*
* Scans all proximity sensors. If any sensor has a value below the distance threshold then it
* returns a 1. Tells the program when it should switch to the obstacle avoidance state.
*
* Inputs:
* uint8_t obstacleDetected
*   A reference parameter that returns a 1 if anything is detected by the proximity sensors, 
*   otherwise returns a 0.
*
* Returns:
* Returns non zero on error when reading proximity sensors. 
* TODO: Define error codes for scanProxSensors()
*
* Implementation:
* Reads the value from each proximity sensor one by one using a for loop. If a proximity sensor is
* found to have a value that exceeds the 'in range' threshold (ie something is within 100mm of the
* robot) then set obstacleDetected to 1 and return from the function without error.
*
* Improvements:
* Add error code returns. (requires error returns for proxSensRead())
*
*/
uint8_t scanProxSensors(uint8_t *obstacleDetected)
{
	uint16_t sensorValue = 0;
	*obstacleDetected = 0;  
	for(uint16_t sensor = MUX_PROXSENS_A; sensor <= MUX_PROXSENS_B; sensor++)
	{
		sensorValue = proxSensRead(sensor);
		if (sensorValue > PS_IN_RANGE)
		{
			*obstacleDetected = 1;
			return 0;
		}
	} 
	return 0;
}

char randomMovementGenerator(void)
{
	srand(systemTimestamp(NULL)); 
	int direction = rand() % 360; //0 - 360 degrees
	char speed = rand() % 100; //upto 100%
	char runTime = rand() % 5; //up to 5 seconds
	moveRobot(direction, speed);
	delay_ms(runTime * 1000);
	
}
