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
	uint16_t result = 0;
	obstacleDetected = 0;  
	for(uint8_t sensor = MUX_PROXSENS_A; sensor <= MUX_PROXSENS_B; sensor++)
	{
		result = proxSensRead(sensor);
		if (result > PS_IN_RANGE)
		{
			obstacleDetected = 1;
			return 0;
		}
	} 
	return 0;
}