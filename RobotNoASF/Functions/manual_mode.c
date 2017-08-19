/*
* manual_mode.c
*
* Author : Adam Parlane
* Created: 1/8/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains the definitions and functions required for manual mode
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* manualControl(message)
*
*/

#include "manual_mode.h"

/*
* Function:
* void manualControl(struct message_info message)
*
* Runs the manual movement controls from the GUI
* These are:
*			move straight (N, NE, E, SE, S, SW, W, NW)
*			rotate (CW, CCW)
*			stop
* Movements also have an assigned speed
*
* Inputs:
* struct message_info message
*			XBee message data
*
* Returns:
* none
*
* Implementation:
* uses convertData(message, receivedTestData);
* to fetch the received data information (speed and direction)
*
*
* Improvements:
* TODO: Adam Comment this -AP
*
*/
void manualControl(struct message_info tmessage)
{
	static uint8_t receivedTestData[5];
	newDataFlag = 0;
	uint16_t straightDirection;
	convertData(tmessage, receivedTestData);
	straightDirection = (receivedTestData[0] << 8) + (receivedTestData[1]);
	if(tmessage.command == MANUAL_STRAIGHT)
	{
		moveRobot(straightDirection, receivedTestData[2]);
		aim = straightDirection;
		aimSpeed = receivedTestData[2];
	}
	else if(tmessage.command == MANUAL_STOP)
	{
		//stopRobot();
		mainRobotState = IDLE;
		
	}
	else if(tmessage.command == CW)
	{
		//CW is reverse so invert speed
		signed char speed = -1*receivedTestData[0];
		rotateRobot(speed);
	}
	else if(tmessage.command == CCW)
	{
		//CCW is forward so no need to invert speed
		rotateRobot(receivedTestData[0]);
	}
}