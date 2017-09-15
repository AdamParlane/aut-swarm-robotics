/*
* comm_functions.c
*
* Author : Adam Parlane/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 15/09/2017 11:17:34 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Retrieves and handles messages from the Xbee network
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void commGetNew()
* void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/xbee_driver.h"

#include "motion_functions.h"
#include "comm_functions.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void commGetNew(struct FrameInfo *frame, struct MessageInfo *message)
*
* Checks for new communications and handlles the interpretation of them
*
* Inputs:
* pointer to frame_info struct and pointer to message_info struct
*
* Returns:
* none
*
* Implementation:
* Checks if the XBee frame buffer is full indicating new data is ready to be read
* If so, interpret the new XBee API frame and use to fill the message buffer
* Then is the message buffer is full interpret the swarm message
*
* Improvement:
* I think that this should be in a higher level function file.
*
*/
void commGetNew(RobotGlobalStructure *sys)
{
	if(xbeeFrameBufferInfoGetFull(&frame) == 0)			//Check for a received XBee Message
	{
		xbeeInterpretAPIFrame(frame);					//Interpret the received XBee Message
		if(!xbeeMessageBufferInfoGetFull(&message))		//Check for a message from the swarm
			commInterpretSwarmMessage(message, sys);	//Interpret the message
	}
}

/*
* Function:
* void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
*
* Interprets and acts on a received swarm messages
*
* Inputs:
* struct MessageInfo message:
*   TODO: Adam: input description
*
* Returns:
* none
*
* Implementation:
* TODO: Adam: implementation description
*
* Improvements:
* Improvements: is it worth defining all these codes, im thinking no << There should be NO magic
* numbers.
*
*/
void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
{
	//handles the incoming commands and sets the appropriate states / flags calls functions
	sys->flags.xbeeNewData = 1;
	if(message.command >= 0xE0) //test command range 0xE0-0xEF
		sys->states.mainf = M_TEST;
	else if (message.command == 0xD0)
	{
		mfStopRobot(sys);
	}
	else if(message.command >= 0xD1 && message.command <= 0xD3) //Manual command range 0xD1-0xD3
		sys->states.mainf = M_MANUAL;
	else if (message.command == 0xD4)
		//move robot randomly
		mfRandomMovementGenerator();
	else if (message.command == 0xD7)
		sys->states.mainf = M_DOCKING;
	//0xD6 and D5 are also reserved for docking
	//at a later date for different methods if required
	else if (message.command == 0xD8)
		sys->flags.obaEnabled = 0;
	else if (message.command == 0xD9)
		sys->flags.obaEnabled = 1;
	else if (message.command == 0xDA)
		sys->states.mainf = M_LIGHT_FOLLOW;
	else if (message.command == 0xDB)
		sys->states.mainf = M_LINE_FOLLOW;
}