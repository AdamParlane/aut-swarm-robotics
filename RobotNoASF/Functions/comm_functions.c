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
#include "../Interfaces/twimux_interface.h"

#include "motion_functions.h"
#include "comm_functions.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void commGetNew(struct FrameInfo *frame, struct MessageInfo *message)
*
* Checks for new communications and handles the interpretation of them
*
* Inputs:
* pointer to frame_info struct and pointer to message_info struct
*
* Returns:
* none
*
* Implementation:
* First, checks if communication polling is enabled, and if the polling interval has elapsed.
* If so, checks if the XBee frame buffer is full indicating new data is ready to be read
* If so, interpret the new XBee API frame and use to fill the message buffer
* Then is the message buffer is full interpret the swarm message
*
*/
void commGetNew(RobotGlobalStructure *sys)
{
	static uint32_t nextPollTime = 0;
	struct FrameInfo commFrame;
	
	//If polling is enabled and the poll interval has elapsed
	if(sys->comms.pollEnabled && sys->timeStamp >= nextPollTime)
	{
		//Set the time at which comms will next be polled
		nextPollTime = sys->timeStamp + sys->comms.pollInterval;
		if(!xbeeFrameBufferInfoGetFull(&commFrame))			//Check for a received XBee Message
		{
			xbeeInterpretAPIFrame(commFrame);					//Interpret the received XBee Message
			if(!xbeeMessageBufferInfoGetFull(&sys->comms.messageData))//Check for a message from the swarm
				commInterpretSwarmMessage(sys);					//Interpret the message
		}
		
	}
}

/*
* Function:
* void commInterpretSwarmMessage(struct MessageInfo message, RobotGlobalStructure *sys)
*
* Interprets and acts on a received swarm messages
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global system data structure
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
void commInterpretSwarmMessage(RobotGlobalStructure *sys)
{
	//handles the incoming commands and sets the appropriate states / flags calls functions
	sys->flags.xbeeNewData = 1;
	if(sys->comms.messageData.command >= 0xE0) //test command range 0xE0-0xEF
		sys->states.mainf = M_TEST;
	else if (sys->comms.messageData.command == 0xD0)
	{
		mfStopRobot(sys);
	}
	//Manual command range 0xD1-0xD3
	else if(sys->comms.messageData.command >= 0xD1 && sys->comms.messageData.command <= 0xD3) 
		sys->states.mainf = M_MANUAL;
	else if (sys->comms.messageData.command == 0xD4)
		//move robot randomly
		mfRandomMovementGenerator();
	else if (sys->comms.messageData.command == 0xD7)
		sys->states.mainf = M_DOCKING;
	//0xD6 and D5 are also reserved for docking
	//at a later date for different methods if required
	else if (sys->comms.messageData.command == 0xD8)
		sys->flags.obaEnabled = 0;
	else if (sys->comms.messageData.command == 0xD9)
		sys->flags.obaEnabled = 1;
	else if (sys->comms.messageData.command == 0xDA)
		sys->states.mainf = M_LIGHT_FOLLOW;
	else if (sys->comms.messageData.command == 0xDB)
		sys->states.mainf = M_LINE_FOLLOW;
}

/*
* Function:
* char commTwi2SlaveRequest()
*
* Checks for a request from a master on TWI2 and acts on it (for the LCD interface)
*
* Inputs:
* none
*
* Returns:
* 0 on success
*
* Implementation:
* [[[WIP]]]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
char commTwi2SlaveRequest(void)
{
	enum TwiSlaveStates {IDLE, GET_CMD, SEND_DATA, FINISH};
	uint8_t twiSlaveState = IDLE;
	uint8_t comInProgress = 0;
	uint8_t command = 0;
	
	do 
	{
		switch (twiSlaveState)
		{
			case IDLE:
				if(twi2SlaveAccessPoll() & 0x01)	//If ReadMode
						twiSlaveState = GET_CMD;
				break;
			
			case GET_CMD:
				twi2SlaveRead(&command, &comInProgress);
				twiSlaveState = SEND_DATA;
				break;
				
			case SEND_DATA:
				switch(command)
				{
					case 1:				//Commands go here
						//twi2SlaveWrite(data, &comInProgress);
						break;
				}
				twiSlaveState = FINISH;
					break;
				
			case FINISH:
				twiSlaveState = IDLE;
				break;
		}
	} while (comInProgress);
	
	return 0;
}