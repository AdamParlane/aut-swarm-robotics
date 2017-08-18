/*
* main.c
*
* Author : et. al
* Created: Unknown
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* main c file for robot source code for BE (Hons) / BEng Tech Final year industrial project 2017
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* int main(void)
* 
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////
uint8_t SBtest, SBtest1;
uint16_t DBtest, DBtest1, DBtest2;
uint16_t battVoltage;					//Stores battery voltage on start up
extern struct Position robotPosition;	//Passed to docking functions and test functions

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* int main(void)
*
* Overview of robot code execution
*
* Inputs:
* none
*
* Returns:
* N/A
*
* Implementation:
* [[[[[WIP]]]]]
*
* Improvements:
* Yes.
*
*/
int main(void)
{
	robotSetup();
	battVoltage = fcBatteryVoltage();	//Add to your watch to keep an eye on the battery
	uint8_t testMode = 0x00;
	char chargeInfo;
	char error; //used for developement to log and watch errors - AP
	struct frame_info frame; //Xbee API frame
	struct message_info message; //Incoming message with XBee metadata removed
	struct transmitDataStructure transmitMessage; //struct to transmit to PC
	
	mainRobotState = DOCKING;
	
	while(1)
	{
		switch (mainRobotState)
		{
			case TEST:
				if(newDataFlag || streamIntervalFlag)//get the new test data
				{
					//get the new test data
					testMode = testManager(message, &transmitMessage, &robotPosition);
				}
				if(testMode == STOP_STREAMING)
					mainRobotState = IDLE;
				else if(testMode == SINGLE_SAMPLE)
				{
					mainRobotState = IDLE;
					SendXbeeAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, transmitMessage.Data, 
												transmitMessage.DataSize);  //Send the Message
				}
				else if(streamIntervalFlag && testMode == STREAM_DATA)
				{
					streamIntervalFlag = 0;
					SendXbeeAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, transmitMessage.Data,
												transmitMessage.DataSize);  //Send the Message
				}
			break;
			
			case TEST_ALL:
				//Not tested
				testAll(&transmitMessage);
			break;
			
			case MANUAL:
				if(newDataFlag)
					manualControl(message);
				chargeInfo = chargeDetector();
				if (chargeInfo == BATT_CHARGING)
				{
					mainRobotStatePrev = mainRobotState;
					mainRobotState = CHARGING;
				}
				else
					error = chargeInfo;
			break;
			
			case DOCKING:
				//if battery low or manual command set
				if(!dfDockRobot(&robotPosition))	//Execute docking procedure state machine
					mainRobotState = IDLE;			//If finished docking, go IDLE
			break;
				
			case FORMATION:
			//placeholder
			break;
			
			case CHARGING:
				if(!fdelay_ms(500))					//Blink LED in charge mode
					led1Tog;
				chargeInfo = chargeDetector();
				if(chargeInfo == BATT_CHARGING)
					break;
				else if(chargeInfo == BATT_CHARGED)
					mainRobotState = mainRobotStatePrev;
				else
					mainRobotState = MANUAL;
			break;
			
			case IDLE:
				//idle
				stopRobot();
			break;
		}
		
		//State independent function calls go here. Important regular system tasks like reading from
		//the communications port or from the navigation sensors.
		getNewCommunications(&frame, &message);
		nfRetrieveNavData();
	}
}