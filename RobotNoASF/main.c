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
* void setup(void)
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define		batteryLow	1

//////////////[Global variables]////////////////////////////////////////////////////////////////////
uint8_t SBtest, SBtest1;
uint16_t DBtest, DBtest1, DBtest2;
struct Position robotPosition;
extern uint8_t checkImuFifo;
uint16_t battVoltage;


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
	//Comms
	struct frame_info frame;
	struct message_info message;
	//Optical
	robotPosition.x = 0;
	robotPosition.y = 0;
	robotPosition.imuYawOffset = 180;	//Ensures that whatever way the robot is facing when powered
										//on is 0 degrees heading.
	struct transmitDataStructure transmitMessage;
	robotState = IDLE;
	
	while(1)
	{
		switch (robotState)
		{
			case TEST:
			//should we move these into small functions to tidy main, tossing up whether it helps
			//at the same time I had this in a function and mark didnt like it
			if(newDataFlag || streamIntervalFlag)//get the new test data
			{
				testMode = testManager(message, &transmitMessage, &robotPosition);//get the new test data
			}
			if(testMode == STOP_STREAMING)
				robotState = IDLE;
			else if(testMode == SINGLE_SAMPLE)
			{
				robotState = IDLE;
				SendXbeeAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, transmitMessage.Data, transmitMessage.DataSize);  //Send the Message
			}
			else if(streamIntervalFlag && testMode == STREAM_DATA)
			{
				streamIntervalFlag = 0;
				SendXbeeAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, transmitMessage.Data, transmitMessage.DataSize);  //Send the Message
			}
			break;
			
			case TEST_ALL:
				//Place holder for state to test all peripherals at once
			break;
			
			case MANUAL:
			//should we move these into small functions to tidy main, tossing up whether it helps
				if(newDataFlag)
					manualControl(message);
				chargeInfo = chargeDetector();
				if (chargeInfo == BATT_CHARGING)
				{
					previousState = robotState;
					robotState = CHARGING;
				}
				else
					error = chargeInfo;
			break;
			
			case DOCKING:
				//if battery low or manual command set
				dockRobot(&robotPosition);	//Execute docking procedure state machine
			break;
			
			case OBSTACLE_AVOIDANCE:
			//Will execute code to guide robot around obstacles. Type of obstacle avoidance
			//performed will depend on the previous state of the robot, ie docking will want
			//to not move out of the way of other robots so as not to go out of alignment but
			//still stop when the dock has been reached.
			break;
			
			case FORMATION:
			//placeholder
			break;
			
			case CHARGING:
			//should we move these into small functions to tidy main, tossing up whether it helps
				ledOn1;
				chargeInfo = chargeDetector();
				if(chargeInfo == BATT_CHARGING)
					break;
				else if(chargeInfo == BATT_CHARGED)
				{
					ledOff1;
					robotState = previousState;
				}
				else
				{
					ledOff1;
					error = chargeInfo;
					robotState = MANUAL;
				}
			break;
			
			case IDLE:
				//idle
				stopRobot();
			break;
		}
		getNewCommunications(&frame, &message);
		//If ready, will read IMU data. Will be moved to a function when NAVIGATION module is added
		if(checkImuFifo)
		{
			imuReadFifo(&robotPosition);		//Read IMU's FIFO buffer
			checkImuFifo = 0;					//Reset interrupt flag
			imuGetEulerAngles(&robotPosition);	//Convert IMU quats to Euler angles
			getMouseXY(&robotPosition);			//Update mouse sensor data while at it
		}
		////////////////////////////////////////////////////////////////////////////////////////////
		//if(obstacleAvoidanceEnabledFlag)
			//obstacleAvoidanceManager();
	}
}