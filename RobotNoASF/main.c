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
extern struct message_info message;
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
	robotSetup(); //Set up the system and peripherals
	battVoltage = fcBatteryVoltage();	//Add to your watch to keep an eye on the battery
	mainRobotState = IDLE;
	while(1)
	{
		switch (mainRobotState)
		{
			case TEST: //System Test Mode
			//Entered when test command received from PC
				testManager(message, &robotPosition);
				break;
			
			case MANUAL: //User controlled mode
			//Entered when manual movement command received from PC
				if(newDataFlag) //if there is new data
					manualControl(message, &robotPosition);
				chargeInfo = chargeDetector();
				break;
			
			case DOCKING:
			//if battery low or manual docking command sent from PC
				if(!dfDockRobot(&robotPosition))	//Execute docking procedure state machine
					mainRobotState = IDLE;			//If finished docking, go IDLE
				break;
			
			case LINE_FOLLOW:
			//Entered when line follow command received from PC
				if(!dfFollowLine(35, &robotPosition))//Line follower will return 0 when complete
					mainRobotState = IDLE;
				break;
					
			case LIGHT_FOLLOW:
			//Entered when light follow command received from PC
				mfTrackLight(&robotPosition);
				break;
				
			case FORMATION:
			//placeholder
				break;
			
			case CHARGING:
				chargeInfo = chargeDetector();
				break;

			case IDLE:					
				stopRobot();
				if(!fdelay_ms(500))					//Blink LED 3 in Idle mode
					led3Tog;	
				break;
		}
		getNewCommunications(); //Checks for and interprets new communications
		nfRetrieveNavData();	//checks if there is new navigation data and updates robotPosition
		//check to see if obstacle avoidance is enabled AND the robot is moving
		if(obstacleAvoidanceEnabledFlag && movingFlag)
			dodgeObstacle(&robotPosition); //avoid obstacles using proximity sensors
	}
}