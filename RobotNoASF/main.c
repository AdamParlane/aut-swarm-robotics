/*
* main.c
*
* Author : et. al
* Created: Unknown
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
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

#include "Interfaces/pio_interface.h"
#include "Interfaces/timer_interface.h"
#include "Interfaces/motor_driver.h"
///Testing only::////////////////////////////////
#include "Interfaces/camera_interface.h"       //
#include "Interfaces/camera_buffer_interface.h"//
/////////////////////////////////////////////////

#include "Functions/power_functions.h"
#include "Functions/comm_functions.h"
#include "Functions/docking_functions.h"
#include "Functions/sensor_functions.h"
#include "Functions/manual_mode.h"
#include "Functions/motion_functions.h"
#include "Functions/navigation_functions.h"
#include "Functions/obstacle_avoidance.h"
#include "Functions/test_functions.h"

//////////////[Global variables]////////////////////////////////////////////////////////////////////
extern RobotGlobalStructure sys;		//System data structure
///TEMP FOR TESTING CAMERA//////////////////////////////////////////////////////////////////////
//uint16_t data[28800];			// 320*90 (w*h) 2 bytes per pixel                             //
////////////////////////////////////////////////////////////////////////////////////////////////
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
* 
* The main system runs on a state machine in a while(1) loop to control the robot
* Before this state machine is run robotSetup() is called to initialise all peripherals
* The main robot state is initialised at IDLE to allow the user (on PC GUI) to input control
*
* The cases are:
* TEST:			***module: test_functions.c/h
*				System test where the robot sends back peripheral data when requested from the PC
*				This may be 1 sample of data or streaming data every 100ms
*				Used to ensure the robot is functioning correctly and the data can be read from the GUI
* MANUAL:		***module: manual_mode.c/h
*				Manual mode allows the user to drive the robot around in real time
*				Again this is called and controlled by the GUI
*				Movement options are move (N, NE, E, SE, S, SW, W, NW) rotate (CW, CCW) at any speed
*				The manualControl function which handles the command is called when it get a new command
*				Charge detector is called to check if the user has docked the robot
* DOCKING:		***module: docking_functions.c/h
*				Executes the ideal docking routine to charge the robot
*				This function returns 0 when complete and changes states to IDLE
* LINE_FOLLOW:	***module: motion_functions.c/h
*				Executes the line follow routine to either charge the robot or just follow a line
*				This function returns 0 when complete and changes states to IDLE
* LIGHT_FOLLOW:	***module: motion_functions.c/h
*				Executes the light follow routine to either charge the robot or just follow a light
* FORMATION:	***module: [WIP]
* CHARGING:		***module: fc_Interface.c/h
*				Checks the robot is still charging and the status of the battery
*				Entered when the robot is docked
*				Charge_info also contains the charging status should other functions require it
* IDLE:			***module: N/A
*				Stops the robot motion by calling mfStopRobot()
*				Blinks LED 3 every 500ms to show externally that robot is in IDLE
*				Entered when other majour blocking functions or states are finished
*				Exitied by PC commands
*
* After the state machine 3 functions are run every loop to check key peripherals
* These are:	commGetNew()			checks and handles incoming commands
*				nfRetrieveData()				updates the robot's navigation structure
*				dodgeObstacle()					executes obstacle avoidance routine
* Note that doegeObstacle() is guarded by 2 flags: obstacleAvoidance enabled and the robot is moving
*
* Improvements:
*
* More functionality incoming (formations, smarter obstacle avoidance, high level error codes etc)
*
*/


int main(void)
{
	robotSetup(); //Set up the system and peripherals
	//Battery voltage stored in sys.power.batteryVoltage
	//Initial main function state is SET IN robot_setup.c (sys.states.mainf) (NOT here)
	while(1)
	{
		switch (sys.states.mainf)
		{
			case M_TEST: //System Test Mode
			//Entered when test command received from PC
				testManager(&sys); //Interprets test command and executes it
				break;
			
			case M_MANUAL: //User controlled mode
			//Entered when manual movement command received from PC
				if(sys.flags.xbeeNewData) //if there is new data
					manualControl(&sys);
				break;
			
			case M_DOCKING:
			//if battery low or manual docking command sent from PC
				switch(dfDockRobot(&sys))				//Execute docking procedure state machine
				{
					case DS_FINISHED:
						sys.states.mainf = M_CHARGING;	//If finished docking, switch to charging
						break;
						
					case DS_CHRG_NOT_FOUND:
						sys.states.mainf = M_IDLE;		//If charger connection failed
						break;
				}	
				break;
			
			case M_LINE_FOLLOW:
			//Entered when line follow command received from PC
				if(!dfFollowLine(100, &sys))//Line follower will return 0 when complete
					sys.states.mainf = M_IDLE;
				break;
					
			case M_LIGHT_FOLLOW:
			//Entered when light follow command received from PC
				mfTrackLight(50, &sys);
				break;
				
			case M_RANDOM:
				mfRandomMovementGenerator(&sys);
				break;
				
			case M_FORMATION:
			//placeholder
				break;
						
			case M_OBSTACLE_AVOIDANCE:
				//avoid obstacles using proximity sensors
				if(!dodgeObstacle(&sys))//returning 0 means obstacles have been avoided
					sys.states.mainf = sys.states.mainfPrev; //reset the state to what it was
				break;
				
			case M_OBSTACLE_AVOIDANCE_DEMO:
				//will act like a function requiring OA for sake of demo'ing
				dodgeObstacleByFacing(&sys);
				break;
				
			case M_MOVE_TO_POSITION:
				//Move the robot to the position in targetX and targetY, then revert to the previous 
				//state
				if(!mfMoveToPosition(sys.pos.targetX, sys.pos.targetY, 50, 0, 60, &sys))
					sys.states.mainf = sys.states.mainfPrev;
				break;

			case M_CHARGING:
				switch(pfChargeCycleHandler(&sys))
				{
					case CCS_FINISHED:
						sys.states.mainf = sys.states.mainfPrev;	//Charge finished successfully
						break;	
				}
				break;
				
			case M_TEST_ALL:
				//Something
				break;
			
			case M_STARTUP_DELAY:
				//Added this non blocking startup delay to see if it is more friendly.
				//Please set initial state in mainfPrev
				if(!fdelay_ms(sys.startupDelay))
					sys.states.mainf = sys.states.mainfPrev;
				break;
				
			case M_IDLE:					
				mfStopRobot(&sys);
				//CAMERA DEBUG STUFF
				//if(!camBufferWriteFrame())					//Load frame into buffer
				//{
					//camBufferReadWin(0, 220, 311, 40, data, 28800);//Read data from buffer	
				//}		
				if(!fdelay_ms(1000))					//Blink LED 3 in Idle mode
					led3Tog;				
				break;
		}
		
		nfRetrieveNavData(&sys);	//checks if there is new navigation data and updates sys->pos
		
		commGetNew(&sys);			//Checks for and interprets new communications
		
		commPCStatusUpdate(&sys);	//Updates PC with battery and state (every 5 seconds)
		
		pfPollPower(&sys);			//Poll battery and charging status
		
		sfPollSensors(&sys);		//Poll prox, colour, line 
		
		//check to see if obstacle avoidance is enabled AND the robot is moving
		//if(sys.flags.obaEnabled && sys.flags.obaMoving && sys.states.mainf != M_OBSTACLE_AVOIDANCE)
			//checkForObstacles(&sys); //avoid obstacles using proximity sensors
			
		
	}
}