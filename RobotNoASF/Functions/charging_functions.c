/*
* charging_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/09/2017 6:32:41 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for handling the charging of the battery
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t cfChargeCycleHandler(struct Position *imuData)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "charging_functions.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t cfChargeCycleHandler(struct Position *imuData)
*
* State machine that handles the charging cycle of the battery
*
* Inputs:
* struct SystemStates *state
*   Pointer to the systemStates data structure
* struct Position *imuData:
*   Pointer to the robotPosition data structure
*
* Returns:
* Returns 0 when finished charging successfully. Returns 0xFF when a fault has occurred, otherwise
* returns the number of the current state.
*
* Implementation:
* First, the state machine will wait for a power connection. When the power has been connected,
* The charging cycle begins. The CHARGING state waits for either a battery charged status from the
* FC chip, or a FAULT status. If battery has finished charging, the state machine moves to the 
* DISMOUNT state, where the robot will turn 180 degrees to face away from the dock. When a fault is
* detected the state machine will also switch to the DISMOUNT state, but it will return from the
* function with 0xFF to indicate to the caller that there was a problem. The caller should take note
* of this, but allow cfChargeCycleHandler to run through until finish (0). This will allow the 
* robot to dismnount itself from the bus bar
*
* Improvements:
* TODO: More commenting
*
*/
uint8_t cfChargeCycleHandler(struct SystemStates *state, Position *imuData)
{
	static float currentHeading = 0;
	uint8_t chipState = 0;
	fcWatchdogReset();							//Reset watchdog timer on fc chip
	chipState = fcState();
	
	switch(state->chargeCycle)
	{
		case CHECK_POWER:
			if(chipState == FC_BATTERY_CHARGING || chipState == FC_POWER_CONNECTED)
				state->chargeCycle = CHARGING;
			break;
		
		case CHARGING:
			if(!fdelay_ms(250))
				led3Tog;
			if(chipState == FC_BATTERY_CHARGED)
			{
				state->chargeCycle = DISMOUNT;
			}
			if((chipState & 0xF0) == 0xF0)		//If fault (Upper nibble = F)
				state->chargeCycle = FAULT;
			break;
		
		case FAULT:
			fcEnableCharging(0);				//Stop charging
			state->chargeCycle = DISMOUNT;
			return 0xFF;						//Indicate that a fault occurred
		
		case DISMOUNT:
			currentHeading = imuData->imuYaw;
			state->chargeCycle = TURN_AWAY;
			break;
		
		case TURN_AWAY:
			if(!mfMoveToHeadingByDistance(currentHeading - 180, 35, 10, imuData))
				state->chargeCycle = FINISHED;
			break;
		
		case FINISHED:
			state->chargeCycle = CHECK_POWER;
			break;
	}
	
	return state->chargeCycle;
}