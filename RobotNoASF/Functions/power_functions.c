/*
* power_functions.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 13/09/2017 6:32:41 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for handling the charging and status of the battery
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t pfChargeCycleHandler(RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../Interfaces/pio_interface.h"
#include "../Interfaces/fc_interface.h"
#include "../Interfaces/timer_interface.h"
#include "../Interfaces/adc_interface.h"

#include "motion_functions.h"
#include "power_functions.h"


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void pfPollPower(RobotGlobalStructure *sys)
*
* Will poll data from the battery and fast charge chip if the corresponding flags are set in sys
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* none
*
* Implementation:
* Each of the three polls has the same guards. The first guard checks if the enable flag for that
* device in sys->power is enabled. If it is, the it checks if the poll time is equal or less
* than the system time stamp. If so, then the desired interval has lapsed since the last poll,
* (or the poll enabled flag has only just been turned on) in either case, perform the poll
* operation. In the case of polling the charge chip state, if the status returned by fcState() has
* an upper nibble with the value 0xF, it indicates that there has been a fault. The lower nibble
* stores the fault code. See fc_interface.h for a list of fault and status codes.
*
*/
void pfPollPower(RobotGlobalStructure *sys)
{
	static uint32_t pollBatteryTime = 0;		//Time at which battery will next be tolled
	static uint32_t pollChargerTime = 0;		//Time at which charge chip will next be polled
	static uint32_t chargerWatchDogTime = 0;	//Time at which watchdog message will next be sent
												//to fast charge chip
	
	//If battery polling is enabled and pollBatteryEnabled flag has just been set, or 
	//pollBatteryTime has elapsed then poll
	if(sys->power.pollBatteryEnabled && pollBatteryTime <= sys->timeStamp)
	{
		//Store the time at which the battery will next be polled
		pollBatteryTime = sys->timeStamp + sys->power.pollBatteryInterval;
		//Read battery voltage
		sys->power.batteryVoltage = adcBatteryVoltage();
		//If actual battery voltage is greater than the maximum battery voltage, then update the
		//maximum battery voltage value.
		if(sys->power.batteryVoltage > sys->power.batteryMaxVoltage)
			sys->power.batteryMaxVoltage = sys->power.batteryVoltage;
		
		//Calculate remaining battery percentage
		sys->power.batteryPercentage 
		=	100*(sys->power.batteryVoltage - sys->power.batteryMinVoltage)
		/	(sys->power.batteryMaxVoltage - sys->power.batteryMinVoltage);
		
		//Read battery temp
		//TODO: Read battery temp
	}
	
	//If fast charge chip polling is enabled
	if(sys->power.pollChargingStateEnabled && pollChargerTime <= sys->timeStamp)
	{
		//Store the time at which the charger status will next be polled
		pollChargerTime = sys->timeStamp + sys->power.pollChargingStateInterval;
		sys->power.fcChipStatus = fcState();
		if((sys->power.fcChipStatus & 0xF0) == 0xF0)//If fault (Upper nibble = F)
		{
			sys->power.fcChipStatus &= 0x0F;		//Remove upper nibble from status, (fault
													//code resides in lower nibble)
			sys->power.fcChipFaultFlag = 1;			//Set the fault flag. Must be reset by the
													//function that reads it, otherwise charging
													//can't resume.
		}
				
		if((sys->power.fcChipStatus == FC_BATTERY_CHARGING//If the chip has external power or is
		|| sys->power.fcChipStatus == FC_POWER_CONNECTED)//charging battery
		&& sys->states.mainf != M_DOCKING)				
		{
			//Don't do this if already in docking state, because docking state needs to be reset
			//when its found the charger, otherwise it won't work right the next time around.
			mfStopRobot(sys);							//Stop moving
			sys->states.mainfPrev = sys->states.mainf;	//Save last state
			sys->states.mainf = M_CHARGING;				//Switch to charging state
		}
	}
	
	//If Charger Watchdog is enabled and chargeWatchDogEnabled flag has just been set, or 
	//chargeWatchDogTime has elapsed then poll
	if(sys->power.pollChargingStateEnabled && chargerWatchDogTime <= sys->timeStamp)
	{
		//Set time at which watchdog will next be triggered
		chargerWatchDogTime = sys->timeStamp + sys->power.chargeWatchDogInterval;
		fcWatchdogReset();
	}
}

/*
* Function:
* uint8_t pfChargeCycleHandler(RobotGlobalStructure *sys)
*
* State machine that handles the charging cycle of the battery
*
* Inputs:
* struct SystemStatesGroup *state
*   Pointer to the sys.states data structure
* RobotGlobalStructure *sys:
*   Pointer to the sys->pos. data structure
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
* of this, but allow pfChargeCycleHandler to run through until finish (0). This will allow the 
* robot to dismnount itself from the bus bar
*
* Improvements:
* TODO: More commenting
*
*/
uint8_t pfChargeCycleHandler(RobotGlobalStructure *sys)
{
	static float currentHeading = 0;
	
	switch(sys->states.chargeCycle)
	{
		case CCS_CHECK_POWER:
			sys->power.pollChargingStateEnabled = 1;
			sys->power.pollChargingStateInterval = 10;
			if(sys->power.fcChipStatus == FC_BATTERY_CHARGING 
			|| sys->power.fcChipStatus == FC_POWER_CONNECTED)
			{
				//Disable prox
				sys->sensors.prox.pollEnabled = 0x00;
				//Disable light
				sys->sensors.colour.pollEnabled = 0;
				sys->states.chargeCycle = CCS_CHARGING;
			}
			break;
		
		case CCS_CHARGING:
			sys->power.pollChargingStateInterval = 1000;
			mfStopRobot(sys);
			//Blink LED
			if(!fdelay_ms(250))
				led3Tog;
			sys->power.chargeWatchDogEnabled = 1;
			if(sys->power.fcChipStatus == FC_BATTERY_CHARGED)	//If finished charging
				sys->states.chargeCycle = CCS_DISMOUNT;

			if(sys->power.fcChipFaultFlag)						//If fault (Upper nibble = F)
			{
				sys->states.chargeCycle = CCS_FAULT;
			}
			
			if(sys->power.fcChipStatus == 0)					//If connection broken
			{
				currentHeading = sys->pos.facing;
				sys->states.chargeCycle = CCS_RECONNECT;
			}
			break;
		
		case CCS_RECONNECT:
			sys->power.pollChargingStateInterval = 10;
			mfAdvancedMove(currentHeading, currentHeading, 100, 70, sys);	//Charge Forward
			//mfTrackLight(70, sys);
			if(sys->power.fcChipStatus == FC_BATTERY_CHARGING
			|| sys->power.fcChipStatus == FC_POWER_CONNECTED)
			{
				sys->states.chargeCycle = CCS_CHARGING;						//If reconnected
				mfStopRobot(sys);	
			}
			
			break;
		
		case CCS_FAULT:
			sys->power.fcChipFaultFlag = 0;
			sys->states.chargeCycle = CCS_DISMOUNT;
			return 0xFF;										//Indicate that a fault occurred
		
		//See which direction we are facing right now, then switch to CCS_TURN_AWAY
		case CCS_DISMOUNT:
			currentHeading = sys->pos.facing;
			sys->states.chargeCycle = CCS_STOP_POLLING;
			break;
		
		//Rotate 180 degrees, then switch to CCS_FINISHED STATE
		case CCS_TURN_AWAY:
			if(!mfRotateToHeading(currentHeading + 180, sys))
				sys->states.chargeCycle = CCS_FINISHED;
			break;
		
		case CCS_STOP_POLLING:
			//reenable sensors
			sys->sensors.prox.pollEnabled = 0x3F;
			sys->sensors.colour.pollEnabled = 0x03;
			//Charge chip polling = 1sec
			sys->power.pollChargingStateInterval = 1000;
			sys->power.pollChargingStateEnabled = 1;
			//Disable charge chip watchdog
			sys->power.chargeWatchDogEnabled = 0;
			sys->states.chargeCycle = CCS_TURN_AWAY;
			break;
			
		case CCS_FINISHED:
			sys->states.chargeCycle = CCS_CHECK_POWER;
			break;
	}
	
	return sys->states.chargeCycle;
}