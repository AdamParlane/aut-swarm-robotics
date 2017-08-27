/*
* fc_interface.c
*
* Author : Esmond Mather, Matthew Witt and Adam Parlane
* Created: 11/07/2017 11:21:08 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for accessing and controlling the fast charge chip.
*
* More Info:
* Atmel SAM 4N Processor Datasheet: http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* TI Battery charger Datasheet:http://www.ti.com/lit/ds/symlink/bq24160.pdf
*
* Functions:
* void fcInit(void)
* void fcWatchdogReset(void)
* uint16_t fcBatteryVoltage(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "fc_interface.h"
#include "twimux_interface.h"	//Needed for TWI0 read and write functions

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void fcInit(void)
*
* Initialises chip disable PIO output on microntroller and loads initial settings into fast charge
* chip. TWI0 must be initialised first.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* PIOB, pin 2 is given to PIO controller, it is set as an output and is pulled low (to enable fast
* charge chip). Next, initial settings for the fast charge chip a written including:
* - Clearing the charge enable bit in the control register to enable charging
* - Tell the chip the battery voltage and maximum input current
* - Tell the chip the maximum charging current of the batter, and the current at which charging
*   terminates.
*
*/
void fcInit(void)
{
	//Initialise Chip disable (CD) line on PB2 (PIO setup)
	REG_PIOB_PER |=	PIO_PER_P2;		//Give control of PB2 to PIOB controller
	REG_PIOB_OER |= PIO_OER_P2;		//Set PB2 as an output
	REG_PIOB_CODR |= PIO_CODR_P2;	//Set PB2 to low
	
	
	uint8_t writeBuffer;
	//NOT SHURE IF THIS LINE IS NEEDED BE CAUSE ACCORDING TO DATASHEET CE IS INVERTED, THEREFORE THE
	//DEFAULT VALUE 0 WOULD MEAN THAT CHARGING IS ENABLED NOT 1. TESTING REQUIRED.
	//Ensures that CE bit is clear in case safety timer has gone off in previous charge.
	writeBuffer = FC_CONTROL_INIT;
	twi0Write(TWI0_FCHARGE_ADDR, FC_CONTROL_REG, 1, &writeBuffer);
	
	//Vreg = 3.98v, input current = 2.5A
	writeBuffer = FC_BATTVOL_INIT;
	twi0Write(TWI0_FCHARGE_ADDR, FC_BATVOL_REG, 1, &writeBuffer);
	
	//Charge current set to max Ic=2875mA, termination current Iterm=100mA (default)
	writeBuffer = FC_CHARGE_INIT;
	twi0Write(TWI0_FCHARGE_ADDR, FC_CHARGE_REG, 1, &writeBuffer);
}

/*
* Function:
* void fcWatchdogReset(void)
*
* Resets the watchdog timer on the fast charge chip. Presumable this is so that the chip knows that
* it is being monitored by its master. If watchdog timer is not reset in 30sec, the chip resets to
* default settings.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Writes the reset bit to the status register on the chip
*
*/
void fcWatchdogReset(void)
{
	uint8_t writeBuffer = FC_STATUS_WDRESET;
	//Resets the FCC watchdog timer. Must be done once every 30s or else registers will reset.
	twi0Write(TWI0_FCHARGE_ADDR, FC_STATUS_REG, 1, &writeBuffer);
}

/*
* Function:
* uint8_t fcVersionRead(void)
*
* Returns revision number from version register on FC chip
*
* Inputs:
* none
*
* Returns:
* Returns a 3 bit value with the version number of the chip
*   0x00: Rev 1.0
*   0x01: Rev 1.1
*   0x02: Rev 2.0
*   0x03: Rev 2.1
*   0x04: Rev 2.2
*   0x05: Rev 2.3
*   0x06+: Future revisions
*
* Implementation:
* Reads the data stored in the version register of the fast charge chip and returns the first three
* bits (the revision number)
*
* Improvements:
* Have a data structure that contains all data about the chip and load all 8bits from the version
* register into it (instead of just revision number)
*
*/
uint8_t fcVersionRead(void)
{
	//Reset state for this register is 0x40 according to datasheet
	uint8_t returnVal = 0;
	twi0Read(TWI0_FCHARGE_ADDR, FC_VERSION_REG, 1, &returnVal);
	//Return just the revision number (first 3 bits)
	return (0x07 & returnVal);
}

/*
* Function:
* float fcBatteryVoltage(void)
*
* Returns current battery voltage
*
* Inputs:
* none
*
* Returns:
* Returns a integer value of the current battery voltage in mV.
*
* Implementation:
* Reads battery voltage ADC channel
* Converts to millivolts using linear conversion factor set in fc_interface.h and returns
*
*/
uint16_t fcBatteryVoltage(void)
{
	uint16_t rawBattReading = 0;
	rawBattReading = adcRead(FC_BATVOLT_ADC_CH);
	return rawBattReading*FC_BATTVOL_CONV;
}

/*
* Function:
* uint8_t chargeDetector(void)
*
* Returns battery charging status
*
* Inputs:
* none
*
* Returns:
* CHARGING for when a valid charging source is connected
* CHARGED for when the battery is charged
* the value of the status control register if there is an error or no charging
*
* Implementation:
* Uses the struct union Register to separate the individual bits
* reads the status control register on the fast charge chip
* returns whether CHARGING, CHARGED, or other
*
* Improvements:
*	//TODO: Probably shouldn't be manipulating mainRobotState from here. Should be able to see when
*	//mainRobotState is being changed from the main function, otherwise it just appears to magically
*	//change itself if you know what I mean.. Maybe set this up with an appropriate return system
*
*/
uint8_t chargeDetector(void)
{
	Register fcstatus;
	twi0Read(TWI0_FCHARGE_ADDR, FC_STATUS_REG, 1, &fcstatus.status);
	if(fcstatus.bit.b5 & fcstatus.bit.b4) //if robot is charging
	{
		//on the first time entering charge mode save the previous state for re entry
		if(mainRobotState != CHARGING) 
			mainRobotStatePrev = mainRobotState;
		if(!fdelay_ms(500))					//Blink LED 1 in charge mode
			led1Tog;
		mainRobotState = CHARGING;
		return BATT_CHARGING;
	}
	else if (fcstatus.bit.b6 & fcstatus.bit.b4)//if robot is charged
		mainRobotState = mainRobotStatePrev;
	else //if the robot is not charging (eg fault or no contact)
		mainRobotState = mainRobotStatePrev;
	//TODO: Set this up with the ability to return an error value (Maybe from twi0Read?)
	return 0;
}