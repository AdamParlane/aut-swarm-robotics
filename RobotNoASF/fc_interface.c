/*
* fc_interface.c
*
* Author : Esmond Mather and Matthew Witt
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
* void FastChargeController_Setup(void)
* void FastChargeController_WatchDogReset(void)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "fc_interface.h"
#include "twimux_interface.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void FastChargeController_Setup(void)
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
void FastChargeController_Setup(void)
{
	//Initialise Chip disable (CD) line on PB2 (PIO setup)
	REG_PIOB_PER |= PIO_PER_P2;		//Give control of PB2 to PIOB controller
	REG_PIOB_OER |= PIO_OER_P2;		//Set PB2 as an output
	REG_PIOB_CODR |= PIO_CODR_P2;	//Set PB2 to low
	
	//NOT SHURE IF THIS LINE IS NEEDED BE CAUSE ACCORDING TO DATASHEET CE IS INVERTED, THEREFORE THE
	//DEFAULT VALUE 0 WOULD MEAN THAT CHARGING IS ENABLED NOT 1. TESTING REQUIRED.
	twi0Write(TWI0_FCHARGE_ADDR, controlReg, initControl);	//Ensures that CE bit is clear in case
															//safety timer has gone off in previous
															//charge.
															
	twi0Write(TWI0_FCHARGE_ADDR, battVReg, initBattV);		//Vreg = 3.98v, input current = 2.5A
	
	twi0Write(TWI0_FCHARGE_ADDR, chargeReg, initCharge);	//Charge current set to max Ic=2875mA,
															//termination current Iterm=100mA
															//(default)
}

/*
* Function:
* void FastChargeController_WatchDogReset(void)
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
void FastChargeController_WatchDogReset(void)
{
	twi0Write(TWI0_FCHARGE_ADDR, statusReg, watchdreset);	//Resets the FCC watch dog timer. Must
															//be done once every 30s or else 
															//registers will reset
}