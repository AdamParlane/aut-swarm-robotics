/*
 * fc_interface.c
 *
 * Created: 11/07/2017 11:21:08 AM
 *  Author: Matthew Witt
 *	Desc: Functions for initialising and controlling the fast charge chip
 */ 

#include "fc_interface.h"
#include "twimux_interface.h"


/******** Fast Charge Controller Registry Setup ********/
void FastChargeController_Setup(void)
{
	//Chip disable (CD) line on PB2 set to enable
	REG_PIOB_PER |= (1<<2);		//Give control of PB2 to PIOB controller
	REG_PIOB_OER |= (1<<2);		//Set PB2 as an output
	REG_PIOB_CODR |= (1<<2);	//Set PB2 to low
	twi0Write(TWI0_FCHARGE_ADDR, controlReg, initControl);	// Ensures that CE bit is clear in case safety timer has gone off in previous charge.
	twi0Write(TWI0_FCHARGE_ADDR, battVReg, initBattV);		// Vreg = 4.0v, input current = 2.5A
	twi0Write(TWI0_FCHARGE_ADDR, chargeReg, initCharge);		// charge current set to max Ic=2875mA, termination current Iterm=100mA (default)
}



void FastChargeController_WatchDogReset(void)
{
	twi0Write(TWI0_FCHARGE_ADDR, statusReg, watchdreset);	//Resets the FCC watch dog timer. Must be done once every 30s or else registers will reset
}