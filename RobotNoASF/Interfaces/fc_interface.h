/*
* fc_interface.h
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:23:08 AM
*
* Project Repository:https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines register addresses and states for the fast charge chip. Defines functions for accessing
* and controlling the fast charge chip.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* TI Battery charger Datasheet:http://www.ti.com/lit/ds/symlink/bq24160.pdf
*
* Functions:
* void fcInit(void)
* void fcWatchdogReset(void)
*
*/

#ifndef FC_INTERFACE_H_
#define FC_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//Fast charge chip registers
#define FC_STATUS_REG	0x00	//Status register (Contains timer reset bit)
#define FC_CONTROL_REG	0x02	//Control register (Contains CE bit)
#define FC_VERSION_REG	0x04
#define FC_BATVOL_REG	0x03	//Battery Voltage register
#define FC_CHARGE_REG	0x05	//Charge Current register
#define FC_NTCMON_REG	0x07	//Register for TS fault bits B2 & B1, 00=normal, 01=nil charge, 
								//10=1/2 current, 11=Vreg reduced.
//Register states
#define FC_STATUS_WDRESET 0x80	//Polls the timer reset bit to stop the watchdog from expiring
								//(FC_STATUS_REG)
#define FC_CONTROL_INIT	0x04    //Ensures that CE bit is clear in case safety timer has gone off in
								//previous charge.
#define FC_BATTVOL_INIT	0x66	//Vreg = 3.98v, input current = 2.5A (FC_BATVOL_REG)
#define FC_CHARGE_INIT	0xFA	//charge current set to max Ic=2875mA, termination current
								//Iterm=100mA (default) (FC_CHARGE_REG)
								
//Misc
//	ADC to battery voltage conversion factor
#define FC_BATTVOL_CONV	ADC_VOLTAGE_REF/1023

///////////////Functions////////////////////////////////////////////////////////////////////////////
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
*/
void fcInit(void);

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
*/
void fcWatchdogReset(void);

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
*/
uint8_t fcVersionRead(void);

#endif /* FC_INTERFACE_H_ */