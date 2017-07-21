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
* void FastChargeController_Setup(void)
* void FastChargeController_WatchDogReset(void)
*
*/

#ifndef FC_INTERFACE_H_
#define FC_INTERFACE_H_

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//Fast charge chip registers
#define statusReg		0x00	//Status register (Contains timer reset bit)
#define controlReg		0x02	//Control register (Contains CE bit)
#define battVReg		0x03	//Battery Voltage register
#define chargeReg		0x05	//Charge Current register
#define NTCmonitorReg	0x07	//Register for TS fault bits B2 & B1, 00=normal, 01=nil charge, 
								//10=1/2 current, 11=Vreg reduced.
//Register states
#define watchdreset		0x80	//Polls the timer reset bit to stop the watchdog from expiring
								//(statusReg)
#define initControl		0x04    //Ensures that CE bit is clear in case safety timer has gone off in
								//previous charge.
#define initBattV		0x66	//Vreg = 3.98v, input current = 2.5A (battVReg)
#define initCharge		0xFA	//charge current set to max Ic=2875mA, termination current
								//Iterm=100mA (default) (chargeReg)

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
*/
void FastChargeController_Setup(void);

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
*/
void FastChargeController_WatchDogReset(void);


#endif /* FC_INTERFACE_H_ */