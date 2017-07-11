/*
 * fc_interface.h
 *
 * Created: 11/07/2017 10:23:08 AM
 *  Author: Matthew Witt
 */ 


#ifndef FC_INTERFACE_H_
#define FC_INTERFACE_H_

/*** Fast Charge Controller ***/
/* Registers */
#define statusReg		0x00	// Status register (Contains timer reset bit)
#define controlReg		0x02	// Control register (Contains CE bit)
#define battVReg		0x03	// Battery Voltage register
#define chargeReg		0x05	// Charge Current register
#define NTCmonitorReg	0x07	// Register for TS fault bits B2 & B1, 00=normal, 01=nil charge, 10=1/2 current, 11=Vreg reduced.
/* Values for registers */
#define watchdreset		0x80	// Polls the timer reset bit to stop the watchdog from expiring (statusReg)
#define initControl		0x04    // Ensures that CE bit is clear in case safety timer has gone off in previous charge.
#define initBattV		0x66	// Vreg = 4.0v, input current = 2.5A (battVReg)
#define initCharge		0xFA	// charge current set to max Ic=2875mA, termination current Iterm=100mA (default) (chargeReg)

void FastChargeController_Setup(void);
void FastChargeController_WatchDogReset(void);


#endif /* FC_INTERFACE_H_ */