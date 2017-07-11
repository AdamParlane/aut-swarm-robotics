/*
 * twimux_interface.h
 *
 * Created: 11/07/2017 10:03:32 AM
 *  Author: Matthew Witt
 *	Desc: Defines for TWI0, and the TWI mux
 */ 


#ifndef TWIMUX_INTERFACE_H_
#define TWIMUX_INTERFACE_H_

#include "sam.h"

///////////////Defines//////////////////
/******** TWI ********/
/*** General Commands ***/
#define twi0RXRDY	REG_TWI0_SR & (1<<1)
#define twi0TXRDY	REG_TWI0_SR & (1<<2)
#define twi0TXCOMP	REG_TWI0_SR & (1<<0)
#define twi0NACK	REG_TWI0_SR & (1<<8)		//Check TWI0 Status register for Not Acknowledged

/*** Device/Slave Addresses ***/
#define TWI0_Mux_Address			0xE0		//Mux Address 000
#define TWI0_LightSensorAddress		0x10		//Light sensors
#define TWI0_ProximitySensorAddress 0x39		//Proximity sensors
#define TWI0_FastChargeChipAddress	0x6B		//Battery Charger (Fast Charge Controller)

/*** I2C Mux Channel ***/
/* Only one channel is active at a time */
#define Mux_RHS_LightSens	0xF8				//Mux Channel 0, Side Panel A
#define Mux_LHS_LightSens	0xF9				//Mux Channel 1, Side Panel A
#define Mux_ProximityA		0xFA				//Mux Channel 2, Side Panel A
#define Mux_ProximityB		0xFF				//Mux Channel 7, Side Panel B
#define Mux_ProximityC		0xFE				//Mux Channel 6, Side Panel C
#define Mux_ProximityD		0xFD				//Mux Channel 5, Side Panel D
#define Mux_ProximityE		0xFC				//Mux Channel 4, Side Panel E
#define Mux_ProximityF		0xFB				//Mux Channel 3, Side Panel F

////////////////Functions/////////////////
void twi0Init(void);
void TWI0_MuxSwitch(uint8_t channel);
uint8_t TWI0_ReadMuxChannel(void);
void TWI0_Write(uint8_t SlaveAddress, uint8_t intAddress, uint8_t Data);
uint8_t TWI0_ReadSB(uint8_t SlaveAddress, uint8_t intAddress);
uint16_t TWI0_ReadDB(uint8_t SlaveAddress, uint8_t intAddress);


#endif /* TWIMUX_INTERFACE_H_ */