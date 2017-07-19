/*
* twimux_interface.h
*
* Author : Esmond Mathers and Matthew Witt
* Created: 11/07/2017 10:03:32 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines macros for TWI0 status and multiplexer addresses. Function definitions for TWI0 and mux
*
* More Info:
* Atmel SAM 4N Processor Datasheet: http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Farnell I2C Mux Datasheet:http://www.farnell.com/datasheets/1815500.pdf
*
* Functions:
* void twi0Init(void)
* void TWI0_MuxSwitch(uint8_t channel)
* uint8_t TWI0_ReadMuxChannel(void)
* void TWI0_Write(uint8_t SlaveAddress, uint8_t intAddress, uint8_t Data)
* uint8_t TWI0_ReadSB(uint8_t SlaveAddress, uint8_t intAddress)
* uint16_t TWI0_ReadDB(uint8_t SlaveAddress, uint8_t intAddress)
*
*/

#ifndef TWIMUX_INTERFACE_H_
#define TWIMUX_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "sam.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//General Commands
//RHR: Receive holding register, THR: Transmit holding register, NACK: Not acknowledge
#define twi0RXRDY	REG_TWI0_SR & (1<<1)		//if 1, RHR has new byte to be read
#define twi0TXRDY	REG_TWI0_SR & (1<<2)		//if 1, THR is empty or NACK error occurred
#define twi0TXCOMP	REG_TWI0_SR & (1<<0)
#define twi0NACK	REG_TWI0_SR & (1<<8)		//Check TWI0 Status register for Not Acknowledged
//Device slave addresses
#define TWI0_Mux_Address			0xE0		//Mux Address 000
#define TWI0_LightSensorAddress		0x10		//Light sensors
#define TWI0_ProximitySensorAddress 0x39		//Proximity sensors
#define TWI0_FastChargeChipAddress	0x6B		//Battery Charger (Fast Charge Controller)
//TWI Mux channels
//Only one active at a time
#define Mux_RHS_LightSens			0xF8		//Mux Channel 0, Side Panel A
#define Mux_LHS_LightSens			0xF9		//Mux Channel 1, Side Panel A
#define Mux_ProximityA				0xFA		//Mux Channel 2, Side Panel A
#define Mux_ProximityB				0xFF		//Mux Channel 7, Side Panel B
#define Mux_ProximityC				0xFE		//Mux Channel 6, Side Panel C
#define Mux_ProximityD				0xFD		//Mux Channel 5, Side Panel D
#define Mux_ProximityE				0xFC		//Mux Channel 4, Side Panel E
#define Mux_ProximityF				0xFB		//Mux Channel 3, Side Panel F

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void twi0Init(void)
*
* Initialises TWI0. Master clock should be setup first.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void twi0Init(void);

/*
* Function:
* void TWI0_MuxSwitch(uint8_t channel)
*
* Sets the I2C multiplexer to desired channel.
*
* Inputs:
* uint8_t channel: Mux channel number 0x8 (Ch0) to 0xF (Ch7)
*
* Returns:
* none
*
*/
void TWI0_MuxSwitch(uint8_t channel);

/*
* Function:
* uint8_t TWI0_ReadMuxChannel(void)
*
* Returns the channel the Mux is currently set to.
*
* Inputs:
* none
*
* Returns:
* a byte that holds a number from 0x8 (Ch0) to 0xF (Ch7) representing the current channel
*
*/
uint8_t TWI0_ReadMuxChannel(void);

/*
* Function:
* void TWI0_Write(uint8_t SlaveAddress, uint8_t intAddress, uint8_t Data)
*
* Will write a byte on TWI0 to the slave device and internal register specified in the parameters
*
* Inputs:
* uint8_t SlaveAddress:
*    The address of the slave device on TWI0 to write to
* uint8_t intAddress:
*    The internal address of the register to write to
* uint8_t Data:
*    The byte to write
*
* Returns:
* none
*
*/
void TWI0_Write(uint8_t SlaveAddress, uint8_t intAddress, uint8_t Data);

/*
* Function:
* uint8_t TWI0_ReadSB(uint8_t SlaveAddress, uint8_t intAddress)
*
* Will read a single byte from a TWI slave device that has 8bit internal register addresses
*
* Inputs:
* uint8_t SlaveAddress:
*    The address of the slave device on TWI0 to read from
* uint8_t intAddress:
*    The internal address of the register to read from
*
* Returns:
* a byte containing the contents of the internal register on the slave device specified.
*
*/
uint8_t TWI0_ReadSB(uint8_t SlaveAddress, uint8_t intAddress);

/*
* Function:
* uint8_t TWI0_ReadDB(uint8_t SlaveAddress, uint8_t intAddress)
*
* Will read two bytes from a TWI slave device that has 8bit internal register addresses
*
* Inputs:
* uint8_t SlaveAddress:
*    The address of the slave device on TWI0 to read from
* uint8_t intAddress:
*    The internal address of the register to read from
*
* Returns:
* a 16bit integer containing the contents of the internal register on the slave device specified.
*
*/
uint16_t TWI0_ReadDB(uint8_t SlaveAddress, uint8_t intAddress);


#endif /* TWIMUX_INTERFACE_H_ */