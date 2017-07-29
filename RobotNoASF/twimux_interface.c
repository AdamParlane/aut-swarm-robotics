/*
* twimux_interface.c
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:41:59 AM
*
* Project Repository:https://github.com/AdamParlane/aut-swarm-robotics
*
* Functions for reading and writing I2C devices on TWI0 bus. Functions for controlling I2C
* multiplexer on TWI0.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Farnell I2C Mux Datasheet:http://www.farnell.com/datasheets/1815500.pdf
*
* Functions:
* void twi0Init(void)
* void twi0MuxSwitch(uint8_t channel)
* uint8_t twi0ReadMuxChannel(void)
* void twi0Write(uint8_t slaveAddress, uint8_t intAddress, uint8_t data)
* uint8_t twi0ReadSingle(uint8_t slaveAddress, uint8_t intAddress)
* uint16_t twi0ReadDouble(uint8_t slaveAddress, uint8_t intAddress)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "twimux_interface.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void twi0Init(void);
*
* Initialises TWI0. Master clock should be setup first.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Supply master clock to TWI0 peripheral
* - Set PA3(Data) and PA4(Clock) to peripheral A mode (TWI)
* - Set the high and low periods of the TWI clock signal using formula from datasheet
*	NOTE: A high period of 0.6uSec and a low period of 1.3uSec is required by both the Proximity
*	and Light Sensors
*                t_low
*   CLDIV =  --------------  - 2
*            2(t_masterclk)
*
*                t_high
*   CHDIV =  --------------  - 2
*            2(t_masterclk)
*
* - Make uC master on the TWI bus
*
*/
void twi0Init(void)
{
	REG_PMC_PCER0
	|=	(1<<ID_TWI0);						//Enable clock access to TWI0, Peripheral TWI0_ID = 19
	REG_PIOA_PDR
	|=	PIO_PDR_P3							// Enable peripheralA control of PA3 (TWD0)
	|	PIO_PDR_P4;							// Enable peripheralA control of PA4 (TWCK0)
	twi0Reset;								//Software reset

	//TWI0 Clock Waveform Setup
	//REG_TWI0_CWGR
	//|=	TWI_CWGR_CKDIV(1)					//Clock speed 400000, fast mode
	//|	TWI_CWGR_CLDIV(73)					//Clock low period 1.5uSec
	//|	TWI_CWGR_CHDIV(48);					//Clock high period  1.0uSec
	//twi0MasterMode;							//Master mode enabled, slave disabled

	REG_TWI0_CWGR
	|=	TWI_CWGR_CKDIV(1)					//Clock speed 400000, fast mode
	|	TWI_CWGR_CLDIV(148)					//Clock low period 1.5uSec
	|	TWI_CWGR_CHDIV(98);					//Clock high period  1.0uSec
	twi0MasterMode;							//Master mode enabled, slave disabled
}

/*
* Function:
* void twi2Init(void);
*
* Initialises TWI2. Master clock should be setup first.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Supply master clock to TWI2 peripheral
* - Set PB0(Data) and PB1(Clock) to peripheral B mode (TWI)
* - Set the high and low periods of the TWI clock signal using formula from datasheet
*	NOTE: A high period of 0.6uSec and a low period of 1.3uSec is required by IMU on TWI2
*	1.3uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
*	0.6uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
* - Make uC master on the TWI bus
*
*/
void twi2Init(void)
{
	REG_PMC_PCER0
	|=	(1<<ID_TWI2);						//Enable clock access to TWI2, Peripheral TWI2_ID = 22
	REG_PIOB_PDR
	|=	PIO_PDR_P0							//Enable peripheralB control of PB0 (TWD2)
	|	PIO_PDR_P1;							//Enable peripheralB control of PB1 (TWCK2)
	REG_PIOB_ABCDSR
	|=	PIO_ABCDSR_P0						//Set peripheral B
	|	PIO_ABCDSR_P1;
	twi2Reset;								//Software Reset
	
	//TWI2 Clock Waveform Setup.
	REG_TWI2_CWGR
	|=	TWI_CWGR_CKDIV(1)					//Clock speed 400000, fast mode
	|	TWI_CWGR_CLDIV(63)					//Clock low period 1.3uSec
	|	TWI_CWGR_CHDIV(28);					//Clock high period  0.6uSec
	twi2MasterMode;							//Master mode enabled, slave disabled
}

/*
* Function:
* void twi0MuxSwitch(uint8_t channel);
*
* Sets the I2C multiplexer to desired channel.
*
* Inputs:
* uint8_t channel: Mux channel number 0x8 (Ch0) to 0xF (Ch7)
*
* Returns:
* none
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the Mux
* - Mux has only one internal register, so no need to specify internal address
* - Load channel parameter into the transmit holding register for transmission and set STOP bit
*   (one byte transmission)
* - Wait for transmission to complete before exiting function.
*
*/
void twi0MuxSwitch(uint8_t channel)
{
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(TWI0_MUX_ADDR);	//Slave address (eg. Mux or Fast Charge Chip)
	//No internal address and set to master write mode by default of zero
	twi0Send(channel);				//Load THR and writing to THR causes start to be sent
	twi0Stop;						//Set STOP bit after tx
	while(!twi0TxReady);			//wait for start and data to be shifted out of holding register
	while(!twi0TxComplete);			//Communication complete, holding and shifting registers
									//empty, Stop sent
}

/*
* Function:
* uint8_t twi0ReadMuxChannel(void)
*
* Returns the channel the Mux is currently set to.
*
* Inputs:
* none
*
* Returns:
* a byte that holds a number from 0x8 (Ch0) to 0xF (Ch7) representing the current channel
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the Mux
* - Mux has only one internal register, so no need to specify internal address
* - Tell TWI0 to read (not write)
* - Set start and stop bits in TWI0 control register (single bit read)
* - Wait for data to be received into RHR
* - Load received data into local variable
* - Wait for transmission to finish before exiting function and returning value
*
*/
uint8_t twi0ReadMuxChannel(void)
{
	uint8_t returnVal;
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(TWI0_MUX_ADDR);	//Slave address (eg. Mux or Fast Charge Chip)
	twi0RegAddrSize(0);				//Set single internal device register
	twi0SetReadMode;				//Master read direction = 1
	twi0StartSingle;				//Send a START|STOP bit as required (single byte read)
	while(!twi0RxReady);			//While Receive Holding Register not ready. wait.
	returnVal = twi0Receive;		//Store data received (the lower byte of 16-bit data)
	while(!twi0TxComplete);			//Wait for transmission complete
	return returnVal;
}

/*
* Function:
* void twi0Write(uint8_t slaveAddress, uint8_t intAddress, uint8_t data)
*
* Will write a byte on TWI0 to the slave device and internal register specified in the parameters
*
* Inputs:
* uint8_t slaveAddress:
*    The address of the slave device on TWI0 to write to
* uint8_t intAddress:
*    The internal address of the register to write to
* uint8_t data:
*    The byte to write
*
* Returns:
* none
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the slave device
* - Set TWI0 to write to a single byte internal address
* - Tell TWI0 the internal address to write to on the slave
* - Load data byte into the transmit holding register for transmission and set STOP bit (one byte
*   transmission)
* - Wait for transmission to complete before exiting function.
*
*/
void twi0Write(uint8_t slaveAddress, uint8_t intAddress, uint8_t data)
{
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(slaveAddress);		//Slave address (eg. Mux or Fast Charge Chip)
	twi0RegAddrSize(1);				//Set one-byte internal device address
	//Master write direction default by 0
	twi0RegAddr(intAddress);		//Set up the address to write to
	twi0Send(data);					//Load the transmit holding register with data to send 
	twi0Stop;						//Set the STOP bit
	while(!twi0TxReady);			//while Transmit Holding Register not ready. wait.
	while(!twi0TxComplete);			//while transmit not complete. wait.
}

/*
* Function:
* uint8_t twi0ReadSingle(uint8_t slaveAddress, uint8_t intAddress)
*
* Will read a single byte from a TWI slave device that has 8bit internal register addresses
*
* Inputs:
* uint8_t slaveAddress:
*    The address of the slave device on TWI0 to read from
* uint8_t intAddress:
*    The internal address of the register to read from
*
* Returns:
* a byte containing the contents of the internal register on the slave device specified.
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the slave device
* - Set TWI0 to read from an 8bit internal address
* - Tell TWI0 to read (not write)
* - Tell TWI0 the internal address to read from on the slave
* - Set start and stop bits in TWI0 control register (single bit read)
* - Wait for data to be received into RHR
* - Load received data into local variable
* - Wait for transmission to finish before exiting function and returning value
*
*/
uint8_t twi0ReadSingle(uint8_t slaveAddress, uint8_t intAddress)
{
	uint8_t data;
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(slaveAddress);		//Slave address (eg. Mux or Fast Charge Chip)
	twi0RegAddrSize(1);				//Set one-byte internal device address
	twi0SetReadMode;				//Master read direction = 1
	twi0RegAddr(intAddress);		//Set up device internal address to read from
	twi0StartSingle;				//Send a START|STOP bit as required (single byte read)
	while(!twi0RxReady);			//While Receive Holding Register not ready. wait.
	data = twi0Receive;				//Store data received (the lower byte of 16-bit data)
	while(!twi0TxComplete);			//Wait for transmission complete
	return data;
}

/*
* Function:
* uint8_t twi0ReadDouble(uint8_t slaveAddress, uint8_t intAddress)
*
* Will read two bytes from a TWI slave device that has 8bit internal register addresses
*
* Inputs:
* uint8_t slaveAddress:
*    The address of the slave device on TWI0 to read from
* uint8_t intAddress:
*    The internal address of the register to read from
*
* Returns:
* a 16bit integer containing the contents of the internal register on the slave device specified.
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the slave device
* - Set TWI0 to read from an 8bit internal address
* - Tell TWI0 to read (not write)
* - Tell TWI0 the internal address to read from on the slave
* - Set start bit in TWI0 control register (begin multi-byte read)
* - Wait for first byte to be received into RHR
* - Load first byte into local variable
* - Set STOP bit in TWI0 control reg. (Must be set before receiving last byte).
* - Wait for second byte to be received into RHR
* - Load second byte into local variable
* - Load upper and lower bytes into 16bit integer and return
*
* Improvements:
* Eliminate data1 and data2 (just have returnVal)
*
*/
uint16_t twi0ReadDouble(uint8_t slaveAddress, uint8_t intAddress)
{
	uint8_t dataL, dataH;
	uint16_t returnVal;
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(slaveAddress);		//Slave address (eg. Mux or Fast Charge Chip)
	twi0RegAddrSize(1);				//Set one-byte internal device address
	twi0SetReadMode;				//Master read direction = 1
	twi0RegAddr(intAddress);		//Set up device internal address to read from
	twi0Start;						//Send a START bit as required (multi-byte read)
	while(!twi0RxReady);			//While Receive Holding Register not ready. wait.
	dataL = twi0Receive;			//Store data received (the lower byte of 16-bit data)
	twi0Stop;						//Set STOP bit as required
	while(!(twi0RxReady));			//While Receive Holding Register not ready. wait.
	dataH = twi0Receive;			//Store data received (the upper byte of 16-bit data)
	while(!twi0TxComplete);			//While transmit not complete. wait.
	returnVal = (dataH << 8) | dataL;//Puts the two 8 bits into 16 bits
	return returnVal;
}
