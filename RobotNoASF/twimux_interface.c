/*
 * twimux_interface.c
 *
 * Created: 11/07/2017 10:41:59 AM
 *  Author: Matthew
 */ 

#include "twimux_interface.h"

//Initialise the TWI0 hardware. No Parameters
//Update to new code format necessary
void twi0Init(void)
{
	/******** TWI0 SETUP ********/
	REG_PMC_PCER0 |= (1<<19);				//Enable clock access to TWI0, Peripheral TWI0_ID = 19
	REG_PIOA_PDR  |= (1<<3);				// Enable peripheralA control of PA3 (TWD0)
	REG_PIOA_PDR  |= (1<<4);				// Enable peripheralA control of PA4 (TWCK0)
	REG_TWI0_CR = (1<<7);					//Software Reset
	/* TWI0 Clock Waveform Setup */
	//NOTE: A high period of 0.6uSec and a low period of 1.3uSec is required by both the Proximity and Light Sensors
	uint8_t CKDIV, CHDIV, CLDIV;
	CKDIV = 1;
	CLDIV = 63;								//x=63 1.3uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
	CHDIV = 28;								//x=28, 0.6uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
	REG_TWI0_CWGR |= (CKDIV<<16);			//Clock speed 400000, fast mode
	REG_TWI0_CWGR |= (CHDIV<<8);			//Clock high period  0.6uSec
	REG_TWI0_CWGR |= (CLDIV<<0);			//Clock low period 1.3uSec
	REG_TWI0_CR |= (1<<2)|(1<<5);			//Master mode enabled, Slave disabled
	uint8_t dummy = REG_TWI0_RHR;			//Ensure RXRDY flag is cleared
}

/******** TWI0 Multiplexer Channel Select ********/
//Sets the Multiplexer to desired channel. Can put called by the user but is mostly called by other functions
void TWI0_MuxSwitch(uint8_t channel)
{
	REG_TWI0_CR |= (1<<2)|(1<<5);			//Master mode enabled, Slave disabled
	REG_TWI0_MMR = (TWI0_Mux_Address<<16);	//Set Slave Device address
	//No internal address and set to master write mode by default of zero
	REG_TWI0_THR = channel;					//Load THR and writing to THR causes start to be sent
	REG_TWI0_CR |= (1<<1);					//Set STOP bit after tx
	while(!(twi0TXRDY));					//wait for start and data to be shifted out of holding register
	while(!(twi0TXCOMP));					//Communication complete, holding and shifting registers empty, Stop sent
}

/******** TWI0 Multiplexer Channel Read ********/
//Allows read back of the currently selected Multiplexer channel
uint8_t TWI0_ReadMuxChannel(void)
{
	/*** This read function tells you the selected Mux channel (8 bits of data, no internal address) ***/
	uint8_t data;
	REG_TWI0_CR |= (1<<2)|(1<<5);			//Enable master mode and disable slave mode
	REG_TWI0_MMR = (TWI0_Mux_Address<<16);	//Device Slave address
	REG_TWI0_MMR |= (1<<12);				//Master read direction = 1
	REG_TWI0_CR = 0x3;						//Send a START|STOP bit as required (single byte read)
	while(!(twi0RXRDY));					//While Receive Holding Register not ready. wait.
	data = REG_TWI0_RHR;					//Store data received
	while(!(twi0TXCOMP));					//Wait for transmission complete
	return data;
}

/******** TWI0 Write ********/
void TWI0_Write(uint8_t SlaveAddress, uint8_t intAddress, uint8_t Data)
{
	REG_TWI0_CR |= (1<<2)|(1<<5);		//Enable master mode and disable slave mode
	REG_TWI0_MMR = (SlaveAddress<<16);	//Slave address (eg. Mux or Fast Charge Chip)
	REG_TWI0_MMR |= (1<<8);				//Set one-byte internal device address
	//Master write direction default by 0
	REG_TWI0_IADR = intAddress;			//Set up the address to write to
	REG_TWI0_THR = Data;				//Load the transmit holding register with data to send (start bit is also sent)
	REG_TWI0_CR |= (1<<1);				//Set the STOP bit
	while(!(twi0TXRDY));				//while Transmit Holding Register not ready. wait.
	while(!(twi0TXCOMP));				//while transmit not complete. wait.
}

/******** TWI0 Read - Single Byte ********/
//This read function is for peripherals with 8 bits of data and an internal address
//Note the start & stop bit must be set simultaneously
uint8_t TWI0_ReadSB(uint8_t SlaveAddress, uint8_t intAddress)
{
	uint8_t data;
	REG_TWI0_CR |= (1<<2)|(1<<5);		//Enable master mode and disable slave mode
	REG_TWI0_MMR = (SlaveAddress<<16);	//Device Slave address
	REG_TWI0_MMR |= (1<<8);				//Set one-byte internal device address
	REG_TWI0_MMR |= (1<<12);			//Master read direction = 1
	REG_TWI0_IADR = intAddress;			//Set up device internal address to read from
	REG_TWI0_CR = 0x3;					//Send a START|STOP bit as required (single byte read)
	while(!(twi0RXRDY));				//While Receive Holding Register not ready. wait.
	data = REG_TWI0_RHR;				//Store data received (the lower byte of 16-bit data)
	while(!(twi0TXCOMP));				//Wait for transmission complete
	return data;
}

/******** TWI0 Read - Double Byte ********/
//This read function is for peripherals with 16 bits of data and an internal address
uint16_t TWI0_ReadDB(uint8_t SlaveAddress, uint8_t intAddress)
{
	uint8_t data1, data2;
	uint16_t data;
	REG_TWI0_CR |= (1<<2)|(1<<5);		//Enable master mode and disable slave mode
	REG_TWI0_MMR = (SlaveAddress<<16);	//Device Slave address
	REG_TWI0_MMR |= (1<<8);				//Set one-byte internal device address
	REG_TWI0_MMR |= (1<<12);			//Master read direction = 1
	REG_TWI0_IADR = intAddress;			//Set up device internal address to read from
	REG_TWI0_CR |= (1<<0);				//Send a START bit as required (single byte read)
	while(!(twi0RXRDY));				//While Receive Holding Register not ready. wait.
	data1 = REG_TWI0_RHR;				//Store data received (the lower byte of 16-bit data)
	REG_TWI0_CR |= (1<<1);				//Set STOP bit as required
	while(!(twi0RXRDY));				//While Receive Holding Register not ready. wait.
	data2 = REG_TWI0_RHR;				//Store data received (the upper byte of 16-bit data)
	while(!(twi0TXCOMP));				//While transmit not complete. wait.
	data = (data2 << 8) | data1;		//Puts the two 8 bits into 16 bits
	return data;
}
