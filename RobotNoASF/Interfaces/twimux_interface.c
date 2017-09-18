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
* void twi2Init(void)
* uint8_t twi0MuxSwitch(uint8_t channel)
* uint8_t twi0ReadMuxChannel(void)
* char twi0Write(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length, unsigned char const *data)
* char twi2Write(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length, unsigned char const *data)
* char twi0Read(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length,	unsigned char *data)
* char twi2Read(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length,	unsigned char *data)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "twimux_interface.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
	REG_TWI0_CWGR
	|=	TWI_CWGR_CKDIV(2)					//Clock speed 400000, fast mode
	|	TWI_CWGR_CLDIV(63)					//Clock low period 1.3uSec
	|	TWI_CWGR_CHDIV(28);					//Clock high period  0.6uSec
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
	|=	TWI_CWGR_CKDIV(2)					//Clock speed 400000, fast mode
	|	TWI_CWGR_CLDIV(63)					//Clock low period 1.3uSec
	|	TWI_CWGR_CHDIV(28);					//Clock high period  0.6uSec
	twi2MasterMode;							//Master mode enabled, slave disabled
}

/*
* Function:
* uint8_t twi0MuxSwitch(uint8_t channel);
*
* Sets the I2C multiplexer to desired channel.
*
* Inputs:
* uint8_t channel:
*   Mux channel number 0x8 (Ch0) to 0xF (Ch7)
*
* Returns:
* non zero on error
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
uint8_t twi0MuxSwitch(uint8_t channel)
{
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(TWI0_MUX_ADDR);	//Slave address (eg. Mux or Fast Charge Chip)
	//No internal address and set to master write mode by default of zero
	twi0Send(channel);				//Load THR and writing to THR causes start to be sent
	twi0Stop;						//Set STOP bit after tx
	//wait for start and data to be shifted out of holding register
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
		return 1;
	//Communication complete, holding and shifting registers empty, Stop sent
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
		return 1;
	else
	{
		return 0;
	}
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
* A byte that holds a number from 0x8 (Ch0) to 0xF (Ch7) representing the current channel.
* Will return 0x01 on error
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
	//While Receive Holding Register not ready. wait.
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
		return 1;
	returnVal = twi0Receive;		//Store data received 
	//Wait for transmission complete
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
		return 1;
	return returnVal;
}

/*
* Function: char twiNWrite(unsigned char slave_addr, unsigned char reg_addr,
*								unsigned char length, unsigned char const *data)
*
* Writes bytes out to TWI devices. Allows multiple bytes to be written at once if desired
*
* Inputs:
* slave_addr is the address of the device to be written to on TWIn. reg_addr is the
* 8bit address of the register being written to. length is the number of bytes to be written. *data
* points to the data bytes to be written.
*
* Returns:
* returns 0 on success. otherwise will return 1 on timeout
*
* Implementation:
* Master mode on TWIn is enabled, TWIn is prepared for transmission ie slave and register addresses
* are set and register address size is set to 1 byte. Next, transmission takes place but there are
* slightly different procedures for single and multi byte transmission. On single byte
* transmission, the STOP state is set in the TWI control register immediately after the byte to be
* sent is loaded into the transmission holding register. On multi-byte transmission, the STOP
* flag isn't set until all bytes have been sent and the transmission holding register is clear.
*
*/
char twi0Write(unsigned char slave_addr, unsigned char reg_addr,
					unsigned char length, unsigned char const *data)
{
	if(length == 0)						//Make sure length is valid
		length = 1;
	//note txcomp MUST = 1 before writing (according to datasheet)
	twi0MasterMode;								//Enable master mode
	twi0SetSlave(slave_addr);					//Slave device address
	twi0RegAddrSize(1);							//Set register address length to 1 byte
	twi0RegAddr(reg_addr);						//set register address to write to

	if(length == 1)
	{
		twi0Send(data[0]);						//set up data to transmit
		twi0Stop;								// Send a stop bit
		//while Transmit Holding Register not ready. wait.
		if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
			return 1;
	} else {
		for(unsigned char b = 0; b < length; b++)//Send data bit by bit until data length is reached
		{
			twi0Send(data[b]);					//set up data to transmit
			//while Transmit Holding Register not ready. wait.
			if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
				return 1;
		}
		twi0Stop;								// Send a stop bit
	}
	//while transmit not complete. wait.
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
		return 1;
	else
		return 0;
}

char twi2Write(unsigned char slave_addr, unsigned char reg_addr,
				unsigned char length, unsigned char const *data)
{
	if(length == 0)						//Make sure length is valid
		length = 1;
	//note txcomp MUST = 1 before writing (according to datasheet)
	twi2MasterMode;								//Enable master mode
	twi2SetSlave(slave_addr);					//Slave device address
	twi2RegAddrSize(1);							//Set register address length to 1 byte
	twi2RegAddr(reg_addr);						//set register address to write to

	if(length == 1)
	{
		twi2Send(data[0]);						//set up data to transmit
		twi2Stop;								// Send a stop bit
		//while Transmit Holding Register not ready. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
			return 1;
	} else {
		for(unsigned char b = 0; b < length; b++)//Send data bit by bit until data length is reached
		{
			twi2Send(data[b]);					//set up data to transmit
			//while Transmit Holding Register not ready. wait.
			if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
				return 1;
		}
		twi2Stop;								// Send a stop bit
	}
	//while transmit not complete. wait.
	if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
		return 1;
	else
		return 0;
}

/*
* Function: char twiNRead(unsigned char slave_addr, unsigned char reg_addr,
*								unsigned char length, unsigned char const *data)
*
* TWI interface read functions. Allows reading multiple bytes sequentially
*
* Inputs:
* slave_addr is the address of the device to be read from on TWIn. The address varies even for the
* IMU driver because the IMU and compass have different TWI slave addresses. reg_addr is the address
* of the register being read from. length is the number of bytes to be read. The IMU automatically
* increments the register address when reading more than one byte. *data points to the location in
* memory where the retrieved data will be stored.
*
* Returns:
* returns 0 on success.
*
* Implementation:
* Master mode on TWIn is enabled, TWIn is prepared for transmission ie slave and register addresses
* are set and register address size is set to 1 byte. Next, reception takes place but there are
* different procedures for single and multi byte reception. On single byte reception, the START and
* STOP flags are set simultaneously in TWIn's control register to indicate that only one byte will
* be read before communication is stopped. With multi-byte reception, the START flag is set
* initially, and the STOP flag in the control register is set when the second to last byte has been
* received (ie there will only be one byte left to receive after the STOP flag is set)
*
*/
char twi0Read(unsigned char slave_addr, unsigned char reg_addr,
					unsigned char length, unsigned char *data)
{
	if(length == 0)						//Make sure length is valid
		length = 1;
	twi0MasterMode;						//Enable master mode
	twi0SetSlave(slave_addr);			//Slave device address
	twi0SetReadMode;					//Set to read from register
	twi0RegAddrSize(1);					//Register addr byte length (0-3)
	twi0RegAddr(reg_addr);				//set up address to read from
	
	if (length == 1)					//If reading one byte, then START and STOP bits need to be
	//set at the same time
	{
		twi0StartSingle;				//Send START & STOP condition as required (single byte read)
		//while Receive Holding Register not ready. wait.
		if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
			return 1;
		data[0] = twi0Receive;			//store data received
		//while transmission not complete. wait.
		if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
			return 1;
		else
			return 0;
	} else {
		twi0Start;						//Send start bit
		for(unsigned char b = 0; b < length; b++)
		{
			if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
				return 1;
			data[b] = twi0Receive;
			if(b == length - 2)
				twi0Stop;					//Send stop on reception of 2nd to last byte
		}
		//while transmit not complete. wait.
		if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
			return 1;
	}
	return 0;
}

char twi2Read(unsigned char slave_addr, unsigned char reg_addr,
					unsigned char length, unsigned char *data)
{
	if(length == 0)						//Make sure length is valid
		length = 1;
	twi2MasterMode;						//Enable master mode
	twi2SetSlave(slave_addr);			//Slave device address
	twi2SetReadMode;					//Set to read from register
	twi2RegAddrSize(1);					//Register addr byte length (0-3)
	twi2RegAddr(reg_addr);				//set up address to read from
	
	if (length == 1)					//If reading one byte, then START and STOP bits need to be
	//set at the same time
	{
		twi2StartSingle;				//Send START & STOP condition as required (single byte read)
		//while Receive Holding Register not ready. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
		return 1;
		data[0] = twi2Receive;			//store data received
		//while transmission not complete. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
			return 1;
		else
			return 0;
	} else {
		twi2Start;						//Send start bit
		for(unsigned char b = 0; b < length; b++)
		{
			if(waitForFlag(&REG_TWI2_SR, TWI_SR_RXRDY, TWI_RXRDY_TIMEOUT))
			return 1;
			data[b] = twi2Receive;
			if(b == length - 2)
			twi2Stop;					//Send stop on reception of 2nd to last byte
		}
		//while transmit not complete. wait.
		if(waitForFlag(&REG_TWI2_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
			return 1;
	}
	return 0;
}