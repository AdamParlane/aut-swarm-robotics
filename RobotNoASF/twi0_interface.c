
#include "twi0_interface.h"


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

/******** Fast Charge Controller Registry Setup ********/
void FastChargeController_Setup(void)
{
	//Chip disable (CD) line on PB2 set to enable
	REG_PIOB_PER |= (1<<2);		//Give control of PB2 to PIOB controller
	REG_PIOB_OER |= (1<<2);		//Set PB2 as an output
	REG_PIOB_CODR |= (1<<2);	//Set PB2 to low
	TWI0_Write(TWI0_FastChargeChipAddress, controlReg, initControl);	// Ensures that CE bit is clear in case safety timer has gone off in previous charge.
	TWI0_Write(TWI0_FastChargeChipAddress, battVReg, initBattV);		// Vreg = 4.0v, input current = 2.5A
	TWI0_Write(TWI0_FastChargeChipAddress, chargeReg, initCharge);		// charge current set to max Ic=2875mA, termination current Iterm=100mA (default)
}

/******** Light Sensor Registry Setup ********/
//White Light Detection Enabled with low lux (40ms integration time), No Trigger, Auto Mode
//Only sets up one light sensor at a time, not both
void LightSensor_Setup(uint8_t channel)
{
	TWI0_MuxSwitch(channel); //Set multiplexer address to correct device
	TWI0_Write(TWI0_LightSensorAddress, LightSens_Config, LightSens_Auto_LowLux);
}

/******** Proximity Sensor Registry Setup ********/
//This function will pass the desired channel to the Multiplexer and setup an *individual* proximity sensor
void Proximity_Setup(uint8_t channel)
{
	TWI0_MuxSwitch(channel); //Set multiplexer address to correct device
	TWI0_Write(TWI0_ProximitySensorAddress, Proximity_Command_REG_1Byte | Proximity_Enable, PDisable);			//Disable and Power down
	TWI0_Write(TWI0_ProximitySensorAddress, Proximity_Command_REG_1Byte | Proximity_PTime, PTIME);				//Proximity ADC time: 2.73 ms, minimum proximity integration time
	TWI0_Write(TWI0_ProximitySensorAddress, Proximity_Command_REG_1Byte | Proximity_PPulse, PPULSE);			//Sets the number of Proximity pulses that the LDR pin will generate during the prox Accum state: (recommended proximity pulse count = 8) PREVIOUSLY HAD BEEN SET TO 0X02
	TWI0_Write(TWI0_ProximitySensorAddress, Proximity_Command_REG_1Byte | Proximity_GainControl, PDiode);		//Gain Control register: LED = 100mA, Proximity diode select, Proximity gain x1, recommended settings
	TWI0_Write(TWI0_ProximitySensorAddress, Proximity_Command_REG_1Byte | Proximity_Enable, PEnable);			//Power ON, Proximity Enable
}

/******** Proximity Sensor Data Read ********/
//Retrieves the Proximity Sensor (16-bit) data from the selected Sensor
//After the programmed number of proximity pulses have been generated, the proximity ADC converts and scales the proximity measurement to a 16-bit value,
//then stores the result in two 8-bit proximity data (PDATAx) registers. Therefore, the TWI must read/retrieve both 8-bit registers.
uint16_t Proximity_Data_Read(uint8_t channel)
{
	uint16_t data;
	TWI0_MuxSwitch(channel);	//Set multiplexer address to correct device
	data = TWI0_ReadDB(TWI0_ProximitySensorAddress, Proximity_Command_REG_Increment | Proximity_DataLow);
	//NOTE: Command_REG of the ProxSensor must be written to, as part of R/W functions.
	//Low data register is read, auto-increment occurs and high data register is read.
	return data;
}

/******** Light Sensor Data Read ********/
//Retrieves the White Light (16-bit) data from the selected Light Sensor
uint16_t LightSensor_Data_Read(uint8_t channel)
{
	uint16_t data;
	TWI0_MuxSwitch(channel);	//Set multiplexer address to correct device
	data = TWI0_ReadDB(TWI0_LightSensorAddress, LightSensorWhite);
	return data;
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

void FastChargeController_WatchDogReset(void)
{
	TWI0_Write(TWI0_FastChargeChipAddress, statusReg, watchdreset);	//Resets the FCC watch dog timer. Must be done once every 30s or else registers will reset
}