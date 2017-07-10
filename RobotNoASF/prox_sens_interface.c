/*
 * prox_sens_interface.c
 *
 * Created: 11/07/2017 11:11:00 AM
 *  Author: Matthew Witt
 *	Desc: Contains functions for initialising and accessing the proximity sensors.
 */ 

#include "prox_sens_interface.h"

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
