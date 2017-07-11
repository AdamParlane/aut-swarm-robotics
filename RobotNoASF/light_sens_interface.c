/*
 * light_sens_interface.c
 *
 * Created: 11/07/2017 11:17:42 AM
 *  Author: Matthew Witt
 *	Desc: Functions for initialising and accessing the light sensors
 */ 

#include "light_sens_interface.h"
#include "twimux_interface.h"

/******** Light Sensor Registry Setup ********/
//White Light Detection Enabled with low lux (40ms integration time), No Trigger, Auto Mode
//Only sets up one light sensor at a time, not both
void LightSensor_Setup(uint8_t channel)
{
	TWI0_MuxSwitch(channel); //Set multiplexer address to correct device
	TWI0_Write(TWI0_LightSensorAddress, LightSens_Config, LightSens_Auto_LowLux);
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
