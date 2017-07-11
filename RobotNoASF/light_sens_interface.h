/*
 * light_sens_interface.h
 *
 * Created: 11/07/2017 10:06:37 AM
 *  Author: Matthew Witt
 *	Desc: All functions and defines relating to accessing the light sensors go here
 */ 



#ifndef LIGHT_SENS_INTERFACE_H_
#define LIGHT_SENS_INTERFACE_H_

#include "sam.h"

/*** Light Sensor ***/
/* Register addresses for reading Colour */
#define LightSensorRed			0x08
#define LightSensorGreen		0x09
#define LightSensorBlue			0x0A
#define LightSensorWhite		0x0B	//White light is detected at the tower for docking alignment
/* Register address for configuration */
#define LightSens_Config		0x00
/* General Commands */
#define LightSens_DetectLowLux	0x06		//40ms integration time, Trigger one time, Force Mode
#define LightSens_DetectMedLux	0x26		//80ms integration time, Trigger one time, Force Mode
#define LightSens_DetectHighLux 0x46		//160ms integration time, Trigger one time, Force Mode
#define LightSens_DetectMaxLux	0x56		//1280ms integration time, Trigger one time, Force Mode
#define LightSens_Auto_LowLux	0x00		//40ms integration time, No Trigger, Auto Mode
#define LightSens_Disable		0x03		//Force Mode, Disable Sensor

void LightSensor_Setup(uint8_t channel);
uint16_t LightSensor_Data_Read(uint8_t channel);

#endif /* LIGHT_SENS_INTERFACE_H_ */