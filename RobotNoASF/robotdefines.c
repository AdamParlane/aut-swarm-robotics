/*
 * CFile1.c
 *
 * Created: 6/07/2017 4:52:23 PM
 *  Author: adams
 */ 


#include "robotdefines.h"

/******** ADC Read Channel Read ********/
uint16_t ADC_ReadCH(uint8_t channel)
{
	uint16_t ADCdata;
	REG_ADC_CHER = (1<<channel);		//Enables ADC on specified channel
	ADCstartconversion;					//Start conversion
	while(!(REG_ADC_ISR & (1<<24)));	//Wait for DRDY flag
	REG_ADC_CHDR = (1<<channel);		//Disable channel on completion
	ADCdata = REG_ADC_LCDR;				//Read data from last converted data register
	return(ADCdata);
}
