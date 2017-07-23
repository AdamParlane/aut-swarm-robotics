/*
* robot_defines.c
*
* Author : Adam Parlane, Matthew Witt
* Created: 6/7/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains general/miscellaneous  functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* uint16_t adcRead(uint8_t channel)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "robot_defines.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint16_t adcRead(uint8_t channel)
*
* Will read the instantaneous value of the given analogue to digital converter channel.
*
* Inputs:
* uint8_t channel
*   Channel number of the desired ADC channel (0-15)
*
* Returns:
* 12bit value of the ADC channel in question (0-4095)
*
* Implementation:
* - Sets the ADC mux channel to read
* - Start ADC conversion
* - Wait for conversion to complete
* - Disable selected ADC channel
* - Return sampled value
*
*/
uint16_t adcRead(uint8_t channel)
{
	adcEnableChan(channel);		//Enables ADC on specified channel
	adcStartConv;				//Start conversion
	while(!adcDataReady);		//Wait for DRDY flag
	adcDisableChan(channel);	//Disable channel on completion
	return adcData;				//Read data from last converted data register and return
}
