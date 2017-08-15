/*
* adc_interface.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:31:00 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains functions to initialise the ADC and to read a sample from the given ADC channel.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void adcSingleConvInit(void)
* uint16_t adcRead(uint8_t channel)
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "adc_interface.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void adcSingleConvInit(void)
*
* Initialises ADC peripheral into single conversion mode.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Writes write protect key to 'Write Protect Mode Register' to enable reconfiguration of the ADC
* - Enables the peripheral clock signal to the ADC
* - Configures the ADC mode register for 1MHz ADC clock speed, 24 ADC clock cycle start up time and
*   sets the transfer field to 2, which is the optimal setting as per the datasheet (pg1101)
*
*/
void adcSingleConvInit(void)
{
	REG_ADC_WPMR
	=	0x41444300;						//Disable ADC write protect
	REG_PMC_PCER0
	|=	(1<<ID_ADC);					//Enable peripheral clock on ADC
	REG_ADC_MR							//ADC mode register
	|=	ADC_MR_PRESCAL(49)				//Prescale ADC conversion by 49 (100MHZ/((49+1)x2))=1MHZ.
	|	ADC_MR_STARTUP_SUT24			//Startup time is 24 ADC clock cycles.
	|	ADC_MR_TRANSFER(2);				//Transfer field must be programmed with value 2.
	//REG_ADC_ACR						//Analogue control register. Sets up internal voltage ref.
										////Seems to be unnecessary.
	//=	ADC_ACR_IRVCE					//Internal ref voltage change enable
	//|	ADC_ACR_IRVS(IRVS_3396)			//Mode 8 is 3.396V. If changed be sure to set 
										////ADC_VOLTAGE_REF appropriately
	//|	ADC_ACR_ONREF;					//Internal reference voltage is on
}

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
* 10bit value of the ADC channel in question (0-1023)
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
	//TODO: Needs waitForFlag here
	while(!adcDataReady);		//Wait for DRDY flag
	adcDisableChan(channel);	//Disable channel on completion
	return adcData;				//Read data from last converted data register and return
}