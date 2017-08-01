/*
* adc_interface.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 25/07/2017 8:30:48 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains ADC channel defines and ADC access function prototypes.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void adcSingleConvInit(void)
* uint16_t adcRead(uint8_t channel)
*
*/

#ifndef ADC_INTERFACE_H_
#define ADC_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//Analogue to digital conversion
//	Macros
#define adcStartConv			(REG_ADC_CR |= ADC_CR_START)	//Start ADC conversion
#define adcEnableChan(value)	(REG_ADC_CHER = (1<<(value)))	//Enable ADC channel for conversion
#define adcDisableChan(value)	(REG_ADC_CHDR = (1<<(value)))	//Disable ADC channel
#define adcData					(REG_ADC_LCDR)					//Last sampled ADC value
#define adcDataReady			(REG_ADC_ISR & ADC_ISR_DRDY)	//ADC conversion complete
//	ADC channel defines
//	It appears that the line follow sensors aren't connected to ADC channels on the V1, and that the
//	IR leds are always on.
//		Line follower ADC channels version 2 robot
#if defined ROBOT_TARGET_V2
#define LF0_ADC_CH			13	// Far left
#define LF1_ADC_CH			15	// Center left
#define LF2_ADC_CH			0	// Center right
#define LF3_ADC_CH			7	// Far right
#endif
//		Fast charge chip ADC channels
#define FC_BATVOLT_ADC_CH	14	// Battery voltage level
#define FC_BATTEMP_ADC_CH	9	// Battery temperature

//	ADC Internal Reference Voltage Settings
#define IRVS_1578			0x7	//1578mV
#define IRVS_1699			0x6	//1699mV
#define IRVS_1820			0x5	//1820mV
#define IRVS_1941			0x4	//1941mV
#define IRVS_2063			0x3	//2063mV
#define IRVS_2184			0x2	//2184mV
#define IRVS_2305			0x1	//2305mV
#define IRVS_2426			0x0	//2426mV
#define IRVS_2547			0xF	//2547mV
#define IRVS_2669			0xE	//2669mV
#define IRVS_2790			0xD	//2790mV
#define IRVS_2911			0xC	//2790mV
#define IRVS_3032			0xB	//3032mV
#define IRVS_3154			0xA	//3154mV
#define IRVS_3275			0x9	//3275mV
#define IRVS_3396			0x8	//3396mV

//	ADC Reference Voltage Setting
#define ADC_VOLTAGE_REF		3396	//Reference voltage (mV) used by ADC. Must be changed if 
									//reference voltage is changed. Will be used for system wide ADC
									//voltage conversions.
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
*/
void adcSingleConvInit(void);

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
*/
uint16_t adcRead(uint8_t channel);

#endif /* ADC_INTERFACE_H_ */