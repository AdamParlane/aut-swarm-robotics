/*
* prox_sens_interface.h
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 10:05:52 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Register and command defines for proximity sensors and function definitions for accessing prox-
* imity sensors
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* APDS-9930 Proximity sensor Datasheet:https://docs.broadcom.com/docs/AV02-3190EN
*
* Register address format:
* Register addresses are only up to 5bits long. The remaining 3bits are used to determine the method
* of data transfer to and from the sensor.
*  bit   7     6     5     4     3     2     1     0
*     +-----+-----+-----+-----+-----+-----+-----+-----+
*     | CMD |    TYPE   |          Address            |
*     +-----+-----+-----+-----+-----+-----+-----+-----+
* CMD should always be 1. TYPE determines the mode of transfer. a value of 0b00 in TYPE means that
* each consecutive read or write to a register address will be to/from that same address. A value
* of 0b01 for TYPE will have the register address increment on each consecutive read/write. This
* simplifies the task of reading long integer values.
*
* Functions:
* void proxSensInit(uint8_t channel)
* uint16_t proxSensRead(uint8_t channel)
*
*/

#ifndef PROX_SENS_INTERFACE_H_
#define PROX_SENS_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "sam.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//Register addresses for reading data. (NOTE: Ch0 and Ch1 are two 16-bit registers)
#define PS_CH0DATAL_REG	0x14	//Ch0 photodiode ADC low data register (Visible+IR light)
#define PS_CH0DATAH_REG	0x15	//Ch0 photodiode ADC high data register (Visible+IR light)
#define PS_CH1DATAL_REG	0x16	//Ch1 photodiode ADC low data register (IR only)
#define PS_CH1DATAH_REG	0x17	//Ch1 photodiode ADC high data register (IR only)
#define PS_PDATAL_REG	0x18	//Proximity ADC low data register
#define PS_PDATAH_REG	0x19	//Proximity ADC high data register
#define PS_STATUS_REG	0x13	//Device status

//Register addresses for configuration, all R/W
#define PS_ENABLE_REG	0x00	//Enable of states and interrupts
#define PS_PTIME_REG	0x02	//Proximity ADC integration time (LPF)
#define PS_WTIME_REG 	0x03	//Wait time
#define PS_CONFIG_REG	0x0D	//Configuration
#define PS_PPULSE_REG	0x0E	//Proximity pulse count register
#define PS_GAINCTL_REG	0x0F	//Gain Control register (Gain Defaults to recommended 1x at powerup)
#define PS_OFFSET_REG	0x1E	//Proximity offset register

//These are OR'd to the address of the desired register to determine how the register should be read
#define PS_CMD_1BYTE	0x80	//Sets read/write protocol, (repeated byte, reads/writes to the same
								//register)
#define PS_CMD_INC		0xA0	//Sets read/write protocol, (auto increment registers to read
								//successive bytes)
								
//Command Codes for proximity sensor register config
#define PS_WTIME_INIT	0xFF	//2.73 ms, minimum Wait time
#define PS_PTIME_INIT	0xFF	//2.73 ms, minimum proximity integration time (recommended).
								//!!ACCORDING to the datasheet, this should be set to at least 50ms
								//or a greater multiple of 50ms in order to filter out 50/60hz
								//flicker that is present in fluorescent lighting. (see Pg 9)
#define PS_PPULSE_INIT	0x08	//Recommended proximity pulse count is 8 Pulses
#define PS_PDIODE_INIT	0x20	//LED = 100mA, Proximity diode select, Proximity gain x1,
								//recommended settings
#define PS_ENABLE_STATE	0x05	//Power ON, Proximity Enable
#define PS_DISABLE_STATE 0x00	//Power OFF, Proximity Disable

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void proxSensInit(uint8_t channel)
*
* This function will pass the desired channel to the Multiplexer and setup an *individual* proximity
* sensor
*
* Inputs:
* uint8_t channel:
*    The I2C multiplexer channel of the desired proximity sensor
*
* Returns:
* none
*
*/
void proxSensInit(uint8_t channel);

/*
* Function:
* uint16_t proxSensRead(uint8_t channel)
*
* Retrieves the Proximity Sensor (16-bit) data from the selected sensor.
*
* Inputs:
* uint8_t channel:
*    The I2C multiplexer channel of the desired proximity sensor
*
* Returns:
* 16bit long integer containing the value of the proximity ADC
*
*/
uint16_t proxSensRead(uint8_t channel);

#endif /* PROX_SENS_INTERFACE_H_ */