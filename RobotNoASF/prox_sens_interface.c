/*
* prox_sens_interface.c
*
* Author : Esmond Mather and Matthew Witt
* Created: 11/07/2017 11:11:00 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains functions for initialising and accessing the proximity sensors.
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

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "prox_sens_interface.h"
#include "twimux_interface.h"

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
* Implementation:
* Will write out default values to the proximity sensor connected to the given multiplexer channel.
* First command powers down the sensor. Next, the integration time of the ADC is set. This filters
* high frequency noise. Next, the number of times the IR led will be pulsed for a proximity reading
* is set. Signal strength increases proportionally to the number of pulses while noise level in-
* creases proportional to the square root of the numebr of pulses. Next the LED current setting
* is set as well as the photo diode (CH1 IR only?). Lastly, the sensor is re-enabled.
*
* Improvements:
* Could this function just initialise all of the sensors?
*
*/
void proxSensInit(uint8_t channel)
{
	//Set multiplexer address to correct device
	twi0MuxSwitch(channel);
	//Disable and Power down
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ENABLE_REG, PS_DISABLE_STATE);
	//Proximity ADC time: 2.73 ms, minimum proximity integration time
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_PTIME_REG, PS_PTIME_INIT);
	//Sets the number of Proximity pulses that the LDR pin will generate during the prox Accum
	//state: (recommended proximity pulse count = 8) PREVIOUSLY HAD BEEN SET TO 0X02
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_PPULSE_REG, PS_PPULSE_INIT);
	//Gain Control register: LED = 100mA, Proximity diode select, Proximity gain x1, recommended
	//settings
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_GAINCTL_REG, PS_PDIODE_INIT);
	//Power ON, Proximity Enable
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ENABLE_REG, PS_ENABLE_STATE);
}

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
* Implementation:
* After the programmed number of proximity pulses have been generated, the proximity ADC converts
* and scales the proximity measurement to a 16-bit value, then stores the result in two 8-bit 
* proximity data (PDATAx) registers. Therefore, the TWI must read/retrieve both 8-bit registers.
*
* Improvements:
* Eliminate data variable.
*
*/
uint16_t proxSensRead(uint8_t channel)
{
	uint16_t data;
	twi0MuxSwitch(channel);	//Set multiplexer address to correct device
	data = twi0ReadDouble(TWI0_PROXSENS_ADDR, PS_CMD_INC | PS_PDATAL_REG);
	//NOTE: Command_REG of the ProxSensor must be written to, as part of R/W functions.
	//Low data register is read, auto-increment occurs and high data register is read.
	return data;
}
