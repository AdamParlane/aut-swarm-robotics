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
* Approx Proximity Values
* Using my hand as an object, testing on side A
* touching - ~5cm = 0x03ff (max)
* 5 cm away 0x0150 - 0x01ff
* 10cm away 0x0070 - 0x0100
* 20cm away 0x0030 - 0x003f
* 30cm away 0x0020 - 0x0029
*
* Functions:
* void proxSensInit(void)
* void proxSingleSensInit(uint8_t channel)
* uint16_t proxSensRead(uint8_t channel)
* uint16_t proxAmbRead(uint8_t channel)
* void proxAmbModeEnabled(void)
* void proxModeEnabled(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "prox_sens_interface.h"
#include "twimux_interface.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void proxSensInit(void)
*
* Initialise all proximity sensors. Tidies up setup() function
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* See description for proxSingleSensInit() below
*
*/
void proxSensInit(void)
{
	proxSingleSensInit(MUX_PROXSENS_A);		//Initialise proximity sensor on panel A
	proxSingleSensInit(MUX_PROXSENS_B);		//Initialise proximity sensor on panel B
	proxSingleSensInit(MUX_PROXSENS_C);		//Initialise proximity sensor on panel C
	proxSingleSensInit(MUX_PROXSENS_D);		//Initialise proximity sensor on panel D
	proxSingleSensInit(MUX_PROXSENS_E);		//Initialise proximity sensor on panel E
	proxSingleSensInit(MUX_PROXSENS_F);		//Initialise proximity sensor on panel F
}

/*
* Function:
* void proxSingleSensInit(uint8_t channel)
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
void proxSingleSensInit(uint8_t channel)
{
	uint8_t writeBuffer;
	//Set multiplexer address to correct device
	twi0MuxSwitch(channel);
	//Disable and Power down
	writeBuffer = PS_DISABLE_ALL;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ENABLE_REG, 1, &writeBuffer);
	//Proximity ADC time: 2.73 ms, minimum proximity integration time
	writeBuffer = PS_PTIME_INIT;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_PTIME_REG, 1, &writeBuffer);
	//Program the ambient light integration time
	writeBuffer = PS_ATIME_INIT;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ATIME_REG, 1, &writeBuffer);
	//Program Wait time
	writeBuffer = PS_WTIME_INIT;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_WTIME_REG, 1, &writeBuffer);
	//Sets the number of Proximity pulses that the LDR pin will generate during the prox Accum
	//state: (recommended proximity pulse count = 8) PREVIOUSLY HAD BEEN SET TO 0X02
	writeBuffer = PS_PPULSE_INIT;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_PPULSE_REG, 1, &writeBuffer);
	//Gain Control register: LED = 100mA, Proximity diode select, Proximity gain x1, recommended
	//settings
	writeBuffer = PS_PDIODE_INIT;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_GAINCTL_REG, 1, &writeBuffer);
	//Power ON, Proximity Enable
	writeBuffer = PS_ENABLE_PROX;
	twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ENABLE_REG, 1, &writeBuffer);
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
*/
uint16_t proxSensRead(uint8_t channel)
{
	unsigned char data[2];
	twi0MuxSwitch(channel);	//Set multiplexer address to correct device
	twi0Read(TWI0_PROXSENS_ADDR, (PS_CMD_INC | PS_PDATAL_REG), 2, data);
	//NOTE: Command_REG of the ProxSensor must be written to, as part of R/W functions.
	//Low data register is read, auto-increment occurs and high data register is read.
	return (data[1]<<8)|(data[0]);
}

/*
* Function:
* uint16_t proxAmbRead(uint8_t channel)
*
* Retrieves the Ambient light (16-bit) data from the selected proximity sensor.
*
* Inputs:
* uint8_t channel:
*    The I2C multiplexer channel of the desired proximity sensor
*
* Returns:
* 16bit long integer containing the value proportional to lux hitting sensor
*
* Implementation:
* Two 16bit values are read from the given sensor, one from each of the two photodiodes on board.
* The channel 0 value contains data on both visible light and infrared light hitting the sensor.
* Channel 1 give just amount of IR light. Then a value that is supposed to relfect human eye
* brightness response is calculated using formulas found in the datasheet (pg9).
* 
*/
uint16_t proxAmbRead(uint8_t channel)
{
	unsigned char data[2];	//TWI data retrieval buffer
	twi0MuxSwitch(channel);	//Set multiplexer address to correct device
	twi0Read(TWI0_PROXSENS_ADDR, (PS_CMD_INC | PS_CH0DATAL_REG), 2, data);
	uint16_t ch0Data = (data[1]<<8)|(data[0]);	//Retrieve CH0 light data (Visible+IR)
	twi0Read(TWI0_PROXSENS_ADDR, (PS_CMD_INC | PS_CH1DATAL_REG), 2, data);
	uint16_t ch1Data = (data[1]<<8)|(data[0]);	//Retrieve CH1 light data (IR only)
	//NOTE: Command_REG of the ProxSensor must be written to, as part of R/W functions.
	//Low data register is read, auto-increment occurs and high data register is read.
	
	//Equations for canceling IR light (Datasheet pg9):
	float IAC1 = (float)ch0Data - 1.862*(float)ch1Data;
	float IAC2 = 0.764*(float)ch0Data - 1.291*(float)ch1Data;
	
	if(IAC1 > IAC2)		//The largest value is the one we want
		return (uint16_t)IAC1;
	else
		return (uint16_t)IAC2;
}

/*
* Function:
* void proxAmbModeEnabled(void)
*
* Enables ambient light mode and disables proximity mode on the proximity sensors.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* A for loop steps through each proximity sensor mux address and writes out the PS_ENABLE_AMBI
* command to the Enable register on each sensor.
*
*/
void proxAmbModeEnabled(void)
{
	uint8_t writeBuffer = PS_ENABLE_AMBI;
	//Enable ambient light mode on all the sensors
	for(int ch = MUX_PROXSENS_A; ch <= MUX_PROXSENS_B; ch++)
	{
		twi0MuxSwitch(ch);	//Switch to next prox sensor
		//Enable the ambient sensor and disable proximity
		twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ENABLE_REG, 1, &writeBuffer);		
	}
	//Wait for the first reading
	delay_ms(53); //(50ms ATIME + 2.73ms WTIME)	
}

/*
* Function:
* void proxModeEnabled(void)
*
* Enables proximity detection mode and disables ambient light mode on the proximity sensors.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* A for loop steps through each proximity sensor mux address and writes out the PS_ENABLE_PROX
* command to the Enable register on each sensor.
*
*/
void proxModeEnabled(void)
{
	uint8_t writeBuffer = PS_ENABLE_PROX;
	//Enable ambient light mode on all the sensors
	for(int ch = MUX_PROXSENS_A; ch <= MUX_PROXSENS_B; ch++)
	{
		twi0MuxSwitch(ch);
		//Enable the ambient sensor and disable proximity
		twi0Write(TWI0_PROXSENS_ADDR, PS_CMD_1BYTE | PS_ENABLE_REG, 1, &writeBuffer);
	}
	//Wait for the first reading
	delay_ms(6); //(2.74ms PTIME + 2.73ms WTIME)
}