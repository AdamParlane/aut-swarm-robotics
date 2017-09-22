/*
* rgbled_driver.c
*
* Author : Jacob Bullot (email)
* Created: 22/09/2017 2:42:29 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Header for the RGB LED driver
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t rgbDriverWrite(uint8_t addrbits, uint8_t cntlbits)
* void rgbLedReset(uint8_t channel)
* void rgbLedResetAll(void)
* void rgbLedOn(uint8_t channel, uint8_t colour, uint8_t brightnessPercentage)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "twimux_interface.h"
#include "rgbled_driver.h"


//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/* for rgb_led_interface.c */

/*
* Function:
* uint8_t rgbDriverWrite(uint8_t channel);
*
* Writes to the single programmable register on the RGB LED driver.
*
* Inputs:
* uint8_t addrbits:
*   Bits [7:5] of the internal register which dictates the function of the control bits [4:0]
*	RGB_CHIP_SHUT_DOWN_CMD - Sets all LED current to zero, control bits [4:0] don't matter.
*					 DOES NOT RESET OTHER REGISTER VALUES.
*	RGB_OUTPUT_CURRENT_CMD - Sets up the maximum possible LED current
*	RGB_GREEN_PWM_CMD - Sets brightness of green LED (PWM1)
*	RGB_BLUE_PWM_CMD - Sets brightness of blue LED (PWM2)
*	RGB_RED_PWM_CMD - Sets brightness of red LED (PWM3)
*	RGB_DIMMING_UPWARD_CMD - Sets the end point for upwards dimming. SET BEFORE WRITING TO DIMMING TIME
*	RGB_DIMMING_DOWNWARD_CMD - Sets the end point for downwards dimming. SET BEFORE WRITING TO DIMMING TIME
*	RGB_DIMMING_TIME_CMD - Sets how long each of the dimming steps will take. THIS BEGINS THE DIMMING.
* uint8_t cntlbits:
*	Bits [4:0] of the internal register which dictate the value being written
*	RGB_MAX_OUTPUT_CURRENT_CMD - Set maximum LED current to 8.73mA. (Blue has a limit of 10mA)
*	RGB_OFF_PWM	- PWM = 0%
*	RGB_HALF_PWM - PWM = 50%
*	RGB_ON_PWM - PWM = 100%
*
* Returns:
* non zero on error
*
* Implementation:
* - Enable TWI0 as bus master
* - Tell TWI0 the address of the RGB Driver
* - RGB driver has only one internal register, so no need to specify internal address
* - Or the address and control bits together to create the data byte to be transmitted. Load data
*	byte into the transmit holding register for transmission and set STOP bit (one byte transmission)
* - Wait for transmission to complete before exiting function.
*/
uint8_t rgbDriverWrite(uint8_t addrbits, uint8_t cntlbits)
{
	twi0MasterMode;					//Master mode enabled, slave disabled
	twi0SetSlave(TWI0_RGBDRIVER_ADDR);	//RGB driver slave address
	//No internal address and set to master write mode by default of zero
	
	
	twi0Send(addrbits|cntlbits);	//Load THR and writing to THR causes start to be sent
	twi0Stop;						//Set STOP bit after tx
	//wait for start and data to be shifted out of holding register
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXRDY, TWI_TXRDY_TIMEOUT))
	return 1;
	//Communication complete, holding and shifting registers empty, Stop sent
	if(waitForFlag((uint32_t*)&REG_TWI0_SR, TWI_SR_TXCOMP, TWI_TXCOMP_TIMEOUT))
	return 1;
	else
	{
		return 0;
	}
}

/*
* Function:
* void rgbLedReset(uint8_t channel)
*
* Resets the RGB LED to an off state where it is ready to accept new PWM values. Also used to turn
*	a LED off.
*
* Inputs:
* uint8_t channel:
*   The I2C multiplexer channel for the desired RGB LED (see twimux_interface.h). Only controls
*   one side panel at a time.
*
* Returns:
* none
*
* Implementation:
* - Sets the multiplexer address to correct side panel
* - Turns all 3 colours to off by setting PWM duty cycle to zero.
* - Sets output current to maximum of 8.73mA. Note LED is still off until a PWM is set above 0.
*
*/
void rgbLedReset(uint8_t channel)
{
	twi0MuxSwitch(channel);	//Set multiplexer address to correct side panel
	rgbDriverWrite(RGB_RED_PWM_CMD, RGB_OFF_PWM); //Red LED off
	rgbDriverWrite(RGB_BLUE_PWM_CMD, RGB_OFF_PWM); //Blue LED off
	rgbDriverWrite(RGB_GREEN_PWM_CMD, RGB_OFF_PWM); //Green LED off
	rgbDriverWrite(RGB_OUTPUT_CURRENT_CMD, RGB_MAX_OUTPUT_CURRENT_CMD); //Set output current to maximum of 8.73mA
}

/*
* Function:
* void rgbLedResetAll(void)
*
* Resets all RGB LEDs to an off state where they are ready to accept new PWM values. Also used to
*	turn all RGB LEDs off.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* - Sequentially runs the reset of each RGB LED.
*
*/
void rgbLedResetAll(void)
{
	rgbLedReset(MUX_RGBDRIVER_B);
	rgbLedReset(MUX_RGBDRIVER_C);
	rgbLedReset(MUX_RGBDRIVER_D);
	rgbLedReset(MUX_RGBDRIVER_E);
	rgbLedReset(MUX_RGBDRIVER_F);
}

/*
* Function:
* void rgbLedOn(uint8_t channel, uint8_t colour, uint8_t brightnessPercentage)
*
* Turns a RGB LED to either red, green, blue or white at the user defined brightness
*
* Inputs:
* uint8_t channel:
*   The I2C multiplexer channel for the desired RGB LED (see twimux_interface.h). Only controls
*   one side panel at a time.
* uint8_t colour
*	Selects RGB LED colour. Must use RGB_RED, RGB_GREEN, RGB_BLUE or RGB_WHITE.
* uint8_t brightnessPercentage
*	Enter the decimal value of the brightness percentage desired
*
* Returns:
* none
*
* Implementation:
* - Resets any previous register values held by the driver. This step also sets the MUX address for
*	the remainder of the function.
* - The PWM control bits can only accept a value up to 31. Setting the brightnessPercentage to a max
*	of 93 ensures this is never exceeded.
* - Switch goes into the case of the selected colour. brightnessPercentage is divided by three so it
*	fits into the control bits.
*
*/
void rgbLedOn(uint8_t channel, uint8_t colour, uint8_t brightnessPercentage)
{
	rgbLedReset(channel);	//Resets any previous register values held by the driver.
	
	//Ensures brightnessPercentage will not exceed 31 after division
	if (brightnessPercentage > 93)
	{
		brightnessPercentage = 93;
	}
	
	switch (colour)
	{
		case RGB_RED:
		rgbDriverWrite(RGB_RED_PWM_CMD, (brightnessPercentage/3));
		rgbDriverWrite(RGB_GREEN_PWM_CMD, RGB_OFF_PWM);
		rgbDriverWrite(RGB_BLUE_PWM_CMD, RGB_OFF_PWM);
		break;
		
		case RGB_GREEN:
		rgbDriverWrite(RGB_RED_PWM_CMD, RGB_OFF_PWM);
		rgbDriverWrite(RGB_GREEN_PWM_CMD, (brightnessPercentage/3));
		rgbDriverWrite(RGB_BLUE_PWM_CMD, RGB_OFF_PWM);
		break;
		
		case RGB_BLUE:
		rgbDriverWrite(RGB_RED_PWM_CMD, RGB_OFF_PWM);
		rgbDriverWrite(RGB_GREEN_PWM_CMD, RGB_OFF_PWM);
		rgbDriverWrite(RGB_BLUE_PWM_CMD, (brightnessPercentage/3));
		break;
		
		case RGB_WHITE:
		rgbDriverWrite(RGB_RED_PWM_CMD, (brightnessPercentage/6));
		rgbDriverWrite(RGB_GREEN_PWM_CMD, (brightnessPercentage/6));
		rgbDriverWrite(RGB_BLUE_PWM_CMD, (brightnessPercentage/6));
		break;
	}
}