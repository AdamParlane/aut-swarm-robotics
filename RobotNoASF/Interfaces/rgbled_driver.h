/*
* rgbled_driver.h
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

#ifndef RGBLED_DRIVER_H_
#define RGBLED_DRIVER_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
/* for twimux_interface.h */
#define TWI0_RGBDRIVER_ADDR			0x38			//RGB LED driver address
/* MUX CHANNEL ADDRESSES FOR RGB DRIVERS */
#define MUX_RGBDRIVER_B				0xFF			//Mux Channel 7, Side Panel B
#define MUX_RGBDRIVER_C				0xFE			//Mux Channel 6, Side Panel C
#define MUX_RGBDRIVER_D				0xFD			//Mux Channel 5, Side Panel D
#define MUX_RGBDRIVER_E				0xFC			//Mux Channel 4, Side Panel E
#define MUX_RGBDRIVER_F				0xFB			//Mux Channel 3, Side Panel F

// Address Bits [7:5] for RGB driver
#define RGB_CHIP_SHUT_DOWN_CMD		0x00			//All LED Current = Zero. DOES NOT RESET REGISTER VALUES.
#define RGB_OUTPUT_CURRENT_CMD		0x20			//Set up the maximum Output LED Current
#define RGB_GREEN_PWM_CMD			0x40			//PWM1 : LED1 Control
#define RGB_BLUE_PWM_CMD			0x60			//PWM2 : LED2 Control
#define RGB_RED_PWM_CMD				0x80			//PWM3 : LED3 Control
#define RGB_DIMMING_UPWARD_CMD		0xA0			//Set the Upward Current End Target
#define RGB_DIMMING_DOWNWARD_CMD	0xC0			//Set the Downward Current End Target
#define RGB_DIMMING_TIME_CMD		0xE0			//Set the number of steps and activate gradual dimming
// Control Bits [4:0] for RGB driver
#define RGB_OFF_PWM						0x00			//PWM = 0%
#define RGB_HALF_PWM					0x0F			//PWM = 50%
#define RGB_ON_PWM						0x1F			//PWM = 100%
#define RGB_MAX_OUTPUT_CURRENT_CMD		0x1C			//Set max continuous current @ 9.2mA (Blue has a limit of 10mA)
// RGB LED on function case values
#define RGB_RED		1
#define RGB_GREEN	2
#define RGB_BLUE	3
#define RGB_WHITE	4

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
*/
uint8_t rgbDriverWrite(uint8_t addrbits, uint8_t cntlbits);

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
*/
void rgbLedReset(uint8_t channel);

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
*/
void rgbLedResetAll(void);

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
*/
void rgbLedOn(uint8_t channel, uint8_t colour, uint8_t brightnessPercentage);

#endif /* RGBLED_DRIVER_H_ */