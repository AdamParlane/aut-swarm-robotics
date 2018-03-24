/*
* camera_interface.c
*
* Author : Brae HW (bhw11@hotmail.co.nz), Matthew Witt, Mansel Jeffares
* Created: 3/04/2017 5:08:25 PM
* 
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Provides functions for controlling and moving image data from the camera
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t camReadReg(uint8_t data);
* void camWriteReg(uint8_t regAddress, uint8_t data);
* void camRegisterReset(void);
* uint8_t camSetup(void);
*
* void camInit(void);
* bool camValidID(void)
* void camHardReset(void);
* uint8_t camUpdateWindowSize(void);
* uint8_t camSetWindowSize(uint16_t hStart, uint16_t hStop, uint16_t vStart, uint16_t vStop);
* void camRead(void);
* void camChangeFormat(uint8_t type);
* void camTestPattern(CameraTestPatterns type);
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "camera_interface.h"
#include "camera_initialisation.h"
#include "camera_buffer_interface.h"
#include "twimux_interface.h"
#include "timer_interface.h"
#include <stdbool.h>

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Camera Register Set
#define GAIN_REG			0x00    // Gain lower 8 bits (rest in vref)
#define BLUE_REG			0x01    // blue gain 
#define RED_REG				0x02    // red gain 
#define VREF_REG			0x03    // Pieces of GAIN, VSTART, VSTOP 
#define COM1_REG			0x04    // Control 1 
#define		COM1_CCIR656	0x40    // CCIR656 enable 

#define BAVE_REG			0x05    // U/B Average level 
#define GbAVE_REG			0x06    // Y/Gb Average level 
#define AECHH_REG			0x07    // AEC MS 5 bits 
#define RAVE_REG			0x08    // V/R Average level 
#define COM2_REG			0x09    // Control 2 
#define		COM2_SSLEEP		0x10    // Soft sleep mode 

#define PID_REG				0x0a    // Product ID MSB 
#define VER_REG				0x0b    // Product ID LSB 

#define COM3_REG			0x0c    // Control 3 
#define		COM3_SWAP		0x40    // Byte swap 
#define		COM3_SCALEEN	0x08    // Enable scaling 
#define		COM3_DCWEN		0x04    // Enable down-sample/crop/window 

#define COM4_REG			0x0d    // Control 4 
#define COM5_REG			0x0e    // All "reserved" 
#define COM6_REG			0x0f    // Control 6 
#define AECH_REG			0x10    // More bits of AEC value 

#define CLKRC_REG			0x11    // Clock control 
#define		CLK_EXT         0x40    // Use external clock directly 
#define		CLK_SCALE       0x3f    // Mask for internal clock scale 

#define COM7_REG			0x12    // Control 7 
#define		COM7_RESET      0x80    // Register reset 
#define		COM7_FMT_MASK   0x38
#define		COM7_FMT_VGA    0x00
#define		COM7_FMT_CIF    0x20    // CIF format 
#define		COM7_FMT_QVGA   0x10    // QVGA format 
#define		COM7_FMT_QCIF   0x08    // QCIF format 
#define		COM7_RGB        0x04    // bits 0 and 2 - RGB format 
#define		COM7_YUV        0x00    // YUV 
#define		COM7_BAYER      0x01    // Bayer format 
#define		COM7_PBAYER     0x05    // "Processed bayer" 

#define COM8_REG			0x13    // Control 8 
#define		COM8_FASTAEC    0x80    // Enable fast AGC/AEC 
#define		COM8_AECSTEP    0x40    // Unlimited AEC step size 
#define		COM8_BFILT      0x20    // Band filter enable 
#define		COM8_AGC        0x04    // Auto gain enable 
#define		COM8_AWB        0x02    // White balance enable 
#define		COM8_AEC        0x01    // Auto exposure enable 

#define COM9_REG			0x14    // Control 9  - gain ceiling 
#define COM10_REG			0x15    // Control 10 
#define		COM10_HSYNC     0x40    // HSYNC instead of HREF 
#define		COM10_PCLK_HB   0x20    // Suppress PCLK on horiz blank 
#define		COM10_HREF_REV  0x08    // Reverse HREF 
#define		COM10_VS_LEAD   0x04    // VSYNC on clock leading edge 
#define		COM10_VS_NEG    0x02    // VSYNC negative 
#define		COM10_HS_NEG    0x01    // HSYNC negative 

#define HSTART_REG			0x17    // Horiz start high bits 
#define HSTOP_REG			0x18    // Horiz stop high bits 
#define VSTART_REG			0x19    // Vert start high bits 
#define VSTOP_REG			0x1a    // Vert stop high bits 
#define PSHFT_REG			0x1b    // Pixel delay after HREF 
#define MIDH_REG			0x1c    // Manuf. ID high 
#define MIDL_REG			0x1d    // Manuf. ID low 
#define MVFP_REG			0x1e    // Mirror / vflip 
#define		MVFP_MIRROR     0x20    // Mirror image 
#define		MVFP_FLIP       0x10    // Vertical flip 

#define AEW_REG				0x24    // AGC upper limit 
#define AEB_REG				0x25    // AGC lower limit 
#define VPT_REG				0x26    // AGC/AEC fast mode op region 
#define HSYST_REG			0x30    // HSYNC rising edge delay 
#define HSYEN_REG			0x31    // HSYNC falling edge delay 
#define HREF_REG			0x32    // HREF pieces 
#define TSLB_REG			0x3a    // lots of stuff 
#define		TSLB_YLAST		0x04    // UYVY or VYUY - see com13 

#define COM11_REG			0x3b    // Control 11 
#define		COM11_NIGHT     0x80    // NIght mode enable 
#define		COM11_NMFR      0x60    // Two bit NM frame rate 
#define		COM11_HZAUTO    0x10    // Auto detect 50/60 Hz 
#define		COM11_50HZ      0x08    // Manual 50Hz select 
#define		COM11_EXP       0x02

#define	COM12_REG			0x3c    // Control 12 
#define		COM12_HREF      0x80    // HREF always 

#define COM13_REG			0x3d    // Control 13 
#define		COM13_GAMMA     0x80    // Gamma enable 
#define		COM13_UVSAT     0x40    // UV saturation auto adjustment 
#define		COM13_UVSWAP    0x01    // V before U - w/TSLB 

#define COM14_REG			0x3e    // Control 14 
#define		COM14_DCWEN     0x10    // DCW/PCLK-scale enable 

#define EDGE_REG			0x3f    // Edge enhancement factor 
#define COM15_REG			0x40    // Control 15 
#define		COM15_R10F0     0x00    // Data range 10 to F0 
#define		COM15_R01FE     0x80    //            01 to FE 
#define		COM15_R00FF     0xc0    //            00 to FF 
#define		COM15_RGB565    0x10    // RGB565 output 
#define		COM15_RGB555    0x30    // RGB555 output 

#define COM16_REG			0x41    // Control 16 
#define		COM16_AWBGAIN   0x08    // AWB gain enable 

#define COM17_REG			0x42    // Control 17 
#define		COM17_AECWIN    0xc0    // AEC window - must match COM4 
#define		COM17_CBAR      0x08    // DSP Color bar 

#define CMATRIX_BASE_REG	0x4f
#define		CMATRIX_LEN		6
#define		CMATRIX_SIGN_REG 0x58

#define BRIGHT_REG			0x55    // Brightness 
#define CONTRAS_REG			0x56    // Contrast control 
#define GFIX_REG			0x69    // Fix gain control 
#define REG76_REG			0x76    // OV's name 
#define		R76_BLKPCOR     0x80    // Black pixel correction enable 
#define		R76_WHTPCOR     0x40    // White pixel correction enable 

#define RGB444_REG			0x8c    // RGB 444 control 
#define		R444_ENABLE     0x02    // Turn on RGB444, overrides 5x5 
#define		R444_RGBX       0x01    // Empty nibble at end 

#define HAECC1_REG			0x9f    // Hist AEC/AGC control 1 
#define HAECC2_REG			0xa0    // Hist AEC/AGC control 2 
#define BD50MAX_REG			0xa5    // 50hz banding step limit 
#define HAECC3_REG			0xa6    // Hist AEC/AGC control 3 
#define HAECC4_REG			0xa7    // Hist AEC/AGC control 4 
#define HAECC5_REG			0xa8    // Hist AEC/AGC control 5 
#define HAECC6_REG			0xa9    // Hist AEC/AGC control 6 
#define HAECC7_REG			0xaa    // Hist AEC/AGC control 7 
#define BD60MAX_REG			0xab    // 60hz banding step limit 
#define SCALING_XSC_REG		0x70	// Has test pattern
#define SCALING_YSC_REG		0x71	// Has test pattern
#define SCALING_DCWCTR_REG	0x72
#define SCALING_PCLK_DIV_REG	0x73
#define SCALING_PCLK_DELAY_REG	0xA2

// Test patterns
#define SHIFT_XSC		0x80	//Bit shift setting register 1 
#define SHIFT_YSC		0x00	//Bit shift setting register 2
#define BAR_XSC			0x00	//Colour bar setting register 1
#define BAR_YSC			0x80	//Colour bar setting register 2
#define FADE_XSC		0x80	//Fade setting register 1
#define FADE_YSC		0x80	//Fade setting register 2

//Buffer PIO Pin definitions
// Buffer pin			SAM4 port/pin	Function			Type		Robot Pin Name
#define RESET_PORT		PIOC			
#define RESET_PIN		PIO_PC15		//Camera reset		Output		RESET
#define PWDN_PORT		PIOC
#define PWDN_PIN		PIO_PC0			//Power Down		Output		PWDN
#define XCLK_PORT		PIOA
#define	XCLK_PIN		PIO_PA0			//Camera Clock		Output		XCLK
#define HREF_PORT		PIOA
#define HREF_PIN		PIO_PA16		//Horiz Reference	Input		HREF
//#define VSYNC_PORT		PIOC
//#define VSYNC_PIN		PIO_PC13		//Vertical Sync		Input		VSYNC (Public, see header)

// Camera Control
#define	resetDisable	(RESET_PORT->PIO_SODR |= RESET_PIN)
#define	resetEnable		(RESET_PORT->PIO_CODR |= RESET_PIN)	//Reset the camera
#define	pwdnDisable		(PWDN_PORT->PIO_CODR |= PWDN_PIN)	//Bring cam out of standby
#define	pwdnEnable		(PWDN_PORT->PIO_SODR |= PWDN_PIN)	//Put cam in standby
#define	HREF			(HREF_PORT->PIO_PDSR & HREF_PIN)	//Indicates when to write to buffer
#define powerUp			{resetDisable; pwdnDisable;}		//Power up the camera
//#define	VSYNC			(REG_PIOC_PDSR & VSYNC_PIN)		//Public (see header file for def)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////

uint16_t hStart, hStop,	vStart,	vStop;		//Image window size (pixels) TODO: Make into a structure
uint16_t winWidth, winHeight, winX, winY;	//Window Size and position (From centre, in pixels)

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t camReadReg(uint8_t regAddress)
*
* Returns the 8 bit value stored in regAddress
*
* Inputs:
* uint8_t regAddress
*	8-bit address of the desired register to read from
*
* Returns:
* Returns the byte stored in the given register
*
* Implementation:
* Simply a wrapper for the TWI read functions that actually do the work.
* Two custom functions were written into the TWI driver to allow reading registers from the camera.
* The first function sets the register to read from, the second reads the data from the last
* register address that was sent to the camera. This has to be done in two steps as the camera is
* unable to handle the repeat start that occurs between writing out the register address and reading
* back the data. This function is private to the camera driver because nothing external should be
* directly accessing camera registers.
*
*/
static inline uint8_t camReadReg(uint8_t regAddress)
{
	twi0SetCamRegister(regAddress);
	return twi0ReadCameraRegister();
}

/*
* Function:
* char camWriteReg(uint8_t regAddress, uint8_t data)
*
* Writes data to the given 8-bit register address
*
* Inputs:
* uint8_t regAddress:
*	The register address to write data to
* uint8_t data:
*	The byte to be written
*
* Returns:
* Returns 0 on success, or non zero otherwise.
*
* Implementation:
* This function is simply a wrapper for the TWI write function.
*
*/
static inline char camWriteReg(uint8_t regAddress, uint8_t data)
{
	return twi0Write(TWI0_CAM_WRITE_ADDR, regAddress, 1, &data);
}

/*
* Function:
* void camRegisterReset(void)
*
* Resets all registers to default values
*
* Inputs:
* None
*
* Returns:
* None
*
* Implementation:
* Sets the register reset bits on the camera
*
*/
static inline void camRegisterReset(void)
{
	camWriteReg(COM7_REG, COM7_RESET);
}

/*
* Function:
* uint8_t camSetup(void)
*
* Performs setup of the camera's registers
*
* Inputs:
* none
*
* Returns:
* Returns 0 on the successful setup of the camera, otherwise returns 0xFF if the camera wasn't
* detected.
*
* Implementation:
* TODO: Implementation of camera setup. This is likely to change a lot of the next short while.
*
* Improvements:
* Many!
*
*/
static uint8_t camSetup(void)
{
	// Camera is not responding or the ID was incorrect
	if (!camValidID()) return 0xFF;	// Stop camera setup

	//camRegisterReset();							//Reset all registers to default.
	//camWriteReg(CLKRC_REG, 0x01);				//No prescaling of the input clock [f/(CLKRC+1)]
	//camWriteReg(TSLB_REG, 0x01);				//Auto adjust output window on resolution change
	////camWriteReg(SCALING_PCLK_DIV_REG, 0xF1);	//MARKED AS DEBUG IN DATASHEET??
	////camWriteReg(SCALING_PCLK_DELAY_REG, 0x02);
	////camWriteReg(COM6_REG, 0xC3);				//Keep HREF at optical black (No diff)
	//camWriteReg(COM12_REG, COM12_HREF);			//Keep HREF on while VSYNCing (prevents loss of
	////pixels on each line)
//
	//////RGB555
	////camWriteReg(RGB444_REG, 0x00);				//Disable RGB444
	////camWriteReg(COM7_REG, COM7_FMT_QVGA|COM7_RGB);	// QVGA and RGB
	////camWriteReg(COM15_REG, 0xF0);					//RGB555 Colour space
//
	////RGB565
	//camWriteReg(RGB444_REG, 0x00);					//Disable RGB444
	//camWriteReg(COM7_REG, COM7_FMT_QVGA|COM7_RGB);	// QVGA and RGB
	//camWriteReg(COM15_REG, 0xD0);					//RGB565 Colour space
	//
	//////RGB444
	////camWriteReg(RGB444_REG, 0x02);				//Enable RGB444
	////camWriteReg(COM7_REG, COM7_FMT_QVGA|COM7_RGB);	//QVGA and RGB
	////camWriteReg(COM15_REG, 0xD0);					//RGB444 Colour space
	//////camWriteReg(COM3_REG, COM3_SWAP);			//Bit swap
	//
	//camWriteReg(CONTRAS_REG, 48);					//Set contrast
	//
	////Set up the Automatic Exposure and Gain Control
	//camWriteReg(COM8_REG, COM8_AEC|COM8_AGC|COM8_AWB); //Also Auto white balance
	//camWriteReg(COM16_REG, COM16_AWBGAIN);
	//
	////Set up the colour matrix:
	//camWriteReg(0x4f, 179);		//1st Coefficient (+)
	//camWriteReg(0x50, 179);		//2nd (-)
	//camWriteReg(0x51, 0);		//3rd (+)
	//camWriteReg(0x52, 61);		//4th (-)
	//camWriteReg(0x53, 176);		//5th (-)
	//camWriteReg(0x54, 228);		//6th (+)
	//camWriteReg(0x58, 0x1A);	//Sign register
	//
	//
	////camSetWindowSize(72, 392, 12, 252);
	
	camSetup2();
	
	camUpdateWindowSize();//Get the window size from the camera

	camTestPattern(CAM_PATTERN_NONE);				//Test pattern can be set here.

	return 0x00;
}

//////////////[Public Functions]////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t camInit(void)
*
* Initialises the uC's hardware for working with the camera, and set's up the camera ready for use.
*
* Inputs:
* none
*
* Returns:
* Returns 0 if initialisation was successful, otherwise returns non zero.
*
* Implementation:
* First the PIO pins are configured from the constants defined above.
* Next, TC0 is setup to produce a 24MHz square wave which the camera will use as a pixel clock from
* pin A0 of the micro.
* Once the microcontroller's hardware is setup, the FIFO buffer and camera itself can be setup.
* The FIFO is set up by calling its initialisation routine from its own driver.
* The camera is hard reset, then is loaded with settings and config data to get it running by
* calling camSetup(). At the moment, if the camSetup() function fails, camInit() will return a non-
* zero deeming the initialisation a failure.
*
*/
uint8_t camInit(void)
{
	////Initialise camera PIO
	//Enable pins
	PWDN_PORT->PIO_PER	|= PWDN_PIN;
	RESET_PORT->PIO_PER	|= RESET_PIN;
	VSYNC_PORT->PIO_PER	|= VSYNC_PIN;
	HREF_PORT->PIO_PER	|= HREF_PIN;
	// Outputs
	PWDN_PORT->PIO_OER	|= PWDN_PIN;
	RESET_PORT->PIO_OER	|= RESET_PIN;
	// Inputs
	VSYNC_PORT->PIO_ODR	|= VSYNC_PIN;
	HREF_PORT->PIO_ODR	|= HREF_PIN;

	pwdnEnable;			//PWDN must be active during startup (See power up sequence in datasheet)

	//Timer Counter 0 Channel 0 Config (Used for the camera clock XCLK_PIN on PA0 (TIOA0))
	//Enable the peripheral clock for TC0
	REG_PMC_PCER0
	|=	(1<<ID_TC0);
	REG_TC0_WPMR
	=	(0x54494D << 8);				//Disable Write Protection
	REG_TC0_CMR0						//TC Channel Mode Register (Pg877)
	|=	TC_CMR_TCCLKS_TIMER_CLOCK1		//Prescaler MCK/2 (100MHz/2 = 50MHz)
	|	TC_CMR_WAVE						//Waveform mode
	|	TC_CMR_WAVSEL_UP_RC				//Clear on RC compare
	|	TC_CMR_ACPA_SET					//Set TIOA0 on RA compare
	|	TC_CMR_ACPC_CLEAR;				//Clear TIOA0 on RC compare
	REG_TC0_RA0							//RA set to 1 counts
	|=	(TC_RA_RA(1));
	REG_TC0_RC0							//RC set to 2 counts (24MHZ). Freqs under 6MHz don't require
	|=	(TC_RC_RC(2));					//the cameras PLL
	REG_TC0_CCR0						//Clock control register
	|=	TC_CCR_CLKEN					//Enable the timer clk.
	|	TC_CCR_SWTRG;					//Start timer register counter
	
	REG_PIOA_ABCDSR1
	|=	(PIO_ABCDSR_P0);				//Set PA0 for peripheral B (TIOA0)
	REG_PIOA_PDR
	|=	XCLK_PIN;						//Allow TC0 to use XCLK_PIN (PA0)

	delay_ms(5);
	pwdnDisable;
	
	camBufferInit();					//Initialise camera RAM buffer
	camHardReset();						//Reset the camera
	return camSetup();					//Load settings into the camera. Returns 0 on successful
	//Setup of camera
}

/*
* Function:
* bool camValidID(void)
*
* Checks camera is responding and has correct ID
*
* Inputs:
* None
*
* Returns:
* 0 If the camera is not responding, or replies with an invalid ID
*
* Implementation:
* Simply reads the ID register 0x0A and checks the value is the same as REG_76
*
* Improvements:
* TODO: Tidy up this function
*
*/
bool camValidID(void)
{
	// PID_REG		0x76	(Default)	// Camera ID MSB
	//uint8_t data = camReadReg(PID_REG);
	uint8_t data = camReadReg(0x0A);
	// if the PID register value is returned and is the camera ID
	return data == REG76_REG;
}

/*
* Function:
* void camHardReset(void)
*
* Performs a hard reset of the camera device.
*
* Inputs:
* None
*
* Returns:
* None
*
* Implementation:
* Clears the reset line to the camera (active low), waits 1ms for it to settle and then brings the
* reset and pwdn lines high to start the camera back up. Again, waits 1ms.
*
*/
void camHardReset(void)
{
	resetEnable;
	delay_ms(1);
	powerUp;
	delay_ms(1);
}

/*
* Function:
* uint8_t camUpdateWindowSize(void)
*
* Returns the dimensions of the capture window within the camera. This is used to format the data
* retrieved from the buffer correctly.
*
* Inputs:
* None (updates global vars within the camera interface module)
*
* Returns:
* 0 on exit
*
* Implementation:
* Retrieves each part of the window dimensions from various registers on the camera and then uses
* bit masking and manipulation to recreate the complete numbers. These are stored in the window size
* globals above.
*
*/
uint8_t camUpdateWindowSize(void)
{
	uint8_t h[4], v[4];
	const uint8_t START_H = 0, START_L = 1, STOP_H = 2, STOP_L = 3;
	
	//uint8_t qvgaMode = ((camReadReg(COM7_REG) & 0x10)>>4);
	
	h[START_H] = camReadReg(HSTART_REG);	//Bits 7:0
	h[START_L] = camReadReg(HREF_REG);		//Bits 2:0
	h[STOP_H] = camReadReg(HSTOP_REG);		//Bits 7:0
	h[STOP_L] = h[START_L];					//Bits 5:3
	v[START_H] = camReadReg(VSTART_REG);	//Bits 7:0
	v[START_L] = camReadReg(VREF_REG);		//Bits 1:0
	v[STOP_H] = camReadReg(VSTOP_REG);		//Bits 7:0
	v[STOP_L] = v[START_L];					//Bits 3:2
	
	hStart	= ((h[START_H]	& 0xFF)<<3) | ((h[START_L]	& 0x07)<<0);
	hStop	= ((h[STOP_H]	& 0xFF)<<3) | ((h[STOP_L]	& 0x34)>>3);
	vStart	= ((v[START_H]	& 0xFF)<<2) | ((v[START_L]	& 0x03)<<0);
	vStop	= ((v[STOP_H]	& 0xFF)<<2) | ((v[STOP_L]	& 0x0C)>>2);
	

	winX = (hStart + hStop)/2;
	winY = (vStart + vStop)/2;
	winWidth = (hStop - hStart);
	winHeight = (vStop - vStart);		

	//If the cam is in QVGA mode, then divide all outputs by 2:
	//if(qvgaMode)
	//{		
		//winX /= 2;
		//winY /= 2;
		//winWidth /= 2;
		//winHeight /=2;
		//hStart /= 2;
		//hStop /= 2;
		//vStart /= 2;
		//vStop /= 2;
	//}	
	
	return 0;
}

/*
* Function:
* uint8_t camSetWindowSize(uint16_t hStart, uint16_t hStop, uint16_t vStart, uint16_t vStop)
*
* Sets the size of the image window.
*
* Inputs:
* uint16_t hStart:
*	The x axis pixel location where the camera should start a line
* uint16_t hStop:
*	The x axis pixel loaction where a line should end
* uint16_t vStart:
*	The vertical line that an image should begin from
* uint16_t the vertical line that an image should end at. 
*
* Returns:
* 0 on exit
*
* Implementation:
* THIS FUNCTION IS STILL EXPERIMENTAL. It seems that the means of setting the window size is not
* entirely obvious. More works is required here. It will most likely be that we will just pull an
* entire frame into the buffer, then use another function to pull a window of the image from there.
* First, this function will check if the camera is in QVGA mode as the output will have to be
* adjusted accordingly. Next, the function retrieves the data from the HREF and VREG registers on
* the camera, as the function will be writing to only a few bits in these registers and the rest of
* the data will need to remain intact.
* 
* If the cam is in QVGA mode, then double the values in the input parameters (The camera seems to
* set the window size based on a full VGA frame, even when in QVGA output mode).
*
* Finally, the approriate bits are cleared in the retrieved HREF and VREF data, bit shifting is
* performed on the input data to get the bits arranged correctly for insertion into the registers.
* Finally, the data is written out to the appropriate registers on the camera.
*
*/
uint8_t camSetWindowSize(uint16_t hStart, uint16_t hStop, uint16_t vStart, uint16_t vStop)
{
	uint8_t h[4], v[4], hRefReg, vRefReg;
	const uint8_t START_H = 0, START_L = 1, STOP_H = 2, STOP_L = 3;
	
	//Check if the camera is in QVGA mode or not.
	//uint8_t qvgaMode = ((camReadReg(COM7_REG) & 0x10)>>4);
	//Get the data from each of the partial registers so we can OR in the new data
	hRefReg = camReadReg(HREF_REG);		//Bits 2:0, 5:3
	vRefReg = camReadReg(VREF_REG);		//Bits 1:0, 3:2
	
	//if(qvgaMode)
	//{
		//hStart *= 2;
		//hStop *= 2;
		//vStart *= 2;
		//vStop *= 2;
	//}
	
	//Clear the appropriate bits in the existing register data
	hRefReg &= ~(0x3F);
	vRefReg &= ~(0x0F);
	
	//Separate out the bitfields ready for each camera register to receive
	h[START_H]	= ((hStart	& 0x07F8)>>3);
	h[START_L]	= ((hStart	& 0x0007)>>0);
	h[STOP_H]	= ((hStop	& 0x07F8)>>3);
	h[STOP_L]	= ((hStop	& 0x0007)>>0);
	v[START_H]	= ((vStart	& 0x03FC)>>2);
	v[START_L]	= ((vStart	& 0x0003)>>0);
	v[STOP_H]	= ((vStop	& 0x03FC)>>2);
	v[STOP_L]	= ((vStop	& 0x0003)>>0);
	
	//Write out the data
	camWriteReg(HSTART_REG, h[START_H]);
	camWriteReg(HSTOP_REG, h[STOP_H]);
	camWriteReg(VSTART_REG, v[START_H]);
	camWriteReg(VSTOP_REG, v[STOP_H]);
	camWriteReg(HREF_REG, h[START_L]|h[STOP_L]|hRefReg);
	camWriteReg(VREF_REG, v[START_L]|v[STOP_L]|vRefReg);	

	return 0;
}

/*
* Function:
* void camRead(void)
*
* Loads a frame from the camera into the FIFO buffer
*
* Inputs:
* None
*
* Returns:
* None
*
* Implementation:
* Waits for Vsync to go high, then resets the write address pointer in the FIFO. When Vsync goes
* low again, the write enable signal is sent to the FIFO. This signal is AND'd in hardware with the
* HREF signal from the camera so that the buffer only writes when there are valid pixels coming
* from the camera.
* Write is enabled until Vsync goes high again, at which point an entire frame should be loaded into
* the FIFO
*
* Improvements:
* Maybe Vsync could be handled by an external interrupt instead of blocking with while loops.
*
*/
void camRead(void)
{
	// Clear read and write buffers
	while (!VSYNC);				//wait for a low vertical sync pulse to reset the pointers in memory
								//buffer, sync pulse goes low
	camBufferWriteReset();		// reset the video buffer memory pointers
	while (VSYNC);
	camBufferWriteStart();
	while (!VSYNC);				// wait for a low vertical sync pulse to reset the pointers in 
								//memory buffer, sync pulse goes low
	camBufferWriteStop();
	camUpdateWindowSize();		//Get the window size from the camera
		
	//read the buffer in parts as we don't have enough memory for an entire frame at once.
	//This shouldn't be here (Should be called from a state in the main function from now on as
	//to not interfere with normal operation of the robot)
	//camBufferReadData(0, 57239, data);			
	
	return;
}

/*
* Function:
* void camChangeFormat(uint8_t type)
*
* Changes the output format of the camera (CURRENTLY NOT USED)
*
* Inputs:
* uint8_t type
*	Just a magic number that specifies the output format
*
* Returns:
* none
*
* Implementation:
* Just sets the appropriate registers. We have found that configuring the camera is actually rather
* complicated, so it's unlikely that we will be changing format from the initial setup.
*
* Improvements:
* TODO: create enum switch
*
*/
void camChangeFormat(uint8_t type)
{
	// COM7_REG			CIF,QVGA,QCIF,RGB
	// IF RGB:
	// COM7_REG			YUV,RGB,Bayer RAW, Processed Bayer RAW
	// IF RGB:
	// RGB444_REG		RGB444, RGB555, RGB565
	// IF RGB555 OR RGB565
	// COM15_REG		RGB555,RGB565

	// COM7_REG			0x04		// Output format RGB
	// RGB444_REG		0x00		// Disable RGB444
	// COM15_REG		0x00		// Enable RGB565

	
	// TEMP: assume RGB565 when RGB
	if (type == COM7_RGB)
	{
		// Enable RGB
		camWriteReg(COM7_REG,COM7_RGB);
		// Disable RGB444
		camWriteReg(RGB444_REG, 0x00);
		// Enable RGB565
		camWriteReg(COM15_REG,COM15_RGB565);
	}
}

/*
* Function:
* void camTestPattern(CameraTestPatterns type)
*
* Will output a test pattern specified by the parameter
*
* Inputs:
* CameraTestPatterns type:
*	An enum that contains the value of different test patterns that can be displayed
*
* Returns:
* none
*
* Implementation:
* The appropriate registers are set for each pattern by way of a switch statement.
*
*/
void camTestPattern(CameraTestPatterns type)
{
	switch (type)
	{
		case CAM_PATTERN_SHIFT:
			camWriteReg(SCALING_XSC_REG, SHIFT_XSC);
			camWriteReg(SCALING_YSC_REG, SHIFT_YSC);
		break;
		case CAM_PATTERN_BAR:
			camWriteReg(SCALING_XSC_REG, BAR_XSC);
			camWriteReg(SCALING_YSC_REG, BAR_YSC);
		break;
		case CAM_PATTERN_FADE:
			camWriteReg(SCALING_XSC_REG, FADE_XSC);
			camWriteReg(SCALING_YSC_REG, FADE_YSC);
		break;
		default:
			camWriteReg(SCALING_XSC_REG, 0x00);
			camWriteReg(SCALING_YSC_REG, 0x00);
		break;
	}
}