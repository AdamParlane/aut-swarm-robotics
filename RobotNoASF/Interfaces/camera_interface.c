/*
* camera_interface.c
*
* Author : Brae HW (bhw11@hotmail.co.nz)
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
* void camInit(void);
* uint8_t camSetup(void);
* void camWriteInstruction(uint8_t regAddress, uint8_t data);
* uint8_t camReadInstruction(uint8_t data);
* void camHardReset(void);
* void camRegisterReset(void);
* bool camValidID(void)
* void camChangeFormat(uint8_t type);
* void camTestPattern(CameraTestPatterns type);
* void camRead(void);
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "camera_interface.h"
#include "camera_buffer_interface.h"
#include "twimux_interface.h"
#include "timer_interface.h"
#include <stdbool.h>

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//********* Camera Register Set *********/
#define GAIN_REG        0x00    // Gain lower 8 bits (rest in vref)
#define BLUE_REG        0x01    // blue gain 
#define RED_REG         0x02    // red gain 
#define VREF_REG        0x03    // Pieces of GAIN, VSTART, VSTOP 
#define COM1_REG        0x04    // Control 1 
#define COM1_CCIR656    0x40    // CCIR656 enable 
#define BAVE_REG        0x05    // U/B Average level 
#define GbAVE_REG       0x06    // Y/Gb Average level 
#define AECHH_REG       0x07    // AEC MS 5 bits 
#define RAVE_REG        0x08    // V/R Average level 
#define COM2_REG        0x09    // Control 2 
#define COM2_SSLEEP     0x10    // Soft sleep mode 
#define PID_REG         0x0a    // Product ID MSB 
#define VER_REG         0x0b    // Product ID LSB 
#define COM3_REG        0x0c    // Control 3 
#define COM3_SWAP       0x40    // Byte swap 
#define COM3_SCALEEN    0x08    // Enable scaling 
#define COM3_DCWEN      0x04    // Enable down-sample/crop/window 
#define COM4_REG        0x0d    // Control 4 
#define COM5_REG        0x0e    // All "reserved" 
#define COM6_REG        0x0f    // Control 6 
#define AECH_REG        0x10    // More bits of AEC value 

#define CLKRC_REG       0x11    // Clock control 
#define		CLK_EXT         0x40    // Use external clock directly 
#define		CLK_SCALE       0x3f    // Mask for internal clock scale 

#define COM7_REG        0x12    // Control 7 
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

#define COM8_REG        0x13    // Control 8 
#define COM8_FASTAEC    0x80    // Enable fast AGC/AEC 
#define COM8_AECSTEP    0x40    // Unlimited AEC step size 
#define COM8_BFILT      0x20    // Band filter enable 
#define COM8_AGC        0x04    // Auto gain enable 
#define COM8_AWB        0x02    // White balance enable 
#define COM8_AEC        0x01    // Auto exposure enable 
#define COM9_REG        0x14    // Control 9  - gain ceiling 
#define COM10_REG       0x15    // Control 10 
#define COM10_HSYNC     0x40    // HSYNC instead of HREF 
#define COM10_PCLK_HB   0x20    // Suppress PCLK on horiz blank 
#define COM10_HREF_REV  0x08    // Reverse HREF 
#define COM10_VS_LEAD   0x04    // VSYNC on clock leading edge 
#define COM10_VS_NEG    0x02    // VSYNC negative 
#define COM10_HS_NEG    0x01    // HSYNC negative 
#define HSTART_REG      0x17    // Horiz start high bits 
#define HSTOP_REG       0x18    // Horiz stop high bits 
#define VSTART_REG      0x19    // Vert start high bits 
#define VSTOP_REG       0x1a    // Vert stop high bits 
#define PSHFT_REG       0x1b    // Pixel delay after HREF 
#define MIDH_REG        0x1c    // Manuf. ID high 
#define MIDL_REG        0x1d    // Manuf. ID low 
#define MVFP_REG        0x1e    // Mirror / vflip 
#define MVFP_MIRROR     0x20    // Mirror image 
#define MVFP_FLIP       0x10    // Vertical flip 
#define AEW_REG         0x24    // AGC upper limit 
#define AEB_REG         0x25    // AGC lower limit 
#define VPT_REG         0x26    // AGC/AEC fast mode op region 
#define HSYST_REG       0x30    // HSYNC rising edge delay 
#define HSYEN_REG       0x31    // HSYNC falling edge delay 
#define HREF_REG        0x32    // HREF pieces 
#define TSLB_REG        0x3a    // lots of stuff 
#define TSLB_YLAST      0x04    // UYVY or VYUY - see com13 
#define COM11_REG       0x3b    // Control 11 
#define COM11_NIGHT     0x80    // NIght mode enable 
#define COM11_NMFR      0x60    // Two bit NM frame rate 
#define COM11_HZAUTO    0x10    // Auto detect 50/60 Hz 
#define COM11_50HZ      0x08    // Manual 50Hz select 
#define COM11_EXP       0x02
#define COM12_REG       0x3c    // Control 12 
#define COM12_HREF      0x80    // HREF always 
#define COM13_REG       0x3d    // Control 13 
#define COM13_GAMMA     0x80    // Gamma enable 
#define COM13_UVSAT     0x40    // UV saturation auto adjustment 
#define COM13_UVSWAP    0x01    // V before U - w/TSLB 
#define COM14_REG       0x3e    // Control 14 
#define COM14_DCWEN     0x10    // DCW/PCLK-scale enable 
#define EDGE_REG        0x3f    // Edge enhancement factor 
#define COM15_REG       0x40    // Control 15 
#define COM15_R10F0     0x00    // Data range 10 to F0 
#define COM15_R01FE     0x80    //            01 to FE 
#define COM15_R00FF     0xc0    //            00 to FF 
#define COM15_RGB565    0x10    // RGB565 output 
#define COM15_RGB555    0x30    // RGB555 output 
#define COM16_REG       0x41    // Control 16 
#define COM16_AWBGAIN   0x08    // AWB gain enable 
#define COM17_REG       0x42    // Control 17 
#define COM17_AECWIN    0xc0    // AEC window - must match COM4 
#define COM17_CBAR      0x08    // DSP Color bar 
#define CMATRIX_BASE_REG 0x4f
#define CMATRIX_LEN 6
#define CMATRIX_SIGN_REG 0x58
#define BRIGHT_REG      0x55    // Brightness 
#define CONTRAS_REG     0x56    // Contrast control 
#define GFIX_REG        0x69    // Fix gain control 
#define REG76_REG       0x76    // OV's name 
#define R76_BLKPCOR     0x80    // Black pixel correction enable 
#define R76_WHTPCOR     0x40    // White pixel correction enable 
#define RGB444_REG      0x8c    // RGB 444 control 
#define R444_ENABLE     0x02    // Turn on RGB444, overrides 5x5 
#define R444_RGBX       0x01    // Empty nibble at end 
#define HAECC1_REG      0x9f    // Hist AEC/AGC control 1 
#define HAECC2_REG      0xa0    // Hist AEC/AGC control 2 
#define BD50MAX_REG     0xa5    // 50hz banding step limit 
#define HAECC3_REG      0xa6    // Hist AEC/AGC control 3 
#define HAECC4_REG      0xa7    // Hist AEC/AGC control 4 
#define HAECC5_REG      0xa8    // Hist AEC/AGC control 5 
#define HAECC6_REG      0xa9    // Hist AEC/AGC control 6 
#define HAECC7_REG      0xaa    // Hist AEC/AGC control 7 
#define BD60MAX_REG     0xab    // 60hz banding step limit 
#define SCALING_XSC_REG	0x70	// Has test pattern
#define SCALING_YSC_REG 0x71	// Has test pattern
#define SCALING_DCWCTR_REG		0x72
#define SCALING_PCLK_DIV_REG	0x73
#define SCALING_PCLK_DELAY_REG	0xA2

// Test patterns
#define SHIFT_XSC		0x80	// 
#define SHIFT_YSC		0x00	//
#define BAR_XSC			0x00	//
#define BAR_YSC			0x80	// 
#define FADE_XSC		0x80	//
#define FADE_YSC		0x80	//

/********** Camera Pin Connections **********/
// Camera Timing
#define RESET_PIN			PIO_PC15
#define PWDN_PIN			PIO_PC0
#define	XCLK_PIN			PIO_PA0
#define VSYNC_PIN			PIO_PC13
#define HREF_PIN			PIO_PA16
// Camera Control
#define	resetDisable	(REG_PIOC_SODR |= PIO_PC15)
#define	resetEnable		(REG_PIOC_CODR |= PIO_PC15)	// Reset the camera
#define	pwdnDisable		(REG_PIOC_CODR |= PWDN_PIN)	// Bring cam out of standby
#define	pwdnEnable		(REG_PIOC_SODR |= PWDN_PIN)	// Put cam in standby
#define powerUp			{resetDisable; pwdnDisable;}// Power up the camera
#define	VSYNC			(REG_PIOC_PDSR & VSYNC_PIN)

//#define HEIGHT				480			// Vertical Pixel Count
//#define WIDTH					640			// Horizontal Pixel Count
//#define BYTE_PER_PIXEL		2			// Pixel data size
//#define V_FRONT				11			// Vertical front porch
//#define V_BACK				31			// Vertical back porch
//#define H_FRONT				16			// Horizontal front porch
//#define H_BACK				48			// Horizontal back porch

//#define cameraByteCount		614400		//640*480*2
#define cameraByteCount			153600		//320*240*2
#define bufferSize				393216		//
//#define bufferOffset			bufferSize - cameraByteCount	// 239616
#define bufferOffset			0	// 239616

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
volatile bool flagLineReady;
uint8_t data[38400];		// 2*320*60 (2*w*h) 2 bytes per pixel
volatile int current;
volatile int bufferCount;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
void camInit(void)
{
	// Enable pins
	REG_PIOC_PER
	|=	RESET_PIN		// Enable PIO control of Camera RESET_PIN
	|	PWDN_PIN		// Set PWDN_PIN
	|	VSYNC_PIN;		// Enable PIO control of Camera VSYNC
	REG_PIOA_PER
	|=	HREF_PIN;		// Enable PIO control of HREF(PA16)
	// Outputs
	REG_PIOC_OER
	|=	PWDN_PIN		// Set PWDN_PIN as an output
	|	RESET_PIN;		// Set RESET_PIN as an output
	// Inputs
	REG_PIOC_ODR
	|=	VSYNC_PIN;		// Set VSYNC as an input
	REG_PIOA_ODR
	|=	HREF_PIN;		// Set HREF as an input

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
	REG_TC0_RA0							//RA set to 5 counts
	|=	(TC_RA_RA(2));
	REG_TC0_RC0							//RC set to 10 counts (5MHZ). Freqs under 6MHz don't require
	|=	(TC_RC_RC(4));					//the cameras PLL
	REG_TC0_CCR0						//Clock control register
	|=	TC_CCR_CLKEN					//Enable the timer clk.
	|	TC_CCR_SWTRG;					//Start timer register counter
	
	REG_PIOA_ABCDSR1
	|=	(PIO_ABCDSR_P0);				//Set PA0 for peripheral B (TIOA0)
	REG_PIOA_PDR
	|=	XCLK_PIN;						//Allow TC0 to use XCLK_PIN (PA0)


	delay_ms(5);
	pwdnDisable;
	
	camHardReset();						//Reset the camera
	camSetup();							//Load settings into the camera
}

uint8_t camSetup(void)
{
	// Hard reset
	//camReset();
	// Camera is not responding or the ID was incorrect
	if (!camValidID()) return 0xFF;	// Stop camera setup

	camRegisterReset();

	camWriteInstruction(CLKRC_REG, 0x01);
	camWriteInstruction(COM7_REG, COM7_FMT_QVGA | COM7_RGB);		// QVGA and RGB
	camWriteInstruction(RGB444_REG, 0x00);							//Disable RGB444
	camWriteInstruction(COM15_REG, COM15_RGB555);					//RGB555 Colour space
	camWriteInstruction(COM3_REG, 0x80);
	camWriteInstruction(COM14_REG, 0x80);
	//camWriteInstruction(SCALING_XSC_REG, 0x3A);			//No mention of these in the datasheet?
	//camWriteInstruction(SCALING_YSC_REG, 0x35);
	camWriteInstruction(SCALING_DCWCTR_REG, 0x11);
	camWriteInstruction(SCALING_PCLK_DIV_REG, 0xF1);
	camWriteInstruction(SCALING_PCLK_DELAY_REG, 0x02);

	
	//camWriteInstruction(HSTART_REG, 0x12);
	//camWriteInstruction(HSTOP_REG, 0x61);
	//camWriteInstruction(VSTART_REG, 0x03);
	//camWriteInstruction(VSTOP_REG, 0x7B);
	//camWriteInstruction(PSHFT_REG, 0x00);


	// PCLK does not toggle during horizontal blank
	camWriteInstruction(COM10_REG, COM10_PCLK_HB);

	// Set image format to RGB565
	//camChangeFormat(COM7_RGB);

	// Test bar image
	//camWriteInstruction(COM17_REG, COM17_CBAR);
	camTestPattern(CAM_PATTERN_NONE);

	return 0x00;
}

// Write to the 8 bit regAdress
void camWriteInstruction(uint8_t regAddress, uint8_t data)
{
	twi0Write(TWI0_CAM_WRITE_ADDR, regAddress, 1, &data);
}

// Returns the 8 bit value stored in regAddress
uint8_t camReadInstruction(uint8_t regAddress)
{
	twi0SetCamRegister(regAddress);
	uint8_t data = twi0ReadCameraRegister();
	return data;
}

// Hard reset of the camera device
void camHardReset(void)
{
	resetEnable;
	delay_ms(1);
	powerUp;
	delay_ms(1);
}

// Resets all registers to default values
void camRegisterReset(void)
{
	camWriteInstruction(COM7_REG, COM7_RESET);
}

// Checks camera is responding and has correct ID
bool camValidID(void)
{
	// PID_REG		0x76	(Default)	// Camera ID MSB

	//uint8_t data = camReadInstruction(PID_REG);
	uint8_t data = camReadInstruction(0x0A);
	// if the PID register value is returned and is the camera ID
	return data == REG76_REG;
}

/********** Camera Data Reading **********/
void camRead(void)
{
	// Clear read and write buffers
	while (VSYNC); // wait for a low vertical sync pulse to reset the pointers in memory buffer, sync pulse goes low
	camBufferWriteReset(); // reset the video buffer memory pointers
	camBufferReadReset();
	camBufferWriteStart();
	
	while (!VSYNC); // wait for sync pulse to go high
	while (VSYNC); // wait for a vertical sync pulse, sync pulse goes low, thus a frame of data has been stored
	
	//camBufferWriteReset(); // reset the video buffer memory pointers
	//camBufferReadReset();
	//camBufferWriteStart();
	//
	//while (!VSYNC); // wait for sync pulse to go high
	//while (VSYNC); // wait for a vertical sync pulse, sync pulse goes low, thus a frame of data has been stored
	
	camBufferWriteStop();

	camBufferReadData(0, 38399, data);

	//uint8_t buffer;
	//uint8_t r = 0;
	//uint8_t g = 0;
	//uint8_t b = 0;


				//r = ((data[j][i/2]&0x7C00)>>10);
				//g = ((data[j][i/2]&0x03E0)>>5);
				//b = ((data[j][i/2]&0x001F)>>0);

	return;
}

/********** Camera Settings **********/
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

	// TODO: create enum switch
	// TEMP: assume RGB565 when RGB
	if (type == COM7_RGB)
	{
		// Enable RGB
		camWriteInstruction(COM7_REG,COM7_RGB);
		// Disable RGB444
		camWriteInstruction(RGB444_REG, 0x00);
		// Enable RGB565
		camWriteInstruction(COM15_REG,COM15_RGB565);
	}
}

void camTestPattern(CameraTestPatterns type)
{
	switch (type)
	{
		case CAM_PATTERN_SHIFT:
			camWriteInstruction(SCALING_XSC_REG, SHIFT_XSC);
			camWriteInstruction(SCALING_YSC_REG, SHIFT_YSC);
		break;
		case CAM_PATTERN_BAR:
			camWriteInstruction(SCALING_XSC_REG, BAR_XSC);
			camWriteInstruction(SCALING_YSC_REG, BAR_YSC);
		break;
		case CAM_PATTERN_FADE:
			camWriteInstruction(SCALING_XSC_REG, FADE_XSC);
			camWriteInstruction(SCALING_YSC_REG, FADE_YSC);
		break;
		default:
			camWriteInstruction(SCALING_XSC_REG, 0x00);
			camWriteInstruction(SCALING_YSC_REG, 0x00);
		break;
	}
}