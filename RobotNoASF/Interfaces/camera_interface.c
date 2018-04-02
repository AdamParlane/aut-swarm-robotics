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
* void camTestPattern(CameraTestPatterns type);
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "camera_interface.h"
#include "camera_buffer_interface.h"
#include "twimux_interface.h"
#include "timer_interface.h"
#include <stdbool.h>

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define VGA_WIDTH	640
#define VGA_HEIGHT	480
#define QVGA_WIDTH	320
#define QVGA_HEIGHT	240
#define CIF_WIDTH	352
#define CIF_HEIGHT	288
#define QCIF_WIDTH	176
#define	QCIF_HEIGHT	144

/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */
#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */
/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58
#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */
#define REG_GFIX	0x69	/* Fix gain control */
#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */
#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */
#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */
#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */
#define REG_SCALING_XSC		0x70	// Has test pattern
#define REG_SCALING_YSC		0x71	// Has test pattern
#define REG_SCALING_DCWCTR	0x72
#define REG_SCALING_PCLK_DIV	0x73
#define REG_SCALING_PCLK_DELAY	0xA2

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
struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

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

//TODO: Commenting
static uint8_t camSetBits(uint8_t regAddress, uint8_t value, uint8_t bitmask)
{
	//Retrieve the existing data in the register
	uint8_t existing = camReadReg(regAddress);
	
	//Clear the bits specified by the bitmask
	existing &= ~bitmask;
	
	//Write in the value
	existing |= value;
	
	//Write back out to the camera
	camWriteReg(regAddress, existing);
	
	return 0;
	
	//Write out to the camera
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
	camWriteReg(REG_COM7, COM7_RESET);
}

//TODO: Commenting
static uint16_t camLoadSettings(struct regval_list *r)
{
	uint16_t num = 0;
	
	//A 0xFF in both the register and value fields indicates the end of the array
	while(r[num].reg_num != 0xFF && r[num].value != 0xFF)
	{
		camWriteReg(r[num].reg_num, r[num].value);
		num++;
	}
	return num;
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
	//Camera is not responding or the ID was incorrect
	if (!camValidID()) return 0xFF;	// Stop camera setup
	
	//Default register settings. Will be loaded after the camera has been initialised.
	struct regval_list ov7670_default_regs[] = {
		{ REG_COM7, COM7_RESET },
	/*
	 * Clock scale: 3 = 15fps
	 *              2 = 20fps
	 *              1 = 30fps
	 */
		{ REG_TSLB,  0x0D },	/* OV */
		{ REG_COM7, 0 },		/* VGA */
		{ REG_CLKRC, 0x1 },		/* OV: clock scale (30 fps) F_internal = F_input/((B5:0) + 1)*/
		/*
		 * Set the hardware window.  These values from OV don't entirely
		 * make sense - hstop is less than hstart.  But they work...
		 */
		{ REG_HSTART, 0x13 },	{ REG_HSTOP, 0x01 },
		{ REG_HREF, 0xb6 },		{ REG_VSTART, 0x02 },
		{ REG_VSTOP, 0x7a },	{ REG_VREF, 0x0a },
		{ REG_COM3, 0 },		{ REG_COM14, 0 },
		/* Mystery scaling numbers */
		{ 0x70, 0x3a },		{ 0x71, 0x35 },
		{ 0x72, 0x11 },		{ 0x73, 0xf0 },
		{ 0xa2, 0x02 },		{ REG_COM10, 0x0 },
		/* Gamma curve values */
		{ 0x7a, 0x20 },		{ 0x7b, 0x10 },
		{ 0x7c, 0x1e },		{ 0x7d, 0x35 },
		{ 0x7e, 0x5a },		{ 0x7f, 0x69 },
		{ 0x80, 0x76 },		{ 0x81, 0x80 },
		{ 0x82, 0x88 },		{ 0x83, 0x8f },
		{ 0x84, 0x96 },		{ 0x85, 0xa3 },
		{ 0x86, 0xaf },		{ 0x87, 0xc4 },
		{ 0x88, 0xd7 },		{ 0x89, 0xe8 },
		/* AGC and AEC parameters.  Note we start by disabling those features,
		   then turn them only after tweaking the values. */
		{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
		{ REG_GAIN, 0 },	{ REG_AECH, 0 },
		{ REG_COM4, 0x40 }, /* magic reserved bit */
		{ REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
		{ REG_BD50MAX, 0x05 },	{ REG_BD60MAX, 0x07 },
		{ REG_AEW, 0x95 },	{ REG_AEB, 0x33 },
		{ REG_VPT, 0xe3 },	{ REG_HAECC1, 0x78 },
		{ REG_HAECC2, 0x68 },	{ 0xa1, 0x03 }, /* magic */
		{ REG_HAECC3, 0xd8 },	{ REG_HAECC4, 0xd8 },
		{ REG_HAECC5, 0xf0 },	{ REG_HAECC6, 0x90 },
		{ REG_HAECC7, 0x94 },
		{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC },
		/* Almost all of these are magic "reserved" values.  */
		{ REG_COM5, 0x61 },	{ REG_COM6, 0x4b },
		{ 0x16, 0x02 },		{ REG_MVFP, 0x07 },
		{ 0x21, 0x02 },		{ 0x22, 0x91 },
		{ 0x29, 0x07 },		{ 0x33, 0x0b },
		{ 0x35, 0x0b },		{ 0x37, 0x1d },
		{ 0x38, 0x71 },		{ 0x39, 0x2a },
		{ REG_COM12, 0x78 },	{ 0x4d, 0x40 },
		{ 0x4e, 0x20 },		{ REG_GFIX, 0 },
		{ 0x6b, 0x4a },		{ 0x74, 0x10 },
		{ 0x8d, 0x4f },		{ 0x8e, 0 },
		{ 0x8f, 0 },		{ 0x90, 0 },
		{ 0x91, 0 },		{ 0x96, 0 },
		{ 0x9a, 0 },		{ 0xb0, 0x84 },
		{ 0xb1, 0x0c },		{ 0xb2, 0x0e },
		{ 0xb3, 0x82 },		{ 0xb8, 0x0a },
		/* More reserved magic, some of which tweaks white balance */
		{ 0x43, 0x0a },		{ 0x44, 0xf0 },
		{ 0x45, 0x34 },		{ 0x46, 0x58 },
		{ 0x47, 0x28 },		{ 0x48, 0x3a },
		{ 0x59, 0x88 },		{ 0x5a, 0x88 },
		{ 0x5b, 0x44 },		{ 0x5c, 0x67 },
		{ 0x5d, 0x49 },		{ 0x5e, 0x0e },
		{ 0x6c, 0x0a },		{ 0x6d, 0x55 },
		{ 0x6e, 0x11 },		{ 0x6f, 0x9f }, /* "9e for advance AWB" */
		{ 0x6a, 0x40 },		{ REG_BLUE, 0x40 },
		{ REG_RED, 0x60 },
		{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB },
		/* Matrix coefficients */
		{ 0x4f, 0x80 },		{ 0x50, 0x80 },
		{ 0x51, 0 },		{ 0x52, 0x22 },
		{ 0x53, 0x5e },		{ 0x54, 0x80 },
		{ 0x58, 0x9e },
		{ REG_COM16, COM16_AWBGAIN },	{ REG_EDGE, 0 },
		{ 0x75, 0x05 },		{ 0x76, 0xe1 },
		{ 0x4c, 0 },		{ 0x77, 0x01 },
		{ REG_COM13, 0xc3 },	{ 0x4b, 0x09 },
		{ 0xc9, 0x60 },		{ REG_COM16, 0x38 },
		{ 0x56, 0x40 },
		{ 0x34, 0x11 },		{ REG_COM11, COM11_EXP|COM11_HZAUTO },
		{ 0xa4, 0x88 },		{ 0x96, 0 },
		{ 0x97, 0x30 },		{ 0x98, 0x20 },
		{ 0x99, 0x30 },		{ 0x9a, 0x84 },
		{ 0x9b, 0x29 },		{ 0x9c, 0x03 },
		{ 0x9d, 0x4c },		{ 0x9e, 0x3f },
		{ 0x78, 0x04 },
		/* Extra-weird stuff.  Some sort of multiplexor register */
		{ 0x79, 0x01 },		{ 0xc8, 0xf0 },
		{ 0x79, 0x0f },		{ 0xc8, 0x00 },
		{ 0x79, 0x10 },		{ 0xc8, 0x7e },
		{ 0x79, 0x0a },		{ 0xc8, 0x80 },
		{ 0x79, 0x0b },		{ 0xc8, 0x01 },
		{ 0x79, 0x0c },		{ 0xc8, 0x0f },
		{ 0x79, 0x0d },		{ 0xc8, 0x20 },
		{ 0x79, 0x09 },		{ 0xc8, 0x80 },
		{ 0x79, 0x02 },		{ 0xc8, 0xc0 },
		{ 0x79, 0x03 },		{ 0xc8, 0x40 },
		{ 0x79, 0x05 },		{ 0xc8, 0x30 },
		{ 0x79, 0x26 },
		{ 0xff, 0xff },	/* END MARKER */
	};

	//Register settings for RGB565 mode.
	struct regval_list ov7670_fmt_rgb565[] = {
		{ REG_COM7, COM7_RGB },	/* Selects RGB mode */
		{ REG_RGB444, 0 },	/* No RGB444 please */
		{ REG_COM1, 0x0 },	/* CCIR601 */
		{ REG_COM15, COM15_RGB565 },
		{ REG_COM9, 0x38 }, 	/* 16x gain ceiling; 0x8 is reserved bit */
		{ 0x4f, 0xb3 }, 	/* "matrix coefficient 1" */
		{ 0x50, 0xb3 }, 	/* "matrix coefficient 2" */
		{ 0x51, 0    },		/* vb */
		{ 0x52, 0x3d }, 	/* "matrix coefficient 4" */
		{ 0x53, 0xa7 }, 	/* "matrix coefficient 5" */
		{ 0x54, 0xe4 }, 	/* "matrix coefficient 6" */
		{ REG_COM13, COM13_GAMMA|COM13_UVSAT },
		{ 0xff, 0xff },
	};

	//Load register defaults:
	camLoadSettings(ov7670_default_regs);
		
	//Setup RGB565
	camLoadSettings(ov7670_fmt_rgb565);
		
	//Enable scaling and downscaling
	camSetBits(REG_COM3, COM3_SCALEEN|COM3_DCWEN, COM3_SCALEEN|COM3_DCWEN);

	//QVGA Resolution (320x240)
	camSetBits(REG_COM7, COM7_FMT_QVGA, COM7_FMT_QVGA);
	
	//Keep HREF on while VSYNCing (prevents loss of pixels on each line)
	camSetBits(REG_COM12, COM12_HREF, COM12_HREF);
	
	//Experimental
	//camWriteReg(CONTRAS_REG, 0x40);
	//camWriteReg(BRIGHT_REG, 0x00);
	
	//Get the window size from the camera
	camUpdateWindowSize();

	//Test pattern can be set here.
	camTestPattern(CAM_PATTERN_NONE);

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
	
	camHardReset();						//Reset the camera
	if(!camSetup())						//Load settings into the camera. Returns 0 on success.
	{
		camBufferInit();				//Initialise camera RAM buffer
		return 0;
	} else {
		return 1;
	}
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
	return data == REG_REG76;
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
	
	h[START_H] = camReadReg(REG_HSTART);	//Bits 7:0
	h[START_L] = camReadReg(REG_HREF);		//Bits 2:0
	h[STOP_H] = camReadReg(REG_HSTOP);		//Bits 7:0
	h[STOP_L] = h[START_L];					//Bits 5:3
	v[START_H] = camReadReg(REG_VSTART);	//Bits 7:0
	v[START_L] = camReadReg(REG_VREF);		//Bits 1:0
	v[STOP_H] = camReadReg(REG_VSTOP);		//Bits 7:0
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
	hRefReg = camReadReg(REG_HREF);		//Bits 2:0, 5:3
	vRefReg = camReadReg(REG_VREF);		//Bits 1:0, 3:2
	
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
	camWriteReg(REG_HSTART, h[START_H]);
	camWriteReg(REG_HSTOP, h[STOP_H]);
	camWriteReg(REG_VSTART, v[START_H]);
	camWriteReg(REG_VSTOP, v[STOP_H]);
	camWriteReg(REG_HREF, h[START_L]|h[STOP_L]|hRefReg);
	camWriteReg(REG_VREF, v[START_L]|v[STOP_L]|vRefReg);	

	return 0;
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
			camWriteReg(REG_SCALING_XSC, SHIFT_XSC);
			camWriteReg(REG_SCALING_YSC, SHIFT_YSC);
		break;
		case CAM_PATTERN_BAR:
			camWriteReg(REG_SCALING_XSC, BAR_XSC);
			camWriteReg(REG_SCALING_YSC, BAR_YSC);
		break;
		case CAM_PATTERN_FADE:
			camWriteReg(REG_SCALING_XSC, FADE_XSC);
			camWriteReg(REG_SCALING_YSC, FADE_YSC);
		break;
		default:
			camWriteReg(REG_SCALING_XSC, 0x00);
			camWriteReg(REG_SCALING_YSC, 0x00);
		break;
	}
}