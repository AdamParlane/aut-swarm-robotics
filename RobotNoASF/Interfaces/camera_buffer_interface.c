/*
* camera_buffer_interface.c
*
* Author : Brae HW (bhw11@hotmail.co.nz)
* Created: 27/07/2017 1:51:11 PM
* 
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Header file for the camera memory buffer interface.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t camBufferReadByte(void);
*
* void camBufferInit(void);
* void camBufferWriteStop(void);
* void camBufferWriteStart(void);
* void camBufferWriteReset(void);
* void camBufferReadStop(void);
* void camBufferReadStart(void);
* void camBufferReadReset(void);
* uint8_t camBufferWriteFrame(void);
* uint8_t camBufferReadWin(uint32_t left, uint32_t top, uint32_t width, uint32_t height,
*								uint16_t dataBuffer[], uint32_t bufferSize);
* uint8_t camBufferReadWin2(uint32_t hStart, uint32_t hStop, uint32_t vStart, uint32_t vStop,
*								uint16_t dataBuffer[], uint32_t bufferSize)
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "camera_interface.h"
#include "external_interrupt.h"	//External interrupts used to fetch frame from camera
#include "timer_interface.h"	//Provides delay_ms()
#include "camera_buffer_interface.h"

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Buffer PIO Pin definitions
// Buffer pin			SAM4 port/pin	Function			Type		Robot Pin Name
//Commented lines have been moved the header file
//#define WE_PORT		PIOC
//#define WE_PIN		PIO_PC7			//Write Enable		Output		VB_WE
//#define WRST_PORT		PIOA
//#define WRST_PIN		PIO_PA24		//Write Reset		Output		VB_WRST
#define RCK_PORT		PIOA
#define RCK_PIN			PIO_PA15		//Read Clock		Output		VB_RCK
#define OE_PORT			PIOA
#define OE_PIN			PIO_PA11		//Output Enable		Output		VB_OE
#define RRST_PORT		PIOA
#define RRST_PIN		PIO_PA26		//Read Reset		Output		VB_RRST

#define DO0_PORT		PIOA			//Data line 0		Input
#define DO0_PIN			PIO_PA6
#define DO1_PORT		PIOA			//Data line 1		Input
#define DO1_PIN			PIO_PA9
#define DO2_PORT		PIOA			//Data line 2		Input
#define DO2_PIN			PIO_PA10
#define DO3_PORT		PIOC			//Data line 3		Input
#define DO3_PIN			PIO_PC2
#define DO4_PORT		PIOA			//Data line 4		Input
#define DO4_PIN			PIO_PA25
#define DO5_PORT		PIOC			//Data line 5		Input
#define DO5_PIN			PIO_PC4
#define DO6_PORT		PIOC			//Data line 6		Input
#define DO6_PIN			PIO_PC5
#define DO7_PORT		PIOC			//Data line 7		Input
#define DO7_PIN			PIO_PC6

//Data parallel input macros
#define DO0				(DO0_PORT->PIO_PDSR & DO0_PIN)
#define DO1				(DO1_PORT->PIO_PDSR & DO1_PIN)
#define DO2				(DO2_PORT->PIO_PDSR & DO2_PIN)
#define DO3				(DO3_PORT->PIO_PDSR & DO3_PIN)
#define DO4				(DO4_PORT->PIO_PDSR & DO4_PIN)
#define DO5				(DO5_PORT->PIO_PDSR & DO5_PIN)
#define DO6				(DO6_PORT->PIO_PDSR & DO6_PIN)
#define DO7				(DO7_PORT->PIO_PDSR & DO7_PIN)

// Reading from buffer to SAM4
#define readResetOn		RRST_PORT->PIO_CODR	|= RRST_PIN		//Buffer read reset
#define readResetOff	RRST_PORT->PIO_SODR	|= RRST_PIN		//Buffer read on (not in reset)
#define outputEnable	OE_PORT->PIO_CODR	|= OE_PIN		//Buffer output enable
#define outputDisable	OE_PORT->PIO_SODR	|= OE_PIN		//Buffer output disable
#define readClkOn		RCK_PORT->PIO_SODR	|= RCK_PIN;		//Turns the clock pin on manually
#define readClkOff		RCK_PORT->PIO_CODR	|= RCK_PIN;		//Turns the clock pin off manually
#define readClockEnable	REG_TC0_CCR1		|= TC_CCR_CLKEN|TC_CCR_SWTRG//Enable the read clock
#define readClockDisable	REG_TC0_CCR1	= 0				//Disable the read clock
#define readNow			REG_TC0_SR1			& TC_SR_CPCS	//A flag that says whether RC has over-
															//flowed on TC1. (Indicates when to read
															//a byte from the RAM)

//////////////[Private Global Variables]////////////////////////////////////////////////////////////
volatile uint32_t ramAddrPointer = 0;//Indicates the address in buffer that is currently being read
extern RobotGlobalStructure sys;	//Give access to the camReadBuffer flag

//////////////[Private Functions]///////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t camBufferReadByte(void)
*
* Returns a byte from the FIFO
*
* Inputs:
* none
*
* Returns:
* A byte read from the 8-bit PIO port that the FIFO is connected to.
*
* Implementation:
* If each pin is not equal to 0, then the statement returns a 1 in the appropriate bit position.
* All the bits are then OR'd together to create a single byte.
*
*/
static uint8_t camBufferReadByte(void)
{
	return
	(DO0 ? 0x01 : 0x00)
	|(DO1 ? 0x02 : 0x00)
	|(DO2 ? 0x04 : 0x00)
	|(DO3 ? 0x08 : 0x00)
	|(DO4 ? 0x10 : 0x00)
	|(DO5 ? 0x20 : 0x00)
	|(DO6 ? 0x40 : 0x00)
	|(DO7 ? 0x80 : 0x00);
}

//////////////[Public Functions]////////////////////////////////////////////////////////////////////
/*
* Function:
* void camBufferInit()
*
* Initialises the micro hardware required to communicate with the camera buffer
*
* Inputs:
* none
*
* Returns:
* 0 on success, otherwise non zero
*
* Implementation:
* First all necessary PIO pins are configured from the definitions above.
* Timer setup for TC1 is present, but commented out as running the read clock from a timer proved
* ineffective as the interrupt would overrun everything else. The clock is instead bit banged
* when needed.
* Finally a delay is inserted which is required for stabilisation, and the read and write memory
* pointers are reset.
*
*/
uint8_t camBufferInit()
{
	/********Buffer control**************/
	//buffer AL422B can store one complete VGA frame

	// buffer AL422B has a active low WE write enable. a 0 is applied to this pin from NAND gate
	//U7 when HREF (camera) and VB_WE (micro) is high
	// this means the buffer only captures 640 pixels active pixels in one line.

	//the buffer must have read enable (RE - tied to gnd), Output enable (OE) and read reset(RRST)
	//low in order to output stored data
	//Enable PIO control for buffer
	RRST_PORT->PIO_PER	|= RRST_PIN;
	RCK_PORT->PIO_PER	|= RCK_PIN;
	OE_PORT->PIO_PER	|= OE_PIN;
	WRST_PORT->PIO_PER	|= WRST_PIN;
	WE_PORT->PIO_PER	|= WE_PIN;
	DO0_PORT->PIO_PER	|= DO0_PIN;
	DO1_PORT->PIO_PER	|= DO1_PIN;
	DO2_PORT->PIO_PER	|= DO2_PIN;
	DO3_PORT->PIO_PER	|= DO3_PIN;
	DO4_PORT->PIO_PER	|= DO4_PIN;
	DO5_PORT->PIO_PER	|= DO5_PIN;
	DO6_PORT->PIO_PER	|= DO6_PIN;
	DO7_PORT->PIO_PER	|= DO7_PIN;
	
	//Set as an output
	RRST_PORT->PIO_OER	|= RRST_PIN;
	RCK_PORT->PIO_OER	|= RCK_PIN;
	OE_PORT->PIO_OER	|= OE_PIN;
	WRST_PORT->PIO_OER	|= WRST_PIN;
	WE_PORT->PIO_OER	|= WE_PIN;
	DO0_PORT->PIO_OER	|= DO0_PIN;
	DO1_PORT->PIO_OER	|= DO1_PIN;
	DO2_PORT->PIO_OER	|= DO2_PIN;
	DO3_PORT->PIO_OER	|= DO3_PIN;
	DO4_PORT->PIO_OER	|= DO4_PIN;
	DO5_PORT->PIO_OER	|= DO5_PIN;
	DO6_PORT->PIO_OER	|= DO6_PIN;
	DO7_PORT->PIO_OER	|= DO7_PIN;
	
	//Enable internal pullup resistors
	RRST_PORT->PIO_PUER	|= RRST_PIN;
	RCK_PORT->PIO_PUER	|= RCK_PIN;
	OE_PORT->PIO_PUER	|= OE_PIN;
	WRST_PORT->PIO_PUER	|= WRST_PIN;
	WE_PORT->PIO_PUER	|= WE_PIN;
	DO0_PORT->PIO_PUER	|= DO0_PIN;
	DO1_PORT->PIO_PUER	|= DO1_PIN;
	DO2_PORT->PIO_PUER	|= DO2_PIN;
	DO3_PORT->PIO_PUER	|= DO3_PIN;
	DO4_PORT->PIO_PUER	|= DO4_PIN;
	DO5_PORT->PIO_PUER	|= DO5_PIN;
	DO6_PORT->PIO_PUER	|= DO6_PIN;
	DO7_PORT->PIO_PUER	|= DO7_PIN;
	
	////TIMER for READ CLOCK
	////Timer Counter 0 Channel 1 Config (Used for the camera buffer read clock RCK on PA15 (TIOA1)
	////Enable the peripheral clock for TC0
	//REG_PMC_PCER0
	//|=	(1<<ID_TC1);					//Enable peripheral clock for Timer0 Ch1
	//REG_TC0_WPMR
	//=	(0x54494D << 8);				//Disable Write Protection
	//REG_TC0_CMR1						//TC Channel Mode Register (Pg877)
	//|=	TC_CMR_TCCLKS_TIMER_CLOCK1		//Prescaler MCK/2 (100MHz/2 = 50MHz)
	//|	TC_CMR_WAVE						//Waveform mode
	//|	TC_CMR_WAVSEL_UP_RC				//Up mode with auto triggering on RC compare
	//|	TC_CMR_ACPA_CLEAR				//Clear TIOA1 on RA compare
	//|	TC_CMR_ACPC_SET;				//Set TIOA1 on RC compare (Read data on rising edge)
	//REG_TC0_RA1							//RA set to 12 counts
	//|=	(TC_RA_RA(12000));
	//REG_TC0_RC1							//RC set to 25 counts (total (almost) square wave of 500ns
	//|=	(TC_RC_RC(25000));					//period, 2MHZ read clock)
	//REG_TC0_IER1						//TC interrupt enable register
	//|=	TC_IER_CPCS;					//Enable Register C compare interrupt
	//REG_TC0_CCR1						//Clock control register
	//=	0;								//Keep the timer disabled until its needed

	//100ms after power, the buffer chip should be reset (Pg14 of datasheet):
	delay_ms(100);
	camBufferWriteReset();
	camBufferReadReset();
	return 0;
}

/*
* Function:
* void camBufferWriteStop(void)
*
* Disables writing to the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Calls the write disable macro which clears the write enable pin (WE_PIN)
*
*/
void camBufferWriteStop(void)
{
	camBufferWriteDisable;
}

/*
* Function:
* void camBufferWriteStart(void)
*
* Enables writing to the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Calls the write enable macro which sets the write enable pin (WE_PIN)
*
*/
void camBufferWriteStart(void)
{
	// Start write
	camBufferWriteEnable;
}

/*
* Function:
* void camBufferWriteReset(void)
*
* Resets the write address pointer to 0
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Clears the write reset pin (WRST), waits for Vsync (from the camera) to go low, then sets
* the write reset pin again.
*
*/
void camBufferWriteReset(void)
{
	// Reset
	camBufferWriteResetOn;
	while (VSYNC);
	// Clear reset
	camBufferWriteResetOff;
}

/*
* Function:
* void camBufferReadStop(void)
*
* Disables reading from the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Calls the read disable macro which sets the read [output] enable pin (OE_PIN)
*
*/
void camBufferReadStop(void)
{
	outputDisable;
}

/*
* Function:
* void camBufferReadStart(void)
*
* Enables reading from the FIFO
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Calls the read enable macro which clears the read [output] enable pin (OE_PIN)
*
*/
void camBufferReadStart(void)
{
	outputEnable;
}

/*
* Function:
* void camBufferReadReset(void)
*
* Resets the read address pointer to 0
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Clears the read reset pin (RRST), clocks the read clock twice as specified by the datasheet, then 
* sets the read reset pin again. Finally, it also resets the RAM read address pointer.
*
*/
void camBufferReadReset(void)
{
	// Reset
	readResetOn;
	readClkOff;
	readClkOn;
	readClkOff;
	readClkOn;
	readClkOff;
	// Clear reset
	readResetOff;
	ramAddrPointer = 0;
}

/*
* Function:
* uint8_t camBufferReadSequence(uint32_t startAddr, uint32_t endAddr, uint16_t *data)
*
* Allows the caller to read the data between two addresses in the video RAM buffer
*
* Inputs:
* unint32_t startAddr:
*	The address to start reading from the buffer
* uint32_t endAddr:
*	The address to finish reading from the data
* uint16_t *data:
*	Pointer to an array that is large enough to store the retrieved data.
*
* Returns:
* 0 on success
*
* Implementation:
* This function will read data from the FIFO between the given memory addresses. This is achieved
* by counting the memory addresses with ramAddrPointer. First, the function checks if the desired
* start address is less than the RAM address pointer. If it is, then the read pointer on the FIFO
* needs to be reset. If the start address is greater than the RAM address pointer, then we need to
* wait for the read pointer to catch up, by clocking the read clock until it gets there.
*
* Once the RAM address pointer is at the start address, The function begins to load the data from
* the FIFO into an array supplied from the function parameters. When complete, the FIFO read is 
* disabled
*
*/
uint8_t camBufferReadSequence(uint32_t startAddr, uint32_t endAddr, uint16_t *data)
{
	uint8_t msb, lsb;
	
	
	sys.flags.camBufferRead = 0;
	
	//If the ramAddrPointer is greater than the startAddr, then reset the RAM's read pointer
	if(startAddr < ramAddrPointer)
	{
		camBufferReadStop();	//Make sure we aren't already in read mode
		camBufferReadReset();	//Reset the read pointers
	}
	
	//Start the read clock
	camBufferReadStart();
	
	//Wait for the RAM read pointer to reach the start address
	while(ramAddrPointer < startAddr)
	{
		readClkOn;
		readClkOff;
		ramAddrPointer++;
	}
	
	//Now we can begin pulling data from the RAM
	while(ramAddrPointer >= startAddr && ramAddrPointer < endAddr)
	{
		readClkOn;
		//We want to be reading on the rising edge of the read clock
		lsb = camBufferReadByte();
		readClkOff;
		readClkOn;
		//We want to be reading on the rising edge of the read clock
		msb = camBufferReadByte();
		readClkOff;		
		
		data[(ramAddrPointer/CAM_IMAGE_BPP) - startAddr/CAM_IMAGE_BPP] = (msb << 8)|(lsb);
		ramAddrPointer += 2;
	}
	
	//Disable reading from the FIFO
	camBufferReadStop();
	
	return 0;
}

/*
* Function:
* uint8_t camBufferReadWin(uint32_t left, uint32_t top, uint32_t width, uint32_t height,
*								uint16_t dataBuffer[], uint32_t bufferSize)
*
* This function will read a small portion of the image stored in the camera buffer
*
* Inputs:
* uint32_t left:
*	The left (x) coordinate of the image to be retrieved
* uint32_t top:
*	The top (y) coordinate of the image to be retrieved
* uint32_t width:
*	The width (in pixels) of the image to be retrieved
* uint32_t height:
*	The height (in pixels) of the image to be retrieved
* uint16_t dataBuffer[]:
*	Pointer to the array where the image will be stored
* uint32_t bufferSize:
*	Length of the storage array in elements. Used for error checking
*
* Returns:
* 1 if the coordinates and dimensions given are out of range of the image stored in the buffer, or
* if the length of the buffer is not big enough to store the retrieved data.
*
* Implementation:
* First, the function performs checks that the coordinates given are valid, and that the storage 
* array supplied is big enough. Function returns a non zero if there are any problems.
*
* Next, the function calculates the initial memory address in RAM to start reading the image from
* based on the image coordinates and dimensions given. Finally, a for loop iterates line by line,
* retrieving each line of data from the FIFO, and storing it in the correct order in the supplied
* array. The function returns a 0 when complete.
*
*/
uint8_t camBufferReadWin(uint32_t left, uint32_t top, uint32_t width, uint32_t height, 
								uint16_t dataBuffer[], uint32_t bufferSize)
{
	//Make sure that the given dimensions are in range
	if(((left + width) > CAM_IMAGE_WIDTH) || ((top + height) > CAM_IMAGE_HEIGHT))
	{
		return 1;
	}
	
	//Check that the supplied array is big enough for the job at hand
	if(bufferSize < (width*height))
	{
		return 1;
	} else {
		//Work out the initial read address:
		uint32_t initialAddr = top*CAM_IMAGE_WIDTH*CAM_IMAGE_BPP + left*CAM_IMAGE_BPP;
		uint32_t startAddr, endAddr;
		//Read lines:
		for(uint16_t line = 0; line < height; line++)
		{
			//Calculate the start and end addresses for the next read from the buffer.
			startAddr = initialAddr + (line*(CAM_IMAGE_WIDTH)*CAM_IMAGE_BPP);
			endAddr = startAddr + width*CAM_IMAGE_BPP;
			camBufferReadSequence(startAddr, endAddr, dataBuffer + (line*width));
		}
		sys.flags.camBufferRead = 0;
	}
	return 0;
}

/*
* Function:
* uint8_t camBufferReadWin2(uint32_t hStart, uint32_t hStop, uint32_t vStart, uint32_t vStop,
*								uint16_t dataBuffer[], uint32_t bufferSize)
*
* This function will read a small portion of the image stored in the camera buffer
*
* Inputs:
* uint32_t hStart:
*	Horizontal start position
* uint32_t hStop:
*	Horizontal stop position
* uint32_t vStart:
*	Vertical start position
* uint32_t vStop:
*	Vertical stop position
* uint16_t dataBuffer[]:
*	Pointer to the array where the image will be stored
* uint32_t bufferSize:
*	Length of the storage array in elements. Used for error checking
*
* Returns:
* 1 if the coordinates and dimensions given are out of range of the image stored in the buffer, or
* if the length of the buffer is not big enough to store the retrieved data.
*
* Implementation:
* This function is a wrapper function for camBufferReadWin() that allows the coordinates of the
* image to be retrieved to be supplied in a different format to above.
*
*/
uint8_t camBufferReadWin2(uint32_t hStart, uint32_t hStop, uint32_t vStart, uint32_t vStop,
								uint16_t dataBuffer[], uint32_t bufferSize)
{
	return camBufferReadWin(hStart, vStart, hStop - hStart, vStop - vStart, dataBuffer, bufferSize);			
}

/*
* Function:
* uint8_t camBufferWriteFrame(void)
*
* Instructs the camera to write one frame to the FIFO. The process is handled by an external
* interrupt.
*
* Inputs:
* none
*
* Returns:
* Returns 0 when there is a frame ready to be read from the buffer
*
* Implementation:
* This is a wrapper function, and most of the work is performed in the external_interrupt module.
* (See extCamWriteBuffer())
*
*/
uint8_t camBufferWriteFrame(void)
{
	//Wrapper function. Will return 0 when there is data ready to be read from the buffer.
	return extCamWriteToBuffer();
}