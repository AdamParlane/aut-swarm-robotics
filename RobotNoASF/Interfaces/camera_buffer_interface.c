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
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "camera_interface.h"
#include "camera_buffer_interface.h"
#include "timer_interface.h"	//Provides delay_ms()

//////////////[Private Defines]/////////////////////////////////////////////////////////////////////
//Buffer PIO Pin definitions
// Buffer pin			SAM4 port/pin	Function			Type		Robot Pin Name
#define WE_PORT			PIOC
#define WE_PIN			PIO_PC7			//Write Enable		Output		VB_WE
#define WRST_PORT		PIOA
#define WRST_PIN		PIO_PA24		//Write Reset		Output		VB_WRST
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

// Writing to buffer from camera
#define writeDisable	WE_PORT->PIO_CODR	|= WE_PIN		//Buffer write disable (Active high via
															//NAND gate)
#define writeEnable		WE_PORT->PIO_SODR	|= WE_PIN		//Buffer write enable
#define writeResetOn	WRST_PORT->PIO_CODR	|= WRST_PIN		//Buffer write reset
#define writeResetOff	WRST_PORT->PIO_SODR |= WRST_PIN		//Buffer write on (not in reset)

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
//TODO: Make sure that this variable is reset to 0 everytime read reset is asserted to stay synced
//with the buffer.

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
	writeDisable;
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
	writeEnable;
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
	writeResetOn;
	while (VSYNC);
	// Clear reset
	writeResetOff;
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
* uint8_t camBufferReadData(uint32_t startAddr, uint32_t endAddr, uint16_t *data)
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
uint8_t camBufferReadData(uint32_t startAddr, uint32_t endAddr, uint8_t *data)
{
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
		//delay_ms(1);
		readClkOff;
		//delay_ms(1);
		ramAddrPointer++;
	}
	
	//Now we can begin pulling data from the RAM
	while(ramAddrPointer >= startAddr && ramAddrPointer <= endAddr)
	{
		readClkOn;
		//We want to be reading on the rising edge of the read clock
		data[ramAddrPointer - startAddr] = camBufferReadByte();
		ramAddrPointer++;
		readClkOff;
	}
	
	//Disable reading from the FIFO
	camBufferReadStop();
	
	return 0;
}