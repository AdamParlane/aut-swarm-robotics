/*
 * timer0.c
 *
 * Created: 6/08/2017 1:23:13 PM
 *  Author: adams
 */ 

#include "timer0.h"


/*
* Function:
* void timer0Init(void)
*
* initialise timer0. will be moved to its own module soon
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* TODO:[explain key steps of function] timer0 init
* [use heavy detail for anything complicated]
*
* Improvements:
* Move to its own module.
*
*/
void timer0Init(void)
{
	////TIMER0////
	//Timer0 is used for delay_ms and get_ms functions required by the imu driver
	REG_PMC_PCER0
	|=	(1<<ID_TC0);						//Enable TC clock (ID_TC0 is the peripheral identifier
	//for timer counter 0)
	REG_PIOA_PDR |= (1<<0);					// enable TCO control of XCLK for Camera (PA0)

	NVIC_EnableIRQ(ID_TC0);					//Enable interrupt vector for TIMER0
	REG_TC0_CMR0							//TC Channel Mode Register (Pg877)
	|=	TC_CMR_TCCLKS_TIMER_CLOCK1			//Prescaler MCK/2 (100MHz/2 = 50MHz)
	|	TC_CMR_WAVE							//Waveform mode
	|	TC_CMR_WAVSEL_UP_RC					//Clear on RC compare
	|	TC_CMR_ACPA_SET						// Set pulse on RA compare
	|	TC_CMR_ACPC_CLEAR;					// Clear pulse on RC compare
	REG_TC0_IER0							//TC interrupt enable register
	|=	TC_IER_CPBS;						//Enable Register B compare interrupt
	REG_TC0_RB0								//Set Register B (the timer counter value at which the
	//interrupt will be triggered)
	=	50;									//Trigger once every us (100MHz/2/1M)
	REG_TC0_RA0 |= (TC_RA_RA(2));			// RA set to 2 counts
	REG_TC0_RC0 |= (TC_RC_RC(4));			// RC set to 4 counts (total square wave of 80ns period, 12.5MHZ)
	REG_TC0_CCR0							//Clock control register
	|=	TC_CCR_CLKEN						//Enable the timer clk.
	|	TC_CCR_SWTRG;						//Start timer register counter
}


/*
* Function: int get_ms(uint32_t *timestamp)
*
* Required by the IMU drivers (hence naming convention). Outputs the system uptime generated from
* Timer0.
*
* Inputs:
* address of an integer where the timestamp will be stored
*
* Returns:
* function will return 1 if invalid pointer is passed, otherwise a 0 on success
*
* Implementation:
* Retrieves the value stored in systemTimestamp (stores the number of millisecs that have elapsed
* since power on) and drops it at the address given by *timestamp. if *timestamp is an invalid
* address then returns a 1.
*
*/
int get_ms(uint32_t *timestamp)
{
	if(!timestamp)
	return 1;
	*timestamp = systemTimestamp;
	return 0;
}


/*
* Function: int delay_ms(uint32_t period_ms)
*
* Required by the IMU drivers (hence naming convention). Halts execution for desired number of
* milliseconds.
*
* Inputs:
* period_ms is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
* Implementation:
* Stores systemTimestamp at the start of the function, then waits until systemTimestamp has
* increased by the amount given in period_ms before continuing.
*
*/
int delay_ms(uint32_t period_ms)
{
	uint32_t startTime = systemTimestamp;
	while(systemTimestamp < (startTime + period_ms));
	return 0;
}

/*
* Function: void TC0_Handler()
*
* Interrupt handler for Timer0. Is used to help implement get_ms() and delay_ms() functions
* required by the IMU driver. Is also used to trigger reading the IMU's FIFO buffer (until
* hardware interrupts are implemented). The only interrupt on Timer0 is on Register C compare,
* which will trigger an interrupt once every millisecond
*
* Inputs:
* none
*
* Returns:
* Increments systemTimestamp once every millisecond.
*
* Implementation:
* If the RC compare flag is set then it increments the systemTimestamp, and also checks if 5ms
* has elapsed. If so, will set a flag to read from the IMU's FIFO buffer (unimplemented)
*
*/
void TC0_Handler()
{
	//The interrupt handler for timer counter 0
	//Triggers every 1ms
	if(REG_TC0_SR0 & TC_SR_CPBS)									//If RB compare flag
	{
		usTimeStamp++;
		if(usTimeStamp % 100)
		{
			systemTimestamp++;
			streamDelayCounter++;
			if(streamDelayCounter == 100) //used for streaming data
			{
				streamDelayCounter = 0;
				streamIntervalFlag = 1;
			}
			//V1 robot doesn't have the IMU's interrupt pin tied in to the uC, so the FIFO will have to be
			//polled. V2 does utilize an external interrupt, so this code is not necessary.
			#if defined ROBOT_TARGET_V1
			//Read IMUs FIFO every 5ms on the V1 platform
			if(systemTimestamp >= (imuFifoNextReadTime + 5))
			{
				imuFifoNextReadTime = systemTimestamp;
			}
			#endif		
		}
	}
}