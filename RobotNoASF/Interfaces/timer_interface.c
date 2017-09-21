/*
* timer_interface.h
*
* Author : Adam Parlane & Matthew Witt
* Created: 6/08/2017 1:23:27 PM
*
* Project Repository:https://github.com/AdamParlane/aut-swarm-robotics
*
* Sets up the timer, for the camera and 1ms interrupts
* Has delay and get ms functions and the timer handler interrupt
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void timer0Init(void)
* int get_ms(uint32_t *timestamp)
* int delay_ms(uint32_t period_ms)
* void TC1_Handler()
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "timer_interface.h"

///////////////Global Vars//////////////////////////////////////////////////////////////////////////

volatile uint16_t delaymsCounter = 0;

extern RobotGlobalStructure sys;	//gives access TC interrupt handler and delay_ms(), get_ms(),
									//and fdelay_ms() access to sys.timeStamp

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void timer0Init(void)
*
* Initializes timer0 and timer counter 1
* Used to time events with a 1ms interrupt on RC compare match
* Sets timr0 CLK speed to 12.5MHz for camera
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Gives Clock access to timer counter 0 and 1
* Note	REG_TC0_CMR0 is TC0 channel 0
*		Reg_TC0_CMR1 is TC0 channel 1
*		For some reason this is a bit strange as the interrupt will only work as TC1
*		This seems to act as the interrupt for channel 1 regardless of if it is timer0 or timer1
*
* Channel 0 is for the Clock and is set at 50MHz using divisor clock 1
* The camera uses set on RA compare (RA is 2 giving f = 25MHz)
* The camera also has clear on RC compare (RC is 4 giving 12.5MHz)
* I believe RC is the main counter for the cameras
* This channel is then enabled and the clock started
*
* Channel 1 is for generating ms timing triggers and is set to 3.125MHz using divisor clock3
* An interrupt is setup  (named TC1_Handler) to trigger on RC compare match
* RC = 3125 therefore match every 3.125MHz / 3125 = 1kHz -> 1ms
* This channel is then enabled and the clock started
* 
* Improvements:
* Find out more about why camera has 2 compare matches
*
*/
void timer0Init(void)
{
	////TIMER0////
	//Timer0 is used for delay_ms and get_ms functions required by the imu driver
	REG_PMC_PCER0
	|=	(1<<ID_TC0)						//Enable TC clock (ID_TC0 is the peripheral identifier
	|	(1<<ID_TC1);					//for timer counter 0)	
	REG_PIOA_PDR |= (1<<0);				// enable TCO control of XCLK for Camera (PA0)
	NVIC_EnableIRQ(ID_TC1);				//Enable interrupts
	REG_TC0_CMR0						//TC Channel Mode Register (Pg877)
	|=	TC_CMR_TCCLKS_TIMER_CLOCK1		//Prescaler MCK/2 (100MHz/2 = 50MHz)
	|	TC_CMR_WAVE						//Waveform mode
	|	TC_CMR_WAVSEL_UP_RC				//Clear on RC compare
	|	TC_CMR_ACPA_SET					// Set pulse on RA compare
	|	TC_CMR_ACPC_CLEAR;				// Clear pulse on RC compare
	REG_TC0_CMR1						//TC Channel Mode Register (Pg877)
	|=	TC_CMR_TCCLKS_TIMER_CLOCK3		//Prescaler MCK/32 (100MHz/32 = MHz)
	|	TC_CMR_WAVE						//Waveform mode
	|	TC_CMR_WAVSEL_UP_RC				//Clear on RC compare
	|	TC_CMR_ACPA_SET					//Set pulse on RA compare
	|	TC_CMR_ACPC_CLEAR;				//Clear pulse on RC compare
	REG_TC0_IER1						//TC interrupt enable register
	|=	TC_IER_CPCS;					//Enable Register C compare interrupt
	REG_TC0_RC1							//Set Register C (the timer counter value at which the
	|=	(TC_RC_RC(3125));				//interrupt will be triggered) Trigger once every ms 
										//(100MHz/2/1M)
	REG_TC0_RA0							//RA set to 2 counts
	|=	(TC_RA_RA(2));
	REG_TC0_RC0							//RC set to 4 counts (total square wave of 80ns period,
	|=	(TC_RC_RC(4));					//12.5MHZ)
	REG_TC0_CCR0						//Clock control register
	|=	TC_CCR_CLKEN					//Enable the timer clk.
	|	TC_CCR_SWTRG;					//Start timer register counter
	REG_TC0_CCR1						//Clock control register
	|=	TC_CCR_CLKEN					//Enable the timer clk.
	|	TC_CCR_SWTRG;					//Start timer register counter	
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
* Retrieves the value stored in sys.timeStamp (stores the number of millisecs that have elapsed
* since power on) and drops it at the address given by *timestamp. if *timestamp is an invalid
* address then returns a 1.
*
*/
int get_ms(uint32_t *timestamp)
{
	if(!timestamp)
		return 1;
	*timestamp = sys.timeStamp;
	return 0;
}

/*
* Function: int delay_ms(uint32_t period_ms)
*
* Halts execution for desired number of milliseconds. 
* Required by the IMU drivers (hence naming convention).
*
* Inputs:
* period_ms is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
* Implementation:
* The function uses a while loop to delay for period_ms number of milliseconds using TC1
* Each time 1 ms has occurred (delaymsCounter, a flag set by TC1_handler)
* Period_ms is decremented until it reaches 0 then the while loop quits and the function returns 0
*
*/
int delay_ms(uint32_t period_ms)
{
	while(period_ms > 0)
	{
		if(delaymsCounter)
		{
			delaymsCounter = 0;
			period_ms --;
		}
	}
	return 0;
}

/*
* Function:
* uint8_t fdelay_ms(uint32_t period_ms)
*
* Multi-task friendly delay
*
* Inputs:
* uint32_t period_ms
*   Delay in ms
*
* Returns:
* 0 when time is up, otherwise 1
*
* Implementation:
* A state machine governs this timer. The start state will read off the time that the delay started
* from the system time stamp. From here it moves to the wait state. If the system time stamp isn't
* greater than the start time plus the delay period, then time isn;t up, so return 1. If the desired
* amount of time has elapsed, then move to the stop state, and reset the function. Return 0 when
* done.
*
*/
uint8_t fdelay_ms(uint32_t period_ms)
{
	enum {START, WAIT, STOP};
	static uint8_t delayState = START;
	static uint32_t startTime;
	uint32_t timeStamp;

	if(!period_ms)
		delayState = STOP;
	
	switch(delayState)
	{
		case START:
			get_ms(&startTime);
			delayState = WAIT;
		break;
		
		case WAIT:
			get_ms(&timeStamp);
			if(timeStamp > startTime + period_ms)
			delayState = STOP;
		break;
		
		case STOP:
			delayState = START;
			return 0;
		break;
	}
	return 1;
}

/*
* Function: void TC1_Handler()
*
* Timer Counter 1 interrupt handler for get_ms, delay_ms and other various timing requirements
* Triggered every 1ms
* 
* Inputs:
* none
*
* Returns:
* None
*
* Implementation:
* Interrupt handler for Timer Counter 1 
* Triggered every 1ms using register C compare match
* Used to help implement get_ms() and delay_ms() functions required by the IMU driver. 
* Every ms it increments the sys.timeStamp, streamDelayCounter and delaymsCounter
* streamDelayCounter is used to send test data to the PC every 100ms so this counts to 100ms
* and then sets a flag letting main() know its time to send the next set of data
* Also includes some IMU manual stuff for V1
*
*/
void TC1_Handler()
{
	//The interrupt handler for timer counter 1
	//Triggers every 1ms
	if(REG_TC0_SR1 & TC_SR_CPCS)	//If RC compare flag (once every ms)
	{
		sys.timeStamp++;//used for get ms
		delaymsCounter++;//used for delay ms
	}
}

