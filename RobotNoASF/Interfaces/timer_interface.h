/*
* timer_interface.h
*
* Author : Adam Parlane & Matthew Witt
* Created: 6/08/2017 1:23:27 PM
*
* Project Repository:https://github.com/wittsend/aut-swarm-robotics
*
* Sets up the timer, for the camera and 1ms interrupts
* Has delay and get ms functions and the timer handler interrupt
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
*
* Functions:
* void sysTimerInit(void)
* int get_ms(uint32_t *timestamp)
* int delay_ms(uint32_t period_ms)
* int delay_us(uint32_t period_us);
*
*/
 
#ifndef TIMER_INTERFACE_H_
#define TIMER_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void sysTimerInit(void)
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
*/
void sysTimerInit(void);

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
*/
int get_ms(uint32_t *timestamp);

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
*/
int delay_ms(uint32_t period_ms);

/*
* Function: int delay_us(uint32_t period_us)
*
* Halts execution for desired number of microseconds.
*
* Inputs:
* period_us is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
* Implementation:
* see delay_ms() description
*
*/
int delay_us(uint32_t period_us);

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
*/
uint8_t fdelay_ms(uint32_t period_ms);
#endif /* TIMER_INTERFACE_H_ */