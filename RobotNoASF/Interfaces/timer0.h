/*
* timer0.h
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
 


#ifndef TIMER0_H_
#define TIMER0_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

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
*/
void timer0Init(void);

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
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
uint8_t fdelay_ms(uint32_t period_ms);

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
*/
void TC1_Handler();

#endif /* TIMER0_H_ */