/*
 * timer0.h
 *
 * Created: 6/08/2017 1:23:27 PM
 *  Author: adams
 */ 


#ifndef TIMER0_H_
#define TIMER0_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"

///////////////Functions////////////////////////////////////////////////////////////////////////////
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