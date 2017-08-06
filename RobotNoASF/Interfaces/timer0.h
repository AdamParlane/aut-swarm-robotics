/*
 * timer0.h
 *
 * Created: 6/08/2017 1:23:27 PM
 *  Author: adams
 */ 


#ifndef TIMER0_H_
#define TIMER0_H_

#include "../robot_defines.h"

//Flags and system globals
uint32_t systemTimestamp = 0;	//Number of ms since powerup. Used by delay_ms and get_ms functions
								//Which in turn are used by the IMU drivers/DMP
#if defined ROBOT_TARGET_V1
uint32_t imuFifoNextReadTime = 0;//The system time at which the IMU will be read next (ie when
//checkImuFifo will next be set to one. Used by the V1 robot
//only as the V2 sets checkImuInfo from external interrupt from
//The IMU.
#endif


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
* Required by the IMU drivers (hence naming convention). Halts execution for desired number of
* milliseconds.
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
*/
void TC0_Handler();

#endif /* TIMER0_H_ */