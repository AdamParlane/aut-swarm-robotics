/*
* external_interrupt.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 9/08/2017 9:26:57 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Contains external interrupt initialisation routine and PIOA interrupt handler
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* See page 446 for information on PIO interrupts (Section 27)
*
* Functions:
* void extIntInit(void)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "external_interrupt.h"
#include "../Interfaces/imu_interface.h"
#include "../IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.h"

//////////////[Global variables]////////////////////////////////////////////////////////////////////
extern RobotGlobalStructure sys;	//imuCheckFifo flag needed by external interrupt

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void extIntInit(void)
*
* Initialisation for external interrupts goes in here
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* First, if building for V2 robot, the interrupt for the IMU is setup. The fisrt line enables the 
* interrupt for the defined IMU int pin. Next, the additional mode register is set for the given
* pin which allows rising edge detection on the IMU interrupt. The thirs line sets the IMU int pin
* for edge detection, and the final register sets the IMU int pin to rising edge.
*
*/
void extIntInit(void)
{
	NVIC_EnableIRQ(ID_PIOA);	//Enable interrupts on PIOA
	//Setup IMU external interrupt pin. Pin and port is defined in imu_interface.h
	IMU_INT_PORT->PIO_IER		//Enable the interrupt on the IMU int pin
	|=	IMU_INT_PIN;
	IMU_INT_PORT->PIO_AIMER		//Enable additional interrupt modes on IMU int pin (Must be enabled
	|=	IMU_INT_PIN;			//So we can have a rising edge interrupt rather than lvl sensitive)
	IMU_INT_PORT->PIO_ESR		//Make pin sensitive to level rather than edge
	|=	IMU_INT_PIN;
	IMU_INT_PORT->PIO_REHLSR	//Make pin rising edge sensitive
	|=	IMU_INT_PIN;
	//Any other external interrupt configurations should follow
}

/*
* Function:
* void PIOA_Handler(void)
*
* Interrupt handler for parallel IO port A
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* General rule of thumb is a series conditional statements that check for the appropriate bit set in
* the interrupt status register which indicates that the desired interrupt has been triggered. With
* in the conditional statement is the code that should be executed on that interrupt.
*
*/
void PIOA_Handler(void)
{
	//If the IMU interrupt has been triggered
	if(IMU_INT_PORT->PIO_ISR & IMU_INT_PIN)	//If IMU interrupt detected
	{
		sys.flags.imuCheckFifo = 1;
		//led1Tog;
	}
}