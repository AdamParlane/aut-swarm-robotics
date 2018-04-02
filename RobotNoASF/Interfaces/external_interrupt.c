/*
* external_interrupt.c
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 9/08/2017 9:26:57 AM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Contains external interrupt initialisation routine and PIOA interrupt handler
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* See page 446 for information on PIO interrupts (Section 27)
*
* Functions:
* void extIntInit(void)
* uint8_t extCamWriteToBuffer(void);
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "external_interrupt.h"
#include "imu_interface.h"
#include "camera_interface.h"
#include "camera_buffer_interface.h"
#include "../IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
#define vsyncRisingEdge		(VSYNC_PORT->PIO_REHLSR |= VSYNC_PIN)//Make VSYNC rising edge sensitive
#define vsyncFallingEdge	(VSYNC_PORT->PIO_FELLSR |= VSYNC_PIN)//Make VSYNC falling edge sensitive
#define vsyncIntDisable		(VSYNC_PORT->PIO_IDR |= VSYNC_PIN)	//Disable the interrupt on VSYNC pin
#define vsyncIntEnable		(VSYNC_PORT->PIO_IER |= VSYNC_PIN)	//Enable the interrupt on VSYNC pin
#define vsyncIntIsEnabled	(VSYNC_PORT->PIO_IMR & VSYNC_PIN)	//Says whether int is already enable

//////////////[Enumerations]////////////////////////////////////////////////////////////////////////
//This enumeration contains the states for the camera to buffer write routine (See the port C
//external interrupt handler)
typedef enum CamWriteToBuffState
{
	CWTB_LAST_FRAME_COMPLETE,
	CWTB_NEW_FRAME_BEGIN,
	CWTB_FRAME_WRITE_COMPLETE
}CamWriteToBuffState;

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//The state of the write process to the camera buffer.
volatile CamWriteToBuffState camWriteState = CWTB_LAST_FRAME_COMPLETE;
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
	NVIC_EnableIRQ(ID_PIOA);	//Enable interrupts on PIOA (For IMU interrupt)
	NVIC_EnableIRQ(ID_PIOC);	//Enable interrupts on PIOC (For camera VSYNC)
	
	//Setup IMU external interrupt pin. Pin and port is defined in imu_interface.h
	IMU_INT_PORT->PIO_IER		//Enable the interrupt on the IMU int pin
	|=	IMU_INT_PIN;
	IMU_INT_PORT->PIO_AIMER		//Enable additional interrupt modes on IMU int pin (Must be enabled
	|=	IMU_INT_PIN;			//so we can have a rising edge interrupt rather than lvl sensitive).
	IMU_INT_PORT->PIO_ESR		//Make pin sensitive to Edge rather than Level
	|=	IMU_INT_PIN;
	IMU_INT_PORT->PIO_REHLSR	//Make pin rising edge sensitive
	|=	IMU_INT_PIN;
	//Any other external interrupt configurations should follow

	//Setup the camera's VSYNC pin as an external interrupt
	VSYNC_PORT->PIO_AIMER		//Enable additional interrupt modes on VSYNC pin (Must be enabled
	|=	VSYNC_PIN;				//so we can have a rising edge interrupt rather than lvl sensitive).
	VSYNC_PORT->PIO_ESR			//Make pin sensitive to Edge rather than Level
	|=	VSYNC_PIN;
	VSYNC_PORT->PIO_REHLSR		//Make pin rising edge sensitive
	|=	VSYNC_PIN;	
}

/*
* Function:
* uint8_t extCamWriteToBuffer(void)
*
* If there is no new data in the buffer, will start the VSYNC ext interrupt to load a frame into
* the camera's FIFO buffer.
*
* Inputs:
* none
*
* Returns:
* Returns 0 if there is new data to be read from the FIFO, otherwise will return 1 while the
* write operation is being performed.
*
* Implementation:
* If the cam buffer flag is set to 1, this function will just exit with a 0. Otherwise, the function
* will check if the vsync interrupt isn;t already enabled and enable it, starting the write process
* which is handled in the interrupt service routine itself.
*
*/
uint8_t extCamWriteToBuffer(void)
{
	//If there is unread data in the buffer
	if(sys.flags.camBufferRead)
		return 0;

	//If the VSYNC interrupt is not already enabled (Indicating a retrieval is in progress)
	if(!vsyncIntIsEnabled)
	{
		vsyncRisingEdge;
		camWriteState = CWTB_LAST_FRAME_COMPLETE;
		vsyncIntEnable;
	} 
	
	return 1;
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

/*
* Function:
* void PIOC_Handler(void)
*
* Interrupt handler for parallel IO port C
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
void PIOC_Handler(void)
{
	if(VSYNC_PORT->PIO_ISR & VSYNC_PIN)		//If Vsync interrupt detected
	{
		switch(camWriteState)
		{
			//When the last [partial] frame has finished being output 
			case CWTB_LAST_FRAME_COMPLETE:
				camBufferWriteResetOn;					//Reset buffer write pointer
				vsyncFallingEdge;						//Look for falling edge on vsync next time
				camWriteState = CWTB_NEW_FRAME_BEGIN;	//Move to the next state next time
				break;
			
			//When the next frame is about to begin	
			case CWTB_NEW_FRAME_BEGIN:
				camBufferWriteResetOff;
				camBufferWriteEnable;					//Enable buffer write
				vsyncRisingEdge;						//Look for rising edge next time
				camWriteState = CWTB_FRAME_WRITE_COMPLETE;//At the next int, the transfer is complt
				break;
			
			//When the transfer is complete
			case CWTB_FRAME_WRITE_COMPLETE:
				vsyncIntDisable;						//Disable the interrupt
				camBufferWriteDisable;					//Buffer write disable
				camWriteState = CWTB_LAST_FRAME_COMPLETE;//Revert to default state
				sys.flags.camBufferRead = 1;			//Set buffer read flag
				break;
		}
	}
}