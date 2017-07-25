/*
* motor_driver.h
*
* Author : Adam Parlane and Matthew Witt
* Created: 13/05/2017 4:18:25 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines pin assignments for the motor driver chips. Also, motor control function prototypes.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* BD6211 Motor driver Datasheet:http://rohmfs.rohm.com/en/products/databook/datasheet/ic/motor/dc/bd621x-e.pdf
*
* Functions:
* void motor_init(void);
* void moveRobot(float direction, unsigned char speed);
* void stopRobot(void);
* void rotateRobot(char direction, unsigned char speed);
* void dockRobot(void);
* void setTestMotors(uint8_t motorData[]);
*
*/

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include <fastmath.h>
#include "robot_defines.h"

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//Miscellaneous
#define CW	0
#define CCW 1

//****Motor Pins***//
//	Robot Version 1 pin assignment
#if defined ROBOT_TARGET_V1
//Motor 1
#define	FIN_1_L	(REG_PIOB_CODR |= (1<<12))
#define	FIN_1_H	(REG_PIOB_SODR |= (1<<12))
#define	RIN_1_L	(REG_PIOC_CODR |= (1<<22))
#define	RIN_1_H	(REG_PIOC_SODR |= (1<<22))
//Motor 2
#define	FIN_2_L	(REG_PIOC_CODR |= (1<<19))
#define	FIN_2_H	(REG_PIOC_SODR |= (1<<19))
#define	RIN_2_L	(REG_PIOA_CODR |= (1<<31))
#define	RIN_2_H	(REG_PIOA_SODR |= (1<<31))
//Motor 3
#define	FIN_3_L	(REG_PIOA_CODR |= (1<<29))
#define	FIN_3_H	(REG_PIOA_SODR |= (1<<29))
#define	RIN_3_L	(REG_PIOA_CODR |= (1<<30))
#define	RIN_3_H	(REG_PIOA_SODR |= (1<<30))
#endif
//	Robot Version 2 pin assignment
#if defined ROBOT_TARGET_V2
//Motor 1
#define	FIN_1_L	(REG_PIOC_CODR |= (1<<23))
#define	FIN_1_H	(REG_PIOC_SODR |= (1<<23))
#define	RIN_1_L	(REG_PIOC_CODR |= (1<<22))
#define	RIN_1_H	(REG_PIOC_SODR |= (1<<22))
//Motor 2
#define	FIN_2_L	(REG_PIOC_CODR |= (1<<19))
#define	FIN_2_H	(REG_PIOC_SODR |= (1<<19))
#define	RIN_2_L	(REG_PIOA_CODR |= (1<<31))
#define	RIN_2_H	(REG_PIOA_SODR |= (1<<31))
//Motor 3
#define	FIN_3_L	(REG_PIOA_CODR |= (1<<29))
#define	FIN_3_H	(REG_PIOA_SODR |= (1<<29))
#define	RIN_3_L	(REG_PIOC_CODR |= (1<<10))
#define	RIN_3_H	(REG_PIOC_SODR |= (1<<10))
#endif



///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function:
* void motor_init(void)
*
* Initialises microcontroller's PWM feature and PIO on the pins connected to the motor drivers.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void motor_init(void);

/*
* Function:
* void moveRobot(float direction, unsigned char speed)
*
* Will start the robot moving in the desired heading at the desired speed
*
* Inputs:
* float direction:
*	heading in degrees in which the robot should move. Irrelevant if speed = 0
* unsigned char speed
*	Speed at which robot should move. Is a percentage of maximum speed (0-100)
*
* Returns:
*	none
*
*/
void moveRobot(float direction, unsigned char speed);

/*
* Function:
* void rotateRobot(char direction, unsigned char speed)
*
* Will rotate the robot on the spot in the given direction and relative speed.
*
* Inputs:
* char direction
*	The direction the robot should rotate (CW or CCW)
* unsigned char speed
*	Speed at which robot should move. Is a percentage of maximum speed (0-100)
*
* Returns:
* none
*
* Implementation:
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
*/
void rotateRobot(char direction, unsigned char speed);

/*
* Function:
* void dockRobot(void)
*
* Function to guide the robot to the dock.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* assume max brightness is 0-100 (scale it to make this work)
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void dockRobot(void);

/*
* Function:
* void stopRobot(void)
*
* Stop all motors
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void stopRobot(void);

/*
* Function:
* void setTestMotors(uint8_t motorData[])
*
* [brief purpose of function]
*
* Inputs:
* uint8_t motorData[]
*   two element 8bit array that contains a data packet from PC GUI relevant to the motor test
*   routine. (ie speed, direction and what motor to run)
*
* Returns:
* none
*
*/
void setTestMotors(uint8_t motorData[]);

#endif /* MOTOR_DRIVER_H_ */