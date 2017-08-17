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
* void wiggleForward(uint8_t forwardSpeed, uint8_t lateralSpeed, uint8_t direction)
* void stopRobot(void);
* void rotateRobot(signed char speed);
* void dfDockRobot(void);
* void setTestMotors(uint8_t motorData[]);
* char motor1Drive(signed char speed)
* char motor2Drive(signed char speed)
* char motor3Drive(signed char speed)
*
*/

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include <fastmath.h>
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Miscellaneous
 // just changed these from 0 and 1, hopefully doesnt change anything

//****Motor Pins***//
//	Robot Version 1 pin assignment
#if defined ROBOT_TARGET_V1
//Rear motor (1)
#define	rearFwdLo				(REG_PIOB_CODR |= (1<<12))
#define	rearFwdHi				(REG_PIOB_SODR |= (1<<12))
#define	rearRevLo				(REG_PIOC_CODR |= (1<<22))
#define	rearRevHi				(REG_PIOC_SODR |= (1<<22))
#define rearMotorForward		{rearFwdHi; rearRevLo;}
#define rearMotorReverse		{rearRevHi; rearFwdLo;}
#define rearMotorStop			{rearFwdLo; rearRevLo;}
#define rearMotorBrake			{rearFwdHi; rearRevHi;}
	
//Front right motor (2)
#define	frontRightFwdLo			(REG_PIOC_CODR |= (1<<19))
#define	frontRightFwdHi			(REG_PIOC_SODR |= (1<<19))
#define	frontRightRevLo			(REG_PIOA_CODR |= (1<<31))
#define	frontRightRevHi			(REG_PIOA_SODR |= (1<<31))
#define frontRightMotorForward	{frontRightFwdHi; frontRightRevLo;}
#define frontRightMotorReverse	{frontRightRevHi; frontRightFwdLo;}
#define frontRightMotorStop		{frontRightFwdLo; frontRightRevLo;}
#define frontRightMotorBrake	{frontRightFwdHi; frontRightRevHi;}
	
//Front left motor (3)
#define	frontLeftFwdLo			(REG_PIOA_CODR |= (1<<29))
#define	frontLeftFwdHi			(REG_PIOA_SODR |= (1<<29))
#define	frontLeftRevLo			(REG_PIOA_CODR |= (1<<30))
#define	frontLeftRevHi			(REG_PIOA_SODR |= (1<<30))
#define frontLeftMotorForward	{frontLeftFwdHi; frontLeftRevLo;}
#define frontLeftMotorReverse	{frontLeftRevHi; frontLeftFwdLo;}
#define frontLeftMotorStop		{frontLeftFwdLo; frontLeftRevLo;}
#define frontLeftMotorBrake		{frontLeftFwdHi; frontLeftRevHi;}
#endif

//	Robot Version 2 pin assignment
#if defined ROBOT_TARGET_V2
//Rear motor (1)
#define	rearFwdLo				(REG_PIOC_CODR |= (1<<23))
#define	rearFwdHi				(REG_PIOC_SODR |= (1<<23))
#define	rearRevLo				(REG_PIOC_CODR |= (1<<22))
#define	rearRevHi				(REG_PIOC_SODR |= (1<<22))
#define rearMotorForward		{rearFwdHi; rearRevLo;}
#define rearMotorReverse		{rearRevHi; rearFwdLo;}
#define rearMotorStop			{rearFwdLo; rearRevLo;}
#define rearMotorBrake			{rearFwdHi; rearRevHi;}
	
//Front right motor (2)
#define	frontRightFwdLo			(REG_PIOC_CODR |= (1<<19))
#define	frontRightFwdHi			(REG_PIOC_SODR |= (1<<19))
#define	frontRightRevLo			(REG_PIOA_CODR |= (1<<31))
#define	frontRightRevHi			(REG_PIOA_SODR |= (1<<31))
#define frontRightMotorForward	{frontRightFwdHi; frontRightRevLo;}
#define frontRightMotorReverse	{frontRightRevHi; frontRightFwdLo;}
#define frontRightMotorStop		{frontRightFwdLo; frontRightRevLo;}
#define frontRightMotorBrake	{frontRightFwdHi; frontRightRevHi;}
	
//Front left motor (3)
#define	frontLeftFwdLo			(REG_PIOA_CODR |= (1<<29))
#define	frontLeftFwdHi			(REG_PIOA_SODR |= (1<<29))
#define	frontLeftRevLo			(REG_PIOC_CODR |= (1<<10))
#define	frontLeftRevHi			(REG_PIOC_SODR |= (1<<10))
#define frontLeftMotorForward	{frontLeftFwdHi; frontLeftRevLo;}
#define frontLeftMotorReverse	{frontLeftRevHi; frontLeftFwdLo;}
#define frontLeftMotorStop		{frontLeftFwdLo; frontLeftRevLo;}
#define frontLeftMotorBrake		{frontLeftFwdHi; frontLeftRevHi;}
#endif

//PWM duty cycle channels
#define frontLeftPwm			REG_PWM_CUPD1
#define frontRightPwm			REG_PWM_CUPD2
#define rearPwm					REG_PWM_CUPD3

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
void moveRobot(signed int direction, unsigned char speed);

/*
* Function:
* void rotateRobot(signed char speed)
*
* Will rotate the robot on the spot in the given direcion and relative speed.
* Sign of speed sets direction (negative is CW, positive is CCW)
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
*/
void rotateRobot(signed char speed);

/*
* Function:
* void wiggleForward(uint8_t forwardSpeed, uint8_t lateralSpeed, uint8_t direction)
*
* Will move robot forward at desired speed forward, but will also allow front motor to be turned on
* so that direction of travel will become an arc to the left or right. Allows for line following and
* docking.
*
* Inputs:
* uint8_t forwardSpeed:
*   Percentage of full speed that the robot will move forward at (0-100)
*
* uint8_t lateralSpeed:
*   Percentage of full speed that the lateral wheel will be allowed to spin at to steer robot left
*   or right. (0-100)
*
* uint8_t direction:
*   Direction that robot will rotate towards on its arc (CW and CCW)
*
* Returns:
* none
*
*/
void wiggleForward(uint8_t forwardSpeed, uint8_t lateralSpeed, uint8_t direction);

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
*
* Function:
* void motorPWMcurve(void)
*
* Runs motor 2 at 10% duty cycle steps for 5 seconds each
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Powers motor 2 0-100% duty cycle in 5 second 10% steps
* Purpose is to test the PWM curve on each robot
*
*/
void PWMSpeedTest(void);

/*
* Function:
* void setTestMotors(uint8_t motorData[])
*
* sets test motors to perform motor test
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


/*
*
* Function:
* char rearMotorDrive(signed char speed)
*
* Runs motor 1 at desired speed and direction
*
* Inputs:
* char speed -100- +100
*
* Returns:
* char: 1 if success
*		0 if speed is out of range
*
*/
char rearMotorDrive(signed char speed);

/*
*
* Function:
* char frontRightMotorDrive(signed char speed)
*
* Runs motor 2 at desired speed and direction
*
* Inputs:
* char speed -100- +100
*
* Returns:
* char: 1 if success
*		0 if speed is out of range
*
*/
char frontRightMotorDrive(signed char speed);

/*
*
* Function:
* char frontLeftMotorDrive(signed char speed)
*
* Runs motor 3 at desired speed and direction
*
* Inputs:
* char speed -100- +100
*
* Returns:
* char: 1 if success
*		0 if speed is out of range
*
*/
char frontLeftMotorDrive(signed char speed);

#endif /* MOTOR_DRIVER_H_ */