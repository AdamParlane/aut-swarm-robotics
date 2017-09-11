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
* void motorInit(void);
* char rearMotorDrive(signed char speed)
* char frontRightMotorDrive(signed char speed)
* char frontLeftMotorDrive(signed char speed)
* void moveRobot(float direction, unsigned char speed);
* void stopRobot(void);
* void rotateRobot(signed char speed);
* void setTestMotors(uint8_t motorData[]);
*
*/

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include <fastmath.h>
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//[[[IMPORTANT]]]
//Forward drives the motors in a direction that gets the robot moving in a clockwise direction.
//Reverse drives the motors in a direction that gets the robot moving in an anti-clockwise direction
//****Motor Pins***//
//Rear motor (1)
#define	rearFwdLo				(REG_PIOC_CODR |= (1<<23))
#define	rearFwdHi				(REG_PIOC_SODR |= (1<<23))
#define	rearRevLo				(REG_PIOC_CODR |= (1<<22))
#define	rearRevHi				(REG_PIOC_SODR |= (1<<22))
	
//Front right motor (2)
#define	frontRightFwdLo			(REG_PIOC_CODR |= (1<<19))
#define	frontRightFwdHi			(REG_PIOC_SODR |= (1<<19))
#define	frontRightRevLo			(REG_PIOA_CODR |= (1<<31))
#define	frontRightRevHi			(REG_PIOA_SODR |= (1<<31))
	
//Front left motor (3)
#define	frontLeftFwdLo			(REG_PIOA_CODR |= (1<<29))
#define	frontLeftFwdHi			(REG_PIOA_SODR |= (1<<29))
#define	frontLeftRevLo			(REG_PIOC_CODR |= (1<<10))
#define	frontLeftRevHi			(REG_PIOC_SODR |= (1<<10))

//Motor control macros
#define frontLeftMotorCW		{frontLeftFwdHi; frontLeftRevLo;}
#define frontLeftMotorCCW		{frontLeftRevHi; frontLeftFwdLo;}
#define frontLeftMotorStop		{frontLeftFwdLo; frontLeftRevLo;}
#define frontLeftMotorBrake		{frontLeftFwdHi; frontLeftRevHi;}
#define frontRightMotorCW		{frontRightFwdHi; frontRightRevLo;}
#define frontRightMotorCCW		{frontRightRevHi; frontRightFwdLo;}
#define frontRightMotorStop		{frontRightFwdLo; frontRightRevLo;}
#define frontRightMotorBrake	{frontRightFwdHi; frontRightRevHi;}
#define rearMotorCW				{rearFwdHi; rearRevLo;}
#define rearMotorCCW			{rearRevHi; rearFwdLo;}
#define rearMotorStop			{rearFwdLo; rearRevLo;}
#define rearMotorBrake			{rearFwdHi; rearRevHi;}
	
//PWM duty cycle channels
#define frontLeftPwm			REG_PWM_CUPD1
#define frontRightPwm			REG_PWM_CUPD2
#define rearPwm					REG_PWM_CUPD3

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void motorInit(void)
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
void motorInit(void);

/*
*
* Function:
* char rearMotorDrive(signed char speed)
*
* Runs rear motor at desired speed and direction. Negative speed value will make robot turn to the
* left (CCW), whereas positive speed will make robot turn to the right (CW)
*
* Inputs:
* char speed -100- +100
*
* Returns:
* char: 0 if success
*		1 if speed is out of range
*
*/
char rearMotorDrive(signed char speed);

/*
*
* Function:
* char frontRightMotorDrive(signed char speed)
*
* Runs front right motor at desired speed and direction. Negative speed value will make robot turn
* to the left (CCW), whereas positive speed will make robot turn to the right (CW)
*
* Inputs:
* char speed -100- +100
*
* Returns:
* char: 0 if success
*		1 if speed is out of range
*
*/
char frontRightMotorDrive(signed char speed);

/*
*
* Function:
* char frontLeftMotorDrive(signed char speed)
*
* Runs front left motor at desired speed and direction. Negative speed value will make robot turn
* to the left (CCW), whereas positive speed will make robot turn to the right (CW)
*
* Inputs:
* char speed -100- +100
*
* Returns:
* char: 0 if success
*		1 if speed is out of range
*
*/
char frontLeftMotorDrive(signed char speed);

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
* Function:
* uint8_t steerRobot(uint8_t speed, int8_t turnRatio)
*
* Allows robot to turn while moving forward.
*
* Inputs:
* uint8_t speed:
*   A percentage of maximum speed (0-100%)
* int8_t turnRatio:
*   The ratio of rotation to be applied to the motion (+-%). if turnRatio is 0%, then robot just
*   drives straight at 'speed'. -100% will have robot rotating on the spot anti-clockwise at
*   'speed'. 50% would be half and half driving forward with a clockwise rotational element applied.
*
* Returns:
* 0 on success
*
*/
uint8_t steerRobot(uint8_t speed, int8_t turnRatio);

#endif /* MOTOR_DRIVER_H_ */