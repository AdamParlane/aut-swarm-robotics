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
* void mdStopMotors(void);
* void setTestMotors(uint8_t motorData[]);
* void moveRobot(float heading, float speed, float turnRatio);
*
*/

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

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

//Pre-calculated wheel specific angles in Radians:
#define RM_ANGLE_RAD			4.712388980		//270*PI/180
#define FRM_ANGLE_RAD			0.523598775		//30*PI/180
#define FLM_ANGLE_RAD			2.617993877		//150*PI/180

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
* void mdStopMotors(void)
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
void mdStopMotors(void);

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
* uint8_t moveRobot(float heading, float speed, float turnRatio)
*
* Provides total control of the motion of the robot. This function replaces the old moveRobot(),
* rotateRobot() and steerRobot() functions as it is a combination of all three. It should allow us
* to achieve our aim to get the robot to rotate as it moves to a target heading.
*
* How to achieve old moveRobot() behaviour:
*   moveRobot(90, 50, 0) <-- turnRatio set to 0
*
* How to achieve old rotateRobot() behaviour:
*   moveRobot(0, 60, 100) <-- turnRatio is 100 and heading is ignored
*
* Inputs:
* float heading:
*   The heading the the robot will move in (degrees). If turnRatio is 100 then heading has no effect
* float speed:
*   A percentage of maximum speed (-100% to 100%) and direction of rotation (<0 is CCW and >0 is CW)
*   If turnRatio is 0 then signedness of speed has no effect.
* float turnRatio:
*   The ratio of rotation to be applied to the motion (-100% to 100%). if turnRatio is 0%, then
*   robot just drives straight at 'speed'. -100% will have robot rotating CCW on the spot at
*   'speed'. 50% would be half and half driving forward with a CW rotational element applied. If
*   both speed and turnRatio are negative, then robot will rotate in CW (-1*-1) = 1
*
* Returns:
* 0 on success
*
*/
uint8_t moveRobot(float heading, float speed, float turnRatio);

#endif /* MOTOR_DRIVER_H_ */