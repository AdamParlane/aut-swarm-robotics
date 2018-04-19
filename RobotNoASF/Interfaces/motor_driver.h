/*
* motor_driver.h
*
* Author : Adam Parlane and Matthew Witt
* Created: 13/05/2017 4:18:25 PM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Defines pin assignments for the motor driver chips. Also, motor control function prototypes.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* BD6211 Motor driver Datasheet:http://rohmfs.rohm.com/en/products/databook/datasheet/ic/motor/dc/bd621x-e.pdf
*
* Functions:
* void motorInit(void);
* void findMotorMinSpeeds(void);
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
* Function:
* void findMotorMinSpeeds(void)
*
* Finds the minimum PWM duty needed to start each motor moving
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void findMotorMinSpeeds(void);

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