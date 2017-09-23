/*
* motor_driver.c
*
* Author : Adam Parlane and Matthew Witt
* Created: 13/05/2017 4:16:25 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Provides functions for controlling the motors and moving the robot.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* BD6211 Motor driver Datasheet:http://rohmfs.rohm.com/en/products/databook/datasheet/ic/motor/dc/bd621x-e.pdf
*
* The 3 motors are driven by 1 H-Bridge each with a differential output on pins OUT1 & OUT2
* This controls the speed and direction of the motor
* The H-Bridges are controlled by the micro using 3 Pins per H-Bridge and PWM
* Each H-Bridge uses FIN_x, RIN_x & VREF_x (where x is the motor number)
* FIN and RIN control the motor direction, 1 and only 1 must be high to enable the desired direction
* VREF uses PWM with duty cycle 0-100(%) to set the speed of the motor
*
* Functions:
* void motorInit(void);
* void findMotorMinSpeeds(void);
* char rearMotorDrive(signed char speed)
* char frontRightMotorDrive(signed char speed)
* char frontLeftMotorDrive(signed char speed)
* void mdStopMotors(void);
* void setTestMotors(uint8_t motorData[]);
* void moveRobot(float direction, float speed, float turnRatio);
*
*/
 
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "motor_driver.h"
#include "../Functions/test_functions.h"
#include "../Functions/navigation_functions.h"	//nfWrapAngle in moveRobot()
#include "opt_interface.h"					//detectMouseMove() for findMinSpeed
#include "timer_interface.h"				//delay_ms()
#include <stdlib.h>							//abs()
#include <tgmath.h>							//Trigonometry

//////////////[Global Variables]////////////////////////////////////////////////////////////////////
uint8_t motorMinSpeed[3] = {0, 0, 0};	//Stores the minimum speeds for the motors

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void motorInit(void)
*
* Initializes micro controller's PWM feature and PIO on the pins connected to the motor drivers.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Firstly for ROBOT V1 PB12 is used as FIN_1, this is the erase pin
* so the erase functionality is disabled to allow PIO access of PB12
* The PMC is then given master clock access to PWM (PCER0 PID31)
* Motor 1 uses channel 3, motor 2 on channel 2 and motor 3 on channel 1
* In the PWM Channel Mode Register (PWM_CMRx) the following are set
* For each channel the prescale is set to CLK/1
* The PWM output starts at high level (this shouldnt make a difference in the program, high or low)
* The output is left aligned (again this could go either way as long as its consistent)
* Most importantly we want to update the DUTY CYCLE not the period in order to change the motors'
* speeds easily
* The starting duty cycle is set to 0 in PWM_CDTYx as the motors are to initialize as off
* The PWM counter is set to 100, this allows the duty cycle to simply be a percentage
* Using the PIOx_PDR control of VREFx is given to the peripheral, 
* the next line states that peripheral is PWM peripheral B (same for all channels)
*
* Now that PWM is setup PIO is used to PIO control of FIN_x and RIN_x 
* Then the Output Enable Register (OER) is used to enable FIN_x and RIN_x as outputs
* 
* Now that everything is setup the PWM channels are enabled with the PWM_ENA registers

* Improvements:
* TODO: convert code to be using the SAM4N macros
*
*/
void motorInit(void)
{
	REG_PMC_PCER0 |= PMC_PCER0_PID31;	//Enable clock access for PWM
	
	//****Rear Channel 3 (Motor 1)****//
	REG_PWM_CMR3 |= (0x4<<0);		//Channel pre scale CLK by 16 = 24.4KHz
	REG_PWM_CMR3 |= (1<<9);			//output starts at high level
	REG_PWM_CMR3 &= ~(1<<8);		//Left aligned output
	REG_PWM_CMR3 &= ~(1<<10);		//Update Duty cycle (NOT period)
	REG_PWM_CDTY3 = 0;				//PWM Duty cycle (default = 0)
	REG_PWM_CPRD3 = 100;			//PWM Counter 0 - 100 (makes duty cycle %)
		
	REG_PIOC_PDR |= (1<<21);		//Enable peripheral control of PC21
	REG_PIOC_ABCDSR = (1<<21);		//Assign PC21 to PWM Peripheral B		
	REG_PIOC_PER |= (1<<23);		//Enable PIO control of PB12
	REG_PIOC_OER |= (1<<23);		//Set PB12 as output

	rearFwdLo;		
	REG_PIOC_PER |= (1<<22);		//Enable PIO control of PC22
	REG_PIOC_OER |= (1<<22);		//Set PC22 as output
	rearRevLo;
								
	//****Front right Channel 2 (Motor 2)****//
	REG_PWM_CMR2 |= (0x4<<0);		//Channel pre scale CLK by 16 = 24.4KHz
	REG_PWM_CMR2 |= (1<<9);			//output starts at low level
	REG_PWM_CMR2 &= ~(1<<8);		//Left aligned output
	REG_PWM_CMR2 &= ~(1<<10);		//Update Duty cycle (NOT period)
	REG_PWM_CDTY2 = 0;				//PWM Duty cycle (default = 0)
	REG_PWM_CPRD2 = 100;			//PWM Counter 0 - 100 (makes duty cycle %)
		
	REG_PIOC_PDR |= (1<<20);		//Enable peripheral control of PC20
	REG_PIOC_ABCDSR = (1<<20);		//Assign VREF_1 to PWM Peripheral B			
	REG_PIOC_PER |= (1<<19);		//Enable PIO control of PC19
	REG_PIOC_OER |= (1<<19);		//Set PC19 as output
	frontRightRevLo;		
	REG_PIOA_PER |= (1<<31);		//Enable PIO control of PA31
	REG_PIOA_OER |= (1<<31);		//Set PA31 as output
	frontRightFwdLo;		

	//****Front left Channel 1 (Motor 3)****//
	REG_PWM_CMR1 |= (0x4<<0);		//Channel pre scale CLK by 16 = 24.4KHz
	REG_PWM_CMR1 |= (1<<9);			//output starts at low level
	REG_PWM_CMR1 &= ~(1<<8);		//Left aligned output
	REG_PWM_CMR1 &= ~(1<<10);		//Update Duty cycle (NOT period)
	REG_PWM_CDTY1 = 0;				//PWM Duty cycle (default = 0)
	REG_PWM_CPRD1 = 100;			//PWM Counter 0 - 100 (makes duty cycle %)
		
	REG_PIOC_PDR |= (1<<9);			//Enable peripheral control of PC9
	REG_PIOC_ABCDSR = (1<<9);		//Assign PC9 to PWM Peripheral B		
	REG_PIOA_PER |= (1<<29);		//Enable PIO control of PA30
	REG_PIOA_OER |= (1<<29);		//Set PA30 as output
	frontLeftRevLo;		
	REG_PIOC_PER |= (1<<10);		//Enable PIO control of PC10
	REG_PIOC_OER |= (1<<10);		//Set PC10 as output
	frontLeftFwdLo;
	
	//****Enable PWM Channels as last step of setup****//	
	REG_PWM_ENA |= PWM_ENA_CHID1;	//Enable PWM on channel 1
	REG_PWM_ENA |= PWM_ENA_CHID2;	//Enable PWM on channel 2
	REG_PWM_ENA |= PWM_ENA_CHID3;	//Enable PWM on channel 3
	
	//delay_ms(5000);	//Robot needs to be still for this
	//findMotorMinSpeeds();
}

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
* Implementation:
* - Resets the PWM of each motor to 0 duty
* - Sets Cloclwise direction
* - Starts a for loop that will iterate once for each motor
* - Waits 100ms, then checks that there has been no movement on the mouse. If there has then wait
*   another 100ms
* - Slowly increase the PWM duty of the current motor until movement is detected by the mouse. When
*   movement is detected then store that PWM number is the minimum and move on to the next motor
*
*/
void findMotorMinSpeeds(void)
{
	frontRightPwm = 0;
	frontLeftPwm = 0;
	rearPwm = 0;
	
	frontRightMotorCW;
	frontLeftMotorCW;
	rearMotorCW;

	for(uint8_t motor = 0; motor < 3; motor++)
	{	
		//Clear mouse buffer of residual data
		do 
		{
			delay_ms(100);
		} while (detectMouseMove(1));
		for(uint8_t i = 0; i < 100; i++)
		{
			switch(motor)
			{
				case FR_MOTOR:
					frontRightPwm = i;
					frontLeftPwm = 0;
					rearPwm = 0;
					break;
					
				case FL_MOTOR:
					frontLeftPwm = i;
					frontRightPwm = 0;
					rearPwm = 0;
					break;
					
				case R_MOTOR:
					rearPwm = i;
					frontRightPwm = 0;
					frontLeftPwm = 0;
					break;
			}
			detectMouseMove(10);
			delay_ms(100);
			if(detectMouseMove(10))
			{
				motorMinSpeed[motor] = i;
				i = 100;
				frontRightPwm = 0;
				frontLeftPwm = 0;
				rearPwm = 0;
			}
		}
	}

}

/*
*
* Function:
* char rearMotorDrive(signed char speed)
*
* Runs rear motor at desired speed and direction. Negative speed value will make robot turn to the 
* left (CCW), whereas positive speed will make robot turn to the right (CW)
*
* Inputs:
* char speed -100 - +100
*
* Returns:
* char: 0 if success
*		1 if speed is out of range
*
* Implementation:
* checks speed is in range
* updates duty cycle to speed
* sets motor direction based on whether speed is positive (forwards) or negative (reverse)
*
*/
char rearMotorDrive(signed char speed)
{
	speed = capToRangeInt(speed, -100, 100);	//Make sure speed is in range
	//rearPwm = abs((int)((100 - motorMinSpeed[R_MOTOR])/100.0*speed)) + motorMinSpeed[R_MOTOR];
	rearPwm = abs(speed);
	if(speed > 0)		//Forwards
		rearMotorCW;
	if(speed < 0)		//Reverse
		rearMotorCCW;
	if(speed == 0)	
		rearMotorStop;
	return 0;
}

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
* Implementation:
* checks speed is in range
* updates duty cycle to speed
* sets motor direction based on whether speed is positive (forwards) or negative (reverse)
*
*/
char frontRightMotorDrive(signed char speed)
{
	speed = capToRangeInt(speed, -100, 100);	//Make sure speed is in range
	//frontRightPwm 
	//=	abs((int)((100 - motorMinSpeed[FR_MOTOR])/100.0*speed)) + motorMinSpeed[FR_MOTOR];
	frontRightPwm = abs(speed);
	if(speed > 0)		//Forwards
		frontRightMotorCW;
	if(speed < 0)	//Reverse
		frontRightMotorCCW;
	if(speed == 0)	
		frontRightMotorStop;
	return 0;	//Always return 0 on success, non zero on error
}

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
* Implementation:
* checks speed is in range
* updates duty cycle to speed
* sets motor direction based on whether speed is positive (forwards) or negative (reverse)
*
*/
char frontLeftMotorDrive(signed char speed)
{
	speed = capToRangeInt(speed, -100, 100);	//Make sure speed is in range
	//frontLeftPwm 
	//=	abs((int)((100 - motorMinSpeed[FL_MOTOR])/100.0*speed)) + motorMinSpeed[FL_MOTOR];
	frontLeftPwm = abs(speed);
	if(speed > 0)	//Forwards
		frontLeftMotorCW;
	if(speed < 0)	//Reverse
		frontLeftMotorCCW;
	if(speed == 0)	
		frontLeftMotorStop;
	return 0;
}

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
* Implementation:
* Sets all direction pins on the motor drivers low.
*
*/
void mdStopMotors(void)
{
	//Stops the robot from moving
	sys.flags.obaMoving = 0;
	frontRightMotorDrive(0);
	frontLeftMotorDrive(0);
	rearMotorDrive(0);
}

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
* Implementation:
* For the robot test motorData[0] describes which motor to test
* motorData[1] describes both the speed (bits 0-6) and direction (bit 7) of the motor
* The speed must be 0-100
* The direction can be set meaning forward or cleared meaning reverse
* Each if statement is checking which motor and which direction
* Once the correct if statement is entered it will set the correct motor and direction
* The speed will be used to update the duty cycle 
* with bit masking to ensure only the first 7 bits are used
*
*/
void setTestMotors(uint8_t motorData[])
{
	if(motorData[0] == REAR_MOTOR && (motorData[1] & 0x80))			//if bit 7 is set then CW
		rearMotorDrive((motorData[1] & 0x7F));						//Rear motor, Clockwise
	else if(motorData[0] == REAR_MOTOR && ~(motorData[1] & 0x80))	//else CCW
		rearMotorDrive(-(motorData[1] & 0x7F));						//Rear motor, AntiClockwise
		
	else if(motorData[0] == F_RIGHT_MOTOR && (motorData[1] & 0x80))	//if bit 7 is set then CW
		frontRightMotorDrive((motorData[1] & 0x7F));				//FR motor, Clockwise
	else if(motorData[0] == F_RIGHT_MOTOR && ~(motorData[1] & 0x80))//else CCW
		frontRightMotorDrive(-(motorData[1] & 0x7F));				//FR motor, AntiClockwise
		
	else if(motorData[0] == F_LEFT_MOTOR && (motorData[1] & 0x80))	//if bit 7 is set then CW
		frontLeftMotorDrive((motorData[1] & 0x7F));					//FL motor, Clockwise
	else if(motorData[0] == F_LEFT_MOTOR && ~(motorData[1] & 0x80))	//else CCW
		frontLeftMotorDrive(-(motorData[1] & 0x7F));				//FL motor, AntiClockwise
}

/*
*
* Function:
* void PWMSpeedTest(void)
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
void PWMSpeedTest(void)
{
	//Stops the robot from moving
	frontRightFwdHi;
	frontRightRevLo;
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 10;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 20;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 30;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 40;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 50;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 60;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 70;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 80;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 90;
	delay_ms(5000);
	frontRightPwm = 0;
	delay_ms(5000);
	frontRightPwm = 100;
	delay_ms(5000);
	mdStopMotors();
}

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
* How to achieve old rotateRobot(-60) behaviour:
*   moveRobot(0, -60, 100) <-- turnRatio is 100 and heading is ignored
*          --OR--
*   moveRobot(0, 60, -100)
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
* Implementation:
* First, the heading is converted to radians to allow smooth working with cos()
* Then check that speed and turnRatio are in the correct range and fix if necessary.
* Next calculate the speed ratios for forward motion and rotational motion based on the speed and
* turn ratio passed to the function. forwardSpeed is inversely proportional to the absolute value
* of rotational speed. 

* Each motors' speed is calculated using the FORWARD DRIVE DIRECTION of the motor, with respect to
* the front face of the robot
* These are:	Motor 1		270 deg 
*				Motor 2		30  deg
*				Motor 3		150 deg
* The robot speed is then cos(motor direction - desired robot direction) (performed in radians)
* This will produce a ratio up to 1 of the full speed in order to achieve the correct direction
* This is also multiplied by the straight line speed to give the speed that the robot will be moving
* in a straight line. Finally, the rotational speed component is added to this value.
*
* Lastly, apply speeds to the motors.
*
*/
uint8_t moveRobot(float heading, float speed, float turnRatio)
{
	int8_t rearMotorSpeed, frontRightMotorSpeed, frontLeftMotorSpeed;
	
	heading = nfWrapAngle(heading);
	
	//If speed is set to 0, then save processor cycles
	if(speed == 0.0)
	{
		mdStopMotors();
		return 0;
	}
	
	float headingRad = -heading*M_PI/180.0; //convert desired direction to radians (and invert)
		
	//Make sure parameters are in range and correct if necessary
	speed = capToRangeFlt(speed, -100, 100);
	turnRatio = capToRangeFlt(turnRatio, -100, 100);
	
	//Calculate speed ratios
	float rotationalSpeed = speed*(turnRatio/100.0);
	float straightSpeed = abs(speed)-(abs(rotationalSpeed));
	
	//Calculate individual motor speeds
	rearMotorSpeed			= -straightSpeed*cos(RM_ANGLE_RAD - headingRad) + rotationalSpeed;
	frontRightMotorSpeed	= -straightSpeed*cos(FRM_ANGLE_RAD - headingRad) + rotationalSpeed;
	frontLeftMotorSpeed		= -straightSpeed*cos(FLM_ANGLE_RAD - headingRad) + rotationalSpeed;
	
	//Apply speeds and directions to motors
	frontRightMotorDrive(frontRightMotorSpeed);
	frontLeftMotorDrive(frontLeftMotorSpeed);
	rearMotorDrive(rearMotorSpeed);
	
	return 0;
}
