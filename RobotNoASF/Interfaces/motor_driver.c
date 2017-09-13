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
* char rearMotorDrive(signed char speed)
* char frontRightMotorDrive(signed char speed)
* char frontLeftMotorDrive(signed char speed)
* void moveRobot(float direction, unsigned char speed);
* void stopRobot(void);
* void rotateRobot(signed char speed);
* void setTestMotors(uint8_t motorData[]);
*
*/
 
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "motor_driver.h"

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
* Implementation:
* The direction is kept in range of +/-180 by removing whole chunks of 360 deg to it
* The speed is also limited at 100, so any speed over 100 is set to 100
* The direction is then converted to radians to allow smooth working with cos()
* Each motors' speed is calculated using the FORWARD DRIVE DIRECTION of the motor, with respect to
* the front face of the robot
* These are:	Motor 1		270 deg
*				Motor 2		30  deg
*				Motor 3		150 deg
* The robot speed is then cos(motor direction - desired robot direction) (performed in radians)
* This will produce a ratio up to 1 of the full speed in order to achieve the correct direction
* This is also multiplied by the desired speed to achieve an overall robot speed
* The long part with if statements simply turns on the correct FIN and RIN pins to achieve the 
* desired MOTOR direction, this is based off the speed calculation which could return negative
* Finally the ABSOLUTE value of the motorxspeed is written to the PWM_CUPD register to update
* the duty cycle.
*
* Improvements:
* None as of 26/7/17
*
*/
void moveRobot(signed int direction, unsigned char speed)
{
	signed int rearMotorSpeed, frontRightMotorSpeed, frontLeftMotorSpeed;
	float directionRad;
	direction = -1 * direction;
	//keep direction in range +/-180degrees
	//direction = nfWrapAngle(direction);

	//stop speed from being over max in case of user input error
	speed = capToRangeUint(speed, 0, 100);

	directionRad = (direction * M_PI) / 180; //convert desired direction to radians
	rearMotorSpeed = -1 * (speed * cos ((270 * M_PI) / 180 - directionRad ));//radians
	frontRightMotorSpeed = -1 *(speed * cos ((30  * M_PI) / 180 - directionRad ));
	frontLeftMotorSpeed = -1 * (speed * cos ((150 * M_PI) / 180 - directionRad ));

	frontLeftMotorDrive(frontLeftMotorSpeed);
	frontRightMotorDrive(frontRightMotorSpeed);
	rearMotorDrive(rearMotorSpeed);
}

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
* Implementation:
* Sign of speed sets direction (negative is CW, positive is CCW)
* To rotate Clockwise (CW) all FIN should be LOW and all RIN HIGH (all motors spin in reverse)
* To rotate Counterclockwise (CCW) all FIN should be HIGH and all RIN LOW (all motors forward)
* This is set with 2 simple if statements
* All motors should spin at the same speed which is simply the input arguement speed
* This is used to update the duty cycle using PWM_CUPD
*
* Improvements:
* None as of 26/7/17
*
*/
void rotateRobot(signed char speed)
{
	speed = capToRangeInt(speed, -100, 100);	//In range check.
	frontLeftMotorDrive(speed);
	frontRightMotorDrive(speed);
	rearMotorDrive(speed);
}

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
* Implementation:
* Sets all direction pins on the motor drivers low.
*
*/
void stopRobot(void)
{
	//Stops the robot from moving
	movingFlag = 0;
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
	stopRobot();
}

/*
* Function:
* uint8_t steerRobot(uint8_t speed, int8_t turnRatio)
*
* Allows robot to turn while moving forward by a percentage of the forward speed.
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
* Implementation:
* First check that speed and turnRatio are in the correct range and fix if necessary.
* Next calculate the speed ratios for forward motion and rotational motion based on the speed and
* turn ratio passed to the function. forwardSpeed is inversely proportional to the absolute value
* of rotational speed. Finally, apply speeds to the motors.
*
*/
uint8_t steerRobot(uint8_t speed, int8_t turnRatio)
{
	//Make sure parameters are in range and correct if necessary
	speed = capToRangeUint(speed, 0, 100);
	turnRatio = capToRangeInt(turnRatio, -100, 100);
	
	//Calculate speed ratios	
	float rotationalSpeed = speed*(turnRatio/100.0);
	float straightSpeed = speed - (abs(rotationalSpeed));
	
	//Apply speeds and directions to motors
	//Only front motors provide forward drive. Rear motor is for rotation only.
	frontRightMotorDrive((int8_t)(-straightSpeed + rotationalSpeed));
	frontLeftMotorDrive((int8_t)(straightSpeed + rotationalSpeed));
	rearMotorDrive((int8_t)rotationalSpeed);
	
	return 0;
}