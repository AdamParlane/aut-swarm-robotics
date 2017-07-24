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
* Functions:
* void motor_init(void);
* void moveRobot(float direction, unsigned char speed);
* void stopRobot(void);
* void rotateRobot(char direction, unsigned char speed);
* void dockRobot(void);
* void setTestMotors(uint8_t motorData[]);
*
*/
 
///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "motor_driver.h"

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
* Implementation:
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void motor_init(void)
{
	REG_CCFG_SYSIO |= CCFG_SYSIO_SYSIO12; //disable erase pin to give access to PB12 via PIO
	REG_PMC_PCER0 |= (1<<31);		//Enable clock access for PWM
	
	//****Channel 3 (Motor 1)****//
	REG_PWM_CMR3 |= (0x4<<0);		//Channel pre scale CLK by 16 = 24.4KHz
	REG_PWM_CMR3 |= (1<<9);			//output starts at high level
	REG_PWM_CMR3 &= ~(1<<8);		//Left aligned output
	REG_PWM_CMR3 &= ~(1<<10);		//Update Duty cycle (NOT period)
	REG_PWM_CDTY3 = 0;				//PWM Duty cycle (default = 0)
	REG_PWM_CPRD3 = 100;			//PWM Counter 0 - 100 (makes duty cycle %)
		
	REG_PIOC_PDR |= (1<<21);		//Enable peripheral control of PC21
	REG_PIOC_ABCDSR = (1<<21);		//Assign PC21 to PWM Peripheral B		
	REG_PIOB_PER |= (1<<12);		//Enable PIO control of PB12
	REG_PIOB_OER |= (1<<12);		//Set PB12 as output
	FIN_1_L;		
	REG_PIOC_PER |= (1<<22);		//Enable PIO control of PC22
	REG_PIOC_OER |= (1<<22);		//Set PC22 as output
	RIN_1_L;
								
	//****Channel 2 (Motor 2)****//
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
	RIN_2_L;		
	REG_PIOA_PER |= (1<<31);		//Enable PIO control of PA31
	REG_PIOA_OER |= (1<<31);		//Set PA31 as output
	FIN_2_L;		

	//****Channel 1 (Motor 3)****//
	REG_PWM_CMR1 |= (0x4<<0);		//Channel pre scale CLK by 16 = 24.4KHz
	REG_PWM_CMR1 |= (1<<9);			//output starts at low level
	REG_PWM_CMR1 &= ~(1<<8);		//Left aligned output
	REG_PWM_CMR1 &= ~(1<<10);		//Update Duty cycle (NOT period)
	REG_PWM_CDTY1 = 0;				//PWM Duty cycle (default = 0)
	REG_PWM_CPRD1 = 100;			//PWM Counter 0 - 100 (makes duty cycle %)
		
	REG_PIOC_PDR |= (1<<9);			//Enable peripheral control of PC9
	REG_PIOC_ABCDSR = (1<<9);		//Assign PC9 to PWM Peripheral B		
	REG_PIOA_PER |= (1<<30);		//Enable PIO control of PA30
	REG_PIOA_OER |= (1<<30);		//Set PA30 as output
	RIN_3_L;		
	REG_PIOA_PER |= (1<<29);		//Enable PIO control of PA29
	REG_PIOA_OER |= (1<<29);		//Set PA29 as output
	FIN_3_L;
	
	//****Enable PWM Channels as last step of setup****//	
	REG_PWM_ENA |= PWM_ENA_CHID1;	//Enable PWM on channel 1
	REG_PWM_ENA |= PWM_ENA_CHID2;	//Enable PWM on channel 2
	REG_PWM_ENA |= PWM_ENA_CHID3;	//Enable PWM on channel 3
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
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void moveRobot(float direction, unsigned char speed)
{
	float motor1Speed, motor2Speed, motor3Speed;
	float directionRad;
	//keep direction in range +/-180degrees
	while(direction > 180) 
		direction -= 360;
	while(direction < -180)
		direction += 360;
	//stop speed from being over max in case of user input error
	if(speed > 100)
		speed = 100;
	directionRad = (direction * M_PI) / 180; //convert desired direction to radians
	motor1Speed = speed * cos ((270 * M_PI) / 180 - directionRad );//radians
	motor2Speed = speed * cos ((30  * M_PI) / 180 - directionRad );
	motor3Speed = speed * cos ((150 * M_PI) / 180 - directionRad );
	
	//motor 2 & 3 is wired backwards on test robot so forward and back is flipped
	if(motor1Speed > 0)
	{
		//Forward
		RIN_1_L;
		FIN_1_H;		
	}
	else if (motor1Speed == 0)
	{
		//Motor off (coast not brake)
		RIN_1_L;
		FIN_1_L;
	}
	else
	{
		//Reverse
		motor1Speed = motor1Speed * (-1); 
		RIN_1_H;
		FIN_1_L;
	}

	if(motor2Speed > 0)
	{
		//Forward
		RIN_2_L;
		FIN_2_H;
	}
	else if (motor2Speed == 0)
	{
		//Motor off (coast not brake)
		RIN_2_L;
		FIN_2_L;
	}
	else
	{
		//Reverse
		motor2Speed = motor2Speed * (-1); 
		RIN_2_H;
		FIN_2_L;
	}
	
	if(motor3Speed > 0)
	{
		//Forward
		RIN_3_L;
		FIN_3_H;			
	}
	else if (motor3Speed == 0)
	{
		//Motor off (coast not brake)
		RIN_3_L;
		FIN_3_L;
	}
	else
	{
		//Reverse
		motor3Speed = motor3Speed * (-1); 
		RIN_3_H;
		FIN_3_L;
	}
	REG_PWM_CUPD1 = (motor3Speed); //Update duty cycle as per calculations
	REG_PWM_CUPD2 = (motor2Speed); //Update duty cycle as per calculations
	REG_PWM_CUPD3 = (motor1Speed); //Update duty cycle as per calculations
}

/*
* Function:
* void rotateRobot(char direction, unsigned char speed)
*
* Will rotate the robot on the spot in the given direcion and relative speed.
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
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void rotateRobot(char direction, unsigned char speed)
{
	if(direction == CW)			//enable all motors to spin the robot clockwise
	{
		RIN_1_H;
		FIN_1_L;
		RIN_2_H;
		FIN_2_L;
		RIN_3_H;
		FIN_3_L;
	}
	else if(direction == CCW)	//enable all motors to spin the robot counter-clockwise
	{
		RIN_1_L;
		FIN_1_H;
		RIN_2_L;
		FIN_2_H;
		RIN_3_L;
		FIN_3_H;
	}
	//Update all duty cycles to match the desired rotation speed
	REG_PWM_CUPD1 = speed;
	REG_PWM_CUPD2 = speed;
	REG_PWM_CUPD3 = speed;	
}

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
void dockRobot(void)
{
	//************Approx light ranges**************//
	//Ambient light in WS217 0x00DE - 0x0158
	//LED @ 10cm straight on 0x0abe - 0x12ea
	//LED @ 20cm straight on 0x05da - 0x06c6
	//LED @ 30cm straight on 0x0443 - 0x04a9
	//LED @ 30cm 30dg offset High ~0x04a9 Low ~0x0440 
	//LED @ 30cm 60dg offset High ~0x033a Low ~0x02b7 
	//LED @ 30cm 90dg offset High ~0x00ec Low ~0x00d6
	
	//***********Approx Promity Values*************//
	//Using my hand as an object, testing on side A
	//touching - ~5cm = 0x03ff (max)
	//5 cm away 0x0150 - 0x01ff
	//10cm away 0x0070 - 0x0100
	//20cm away 0x0030 - 0x003f
	//30cm away 0x0020 - 0x0029
	
	uint16_t rightBrightness, leftBrightness;
	float diff = 0;
	float rightBrightnessScaled, leftBrightnessScaled;
	
	leftBrightness = lightSensRead(MUX_LIGHTSENS_L, LS_WHITE_REG);
	rightBrightness = lightSensRead(MUX_LIGHTSENS_R, LS_WHITE_REG);
	
	//frontProximity = proxSensRead(MUX_PROXSENS_A); //need to test this
	
	if(rightBrightness > 0x0200 && leftBrightness > 0x0200)//if there is more light than ambient
	{
		//Scale brightness to calculate required position
		rightBrightnessScaled = (rightBrightness / 0xFFFF) * 100;
		leftBrightnessScaled = (leftBrightness / 0xFFFF) * 100;
		//Zero Justified Normalized Differential Shade Calculation
		diff = 2 * (((rightBrightnessScaled * 100)/(rightBrightnessScaled + leftBrightnessScaled)) - 50);
		//Convert to degrees
		moveRobot(diff/2, 50);
	}
	else if((leftBrightness > 0x1000 || rightBrightness >  0x1000))// && frontProximity > 0x0300)
	{
		stopRobot();
	}
	else if((rightBrightness - leftBrightness) > 0x009F)
	{
		rotateRobot(CW, 30); //turn right
	}
	else if((leftBrightness - rightBrightness) > 0x009F)
	{
		rotateRobot(CCW, 30);//turn left
	}
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
	FIN_1_L;
	RIN_1_L;
	FIN_2_L;
	RIN_2_L;
	FIN_3_L;
	RIN_3_L;
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
* [explain key steps of function]
* [use heavy detail for anything complicated]
*
* Improvements:
* Maybe some of the PWM registers could be given more read-friendly names?
*
*/
void setTestMotors(uint8_t motorData[])
{
	if(motorData[0] == MOTOR_1 && (motorData[1] & 0x80))//check if bit 7 is set meaning forward
	{
		FIN_1_H;
		RIN_1_L;
		REG_PWM_CUPD1 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_1 && ~(motorData[1] & 0x80))
	{
		FIN_1_L;
		RIN_1_H;
		REG_PWM_CUPD1 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_2 && (motorData[1] & 0x80))//check if bit 7 is set meaning forward
	{
		FIN_2_H;
		RIN_2_L;
		REG_PWM_CUPD2 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_2 && ~(motorData[1] & 0x80))
	{
		FIN_2_L;
		RIN_2_H;
		REG_PWM_CUPD2 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_3 && (motorData[1] & 0x80))//check if bit 7 is set meaning forward
	{
		FIN_3_H;
		RIN_3_L;
		REG_PWM_CUPD3 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_1 && ~(motorData[1] & 0x80))
	{
		FIN_3_L;
		RIN_3_H;
		REG_PWM_CUPD3 = (motorData[1] & 0x7F);
	}
}