/*
 * motor_driver.h
 *	Moves the robot in a desired direction by calculating PWM for motors
 *  and powering them as required to ensure direction is correct
 *  used by calling the moveRobot(direction, speed)
 *  direction is from  0-360 where 0 is the start heading of the robot
 *  and the positive direction is counter clockwise
 *	Will work in the way that other parts of the robots software can call the function
 *  moveRobot(signed int direction, unsigned char speed)
 *  this will move the robot as described until told to move otherwise
 *  eventually functionality will be added to stop at a given location and facing a certain heading but
 *  for this to be implemented I will focus on getting the navigation going
 *
 * Author : Adam Parlane
 *
 * Created: 13/05/2017 4:18:25 PM
 * 
 */ 

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#include <fastmath.h>
#include "robotdefines.h"

#define CW	0
#define CCW 1

//****Define Motor Pins for readability of code****//
//Motor 1
#define	FIN_1_Low	(REG_PIOB_CODR |= (1<<12))
#define	FIN_1_High	(REG_PIOB_SODR |= (1<<12))
#define	RIN_1_Low	(REG_PIOC_CODR |= (1<<22))
#define	RIN_1_High	(REG_PIOC_SODR |= (1<<22))
//Motor 2
#define	RIN_2_Low	(REG_PIOA_CODR |= (1<<31))
#define	RIN_2_High	(REG_PIOA_SODR |= (1<<31))
#define	FIN_2_Low	(REG_PIOC_CODR |= (1<<19))
#define	FIN_2_High	(REG_PIOC_SODR |= (1<<19))
//Motor 3
#define	RIN_3_Low	(REG_PIOA_CODR |= (1<<29))
#define	RIN_3_High	(REG_PIOA_SODR |= (1<<29))
#define	FIN_3_Low	(REG_PIOA_CODR |= (1<<30))
#define	FIN_3_High	(REG_PIOA_SODR |= (1<<30))

//motor setup function PIO and PWM
void motor_init(void);

//Motor Control Functions
void moveRobot(float direction, unsigned char speed);
void stopRobot(void);
void rotateRobot(char direction, unsigned char speed);
void dockRobot(void);

#endif /* MOTOR_DRIVER_H_ */