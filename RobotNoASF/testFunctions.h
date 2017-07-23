/*
 * testFunctions.h
 *
 * Created: 13/07/2017 4:55:41 PM
 *  Author: adams
 */ 


#ifndef TESTFUNCTIONS_H_
#define TESTFUNCTIONS_H_

#include "robot_defines.h"

#define TEST_COMMUNICATIONS		0xE1
#define TEST_PROXIMITY_SENSORS	0xE4
#define TEST_LIGHT_SENSORS		0xE5
#define TEST_MOTORS				0xE6
#define TEST_MOUSE_SENSOR		0xE7
#define TEST_IMU				0xE8
#define TEST_LINE_FOLLOWERS		0xE9
#define TEST_FAST_CHARGE_CHIP	0xEA
#define TEST_TWI_MULTIPLEXOR	0xEB
#define TEST_TWI_EXTERNAL		0xEC
#define TEST_CAMERA				0xED

#define DATA_RETURN				0x00
#define SINGLE_SAMPLE			0x01
#define STREAM_DATA				0x02
#define STOP_STREAMING			0xFF

#define MOTOR_1					0x01
#define MOTOR_2					0x02
#define MOTOR_3					0x03

//Function Prototypes
void setTestMotors(uint8_t motorData[]);
void convertData(struct message_info message, uint8_t *data);
void testManager(struct message_info message);

#endif /* TESTFUNCTIONS_H_ */