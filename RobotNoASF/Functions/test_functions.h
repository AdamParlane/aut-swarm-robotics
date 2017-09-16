/*
 * test_functions.h
 *
 * Created: 13/07/2017 4:55:41 PM
 *  Author: TODO:COMMECNTING AND HEADERSadams
 */ 

#ifndef TESTFUNCTIONS_H_
#define TESTFUNCTIONS_H_

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

#define TEST_ALL				0xEE
#define TEST_ALL_RETURN			0xEF

#define DATA_RETURN				0x00
#define SINGLE_SAMPLE			0x01
#define STREAM_DATA				0x02
#define STOP_STREAMING			0xFF

#define REAR_MOTOR				0x01
#define F_RIGHT_MOTOR			0x02
#define F_LEFT_MOTOR			0x03

struct transmitDataStructure
{
	uint8_t Data[50];//array for data to be transmitted to PC BEFORE XBee framing has been added
	uint8_t DataSize;//size of the transmit array
};

union float_bytes {
	float val;
	unsigned char bytes[sizeof(float)];
}data;

struct MessageInfo;	//Defined in xbee_driver.h

/*
* Function: uint8_t testManager(struct transmitDataStructure *transmit,	RobotGlobalStructure *sys)
*
* Handles the interpretation of received test commands,
* calling the appropriate test functions / performing tests
* and returning to the PC the test return value
*
* Input is the message structure from the received data
* after the XBee framing has been stripped
*
* No Return Values
*
* Called in the main loop whenever a new command is received
* OR in the case of streaming data it will be called at every streaming interval
* ***Streaming Interval = 100ms***
*
*/
uint8_t getTestData(struct transmitDataStructure *transmit, RobotGlobalStructure *sys);

void testManager(RobotGlobalStructure *sys);



/*
* Function: void testAll(struct transmitDataStructure *transmit, RobotGlobalStructure *sys)
*
* tests all the peripherals in a set order and returns them all back to the GUI in one packet
* calling the appropriate test functions / performing tests
* and returning to the PC the test return values
*
* Input is the transmit array
*
* No Return Values
*
*/
void testAll(struct transmitDataStructure *transmit, RobotGlobalStructure *sys);








#endif /* TESTFUNCTIONS_H_ */