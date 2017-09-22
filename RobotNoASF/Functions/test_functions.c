/*
* test_functions.c
*
* Author : Adam Parlane (adam.parlane@outlook.com) Github: AdamParlane
* Created: 15/07/2017 3:31:20 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Test functions to communicate with the Swarm GUI for the purpose of testing that
* all robot systems are working correctly
*
* Contains the following functions:
* uint8_t testManager(struct MessageInfo xbMessage, struct transmitDataStructure *transmit,
* RobotGlobalStructure *sys)
* void comConvertData(struct MessageInfo xbMessage, uint8_t* data[50])
*
* Functionality of each function is explained before each function
* This .c file should be paired with test_functions.h
*
*/
//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "../Interfaces/xbee_driver.h"
#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/light_sens_interface.h"
#include "../Interfaces/adc_interface.h"
#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/motor_driver.h"
#include "../Interfaces/opt_interface.h"
#include "test_functions.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: uint8_t testManager(struct MessageInfo xbMessage, struct transmitDataStructure *transmit,
*			RobotGlobalStructure *sys)
*
* Handles the interpretation of received test commands,
* calling the appropriate test functions / performing tests
* and returning to the PC the test return value
*
* Inputs:	the message structure from the received data after the XBee framing has been stripped
*			pointer to the transmitData structure (contains transmitArray and size char)
*			pointer to the sys->pos. structure for mouse and IMU data
*
* No Return Values
*
* Called in the main loop whenever a new command is received
* OR in the case of streaming data it will be called at every streaming interval
* ***Streaming Interval = 100ms***
*
* Implementation:
* Uses 2 arrays to keep track of interpreted data for
* (1) receiving and (2) transmitting test data
* Test Type variable controls the state machine and is the type of test being
* carried out (eg test mouse sensor)
* There is a case for each test type
*
* TEST TYPES
* Proximity Sensors
* Light Sensors
* Motors
* Mouse Sensor
* IMU
* Line Followers
* Fast Charge Chip
* TWI Multiplexer
* TWI External Connections
* Cameras
*
* How each case works will be explained there
* The test MODE controls how the data is sent back
* Test Modes
* DATA RETURN	Data heading to PC (used to stop other robots from getting confused)
* STREAM DATA	Continuously send data to the PC every [streaming interval]
* STOP STREAMING Stop streaming data back to the PC
* ***SINGLE SAMPLE Possibly getting removed(18/7) - AP
* SINGLE SAMPLE Send one sample of the required test back to the PC
*
* Transmit Arrays are framed in each case and are as follows
* [0] - test type (proximity, light sensor etc)
* [1] - (if applicable) specific peripheral (eg Prox A, B etc)
*		where there is only 1 peripheral of a given type [1] will be removed and everything
*		else will move up [2] will become [1] and so on
* [2] - The test mode, in terms of the robot returning data to the PC or another robot
*		(all of the below cases)
*		This will ALWAYS be DATA_RETURN (0x00)
*		This is to ensure that a robot receiving the message doesnt mistake the command for a REQUEST
*		For data from that robot but rather the robot is to RECEIVE the data
* [3-N]-The Data to be returned to the PC / robot
*		In the case of data with a size larger than bytes this is broken into byte sized pieces
*		The order will be [3] Data1_High, [4] Data1_Low, [5] Data2_High and so on
* The transmit array sie must also be calculated and sent with the XBee Transmit Request
*/
uint8_t getTestData(struct transmitDataStructure *transmit, RobotGlobalStructure *sys)
{
	static uint8_t receivedTestData[50]; //array for data coming into the robot AFTER Xbee framing has been stripped
	uint16_t peripheralReturnData; //the test data returned from eh relevant peripheral
	char testType = sys->comms.messageData.command;//what peripheral is being tested
	if (sys->flags.xbeeNewData)//only reconvert the data if we are in this function for new data not streaming again
	{
		xbeeCopyData(sys->comms.messageData, receivedTestData);//converts the Xbee data to the receivedTestData array
		sys->flags.xbeeNewData = 0;
	}
	uint8_t testMode = receivedTestData[0];
	transmit->Data[0] = testType; //First return value is the testType so the PC knows what it is receiving
	switch(testType)
	{
		case TEST_COMMUNICATIONS:
			transmit->Data[1] = 0x01;
			transmit->DataSize = 2;
			break;
		
		case TEST_PROXIMITY_SENSORS: //0xE4
			//6 Proximtiy Sensors (A-F) Identified by their Mux channels
			peripheralReturnData = proxSensRead(receivedTestData[1]);
			transmit->Data[1] = DATA_RETURN; //sending data out
			transmit->Data[2] = receivedTestData[1];//Transmit the specific proximity sensor ID
			transmit->Data[3] = peripheralReturnData >> 8; //upper data byte
			transmit->Data[4] = peripheralReturnData & 0xFF; //lower data byte
			transmit->DataSize = 5;
			break;
		
		case TEST_LIGHT_SENSORS:
			//2 Light Sensors (LHS & RHS) Identified by their Mux channels
			peripheralReturnData = lightSensRead(receivedTestData[1], LS_WHITE_REG);
			transmit->Data[1] = DATA_RETURN; //sending data out
			transmit->Data[2] = receivedTestData[1];//Transmit the specific light sensor ID
			transmit->Data[3] = peripheralReturnData >> 8; //upper byte
			transmit->Data[4] = peripheralReturnData & 0xFF; //lower byte
			transmit->DataSize = 5;
			break;

		
		case TEST_MOUSE_SENSOR:
			//Only 1 mouse sensor just trying to attain dx & dy
			transmit->Data[1] = DATA_RETURN; //sending data out
			transmit->Data[2] = sys->pos.Optical.x >> 8; //upper byte
			transmit->Data[3] = sys->pos.Optical.x & 0xFF; //lower byte
			transmit->Data[4] = sys->pos.Optical.y >> 8; //upper byte
			transmit->Data[5] = sys->pos.Optical.y & 0xFF; //lower byte
			transmit->DataSize = 6;
			break;
		
		case TEST_IMU:
			//Pitch, Roll, Yaw all floats
			transmit->Data[1] = DATA_RETURN; //sending data out
			data.val = sys->pos.IMU.pitch;
			transmit->Data[2] = data.bytes[3];		//upper byte
			transmit->Data[3] = data.bytes[2];	//upper middle byte
			transmit->Data[4] = data.bytes[1];		//lower middle byte
			transmit->Data[5] = data.bytes[0];	//lower byte
			data.val = sys->pos.IMU.roll;
			transmit->Data[6] = data.bytes[3];			//upper byte
			transmit->Data[7] = data.bytes[2];			//upper middle byte
			transmit->Data[8] = data.bytes[1];			//lower middle byte
			transmit->Data[9] = data.bytes[0];			//lower byte
			data.val = sys->pos.IMU.yaw;
			transmit->Data[10] = data.bytes[3];			//upper byte
			transmit->Data[11] = data.bytes[2];			//upper middle byte
			transmit->Data[12] = data.bytes[1];			//lower middle byte
			transmit->Data[13] = data.bytes[0];			//lower byte
			transmit->DataSize = 14;
			break;
		
		case TEST_LINE_FOLLOWERS:
			peripheralReturnData = adcRead(receivedTestData[1]);
			transmit->Data[1] = DATA_RETURN; //sending data out
			transmit->Data[2] = receivedTestData[1];//Transmit the specific proximity sensor ID
			transmit->Data[3] = peripheralReturnData >> 8; //upper data byte
			transmit->Data[4] = peripheralReturnData & 0xFF; //lower data byte
			transmit->DataSize = 5;
			break;
		
		case TEST_FAST_CHARGE_CHIP:
			peripheralReturnData = adcBatteryVoltage();
			transmit->Data[1] = DATA_RETURN; //sending data out
			transmit->Data[2] = peripheralReturnData >> 8; //upper byte
			transmit->Data[3] = peripheralReturnData & 0xFF; //lower byte
			transmit->DataSize = 4;
			break;
		
		case TEST_TWI_MULTIPLEXOR:
			//To test the Mux the channel is changed to one set by the PC
			//Then the channel is read off the Mux it should match what was instructed
			//Matching will be checked on the PC side, will appear as an echo if test passes
			twi0MuxSwitch(receivedTestData[1]);//Set the Mux to the specified channel
			transmit->Data[1] = DATA_RETURN;//sending data out
			transmit->Data[2] = twi0ReadMuxChannel();//Return the channel the Mux is currently set to
			transmit->DataSize = 3;
			break;

		case TEST_TWI_EXTERNAL:
			//TO DO Adam & Paul
			break;
		
		case TEST_CAMERA:
			//TO DO Adam & Brae
			break;
		
		case TEST_MOTORS:
			//3 Motors (1:0x01, 2:0x02, 3:0x03)
			//The motors need to be turned on individually at a set direction and speed as commanded by the PC
			//This is done with a different setTestMotors function, found in motorDriver.c
			setTestMotors(receivedTestData); //Turn on the require motor at the set speed and direction
			transmit->Data[1] = DATA_RETURN; //Sending Data Out
			transmit->Data[2] = receivedTestData[1];//Transmit the specific motor ID
			transmit->Data[3] = receivedTestData[2];//Echo's the command
			//TODO: instead of echo read what motor is on with direction and speed and return it
			testMode = SINGLE_SAMPLE;
			transmit->DataSize = 4;
			break;

	}
	return testMode;
}

void testManager(RobotGlobalStructure *sys)
{
	static struct transmitDataStructure transmitMessage;
	static uint8_t testMode = 0x00;
	static uint32_t nextSendTime = 0;	//Time at which next packet will be streamed
	
	if(sys->flags.xbeeNewData)
	{
		//get the new test data
		testMode = getTestData(&transmitMessage, sys);

		switch(testMode)
		{
			case STOP_STREAMING:
				sys->states.mainf = M_IDLE;
				break;
				
			case SINGLE_SAMPLE:
				sys->states.mainf = M_IDLE;
				xbeeSendAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, transmitMessage.Data,
				transmitMessage.DataSize);  //Send the Message
				break;

			case STREAM_DATA:
				if(sys->timeStamp >= nextSendTime)	//If the minimum time interval has elapsed
				{
					//Set the next time to stream a packet
					nextSendTime = sys->timeStamp + sys->comms.testModeStreamInterval;
					//Send the Message
					xbeeSendAPITransmitRequest(COORDINATOR_64,UNKNOWN_16, transmitMessage.Data,
												transmitMessage.DataSize);
				}
				break;
		}		
	}
}

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
void testAll(struct transmitDataStructure *transmit, RobotGlobalStructure *sys)
{
	//tests all the peripherals in a set order and returns them all back to the GUI in one packet
	//Can only be used once comms test is performed
	//[WIP] needs consultation with Mansel -AP
	//Order will be
	//Position testPosition;
	uint16_t doubleByteData; //used as an intermediate
	transmit->Data[0] = TEST_ALL_RETURN; //0xEF
	doubleByteData = proxSensRead(MUX_PROXSENS_A);
	transmit->Data[1] = doubleByteData >> 8;
	transmit->Data[2] = doubleByteData & 0xFF;
	doubleByteData = proxSensRead(MUX_PROXSENS_B);
	transmit->Data[3] = doubleByteData >> 8;
	transmit->Data[4] = doubleByteData & 0xFF;
	doubleByteData = proxSensRead(MUX_PROXSENS_C);
	transmit->Data[5] = doubleByteData >> 8;
	transmit->Data[6] = doubleByteData & 0xFF;
	doubleByteData = proxSensRead(MUX_PROXSENS_D);
	transmit->Data[7] = doubleByteData >> 8;
	transmit->Data[8] = doubleByteData & 0xFF;
	doubleByteData = proxSensRead(MUX_PROXSENS_E);
	transmit->Data[9] = doubleByteData >> 8;
	transmit->Data[10] = doubleByteData & 0xFF;
	doubleByteData = proxSensRead(MUX_PROXSENS_F);
	transmit->Data[11] = doubleByteData >> 8;
	transmit->Data[12] = doubleByteData & 0xFF;
	doubleByteData = lightSensRead(MUX_LIGHTSENS_L, LS_WHITE_REG);
	transmit->Data[13] = doubleByteData >> 8;
	transmit->Data[14] = doubleByteData & 0xFF;
	doubleByteData = lightSensRead(MUX_LIGHTSENS_R, LS_WHITE_REG);
	transmit->Data[15] = doubleByteData >> 8;
	transmit->Data[16] = doubleByteData & 0xFF;
	getMouseXY(sys);
	//transmit->Data[17] = testPosition.Optical.dx >> 8;	//upper byte
	//transmit->Data[18] = testPosition.Optical.dx & 0xFF; //lower byte
	//transmit->Data[19] = testPosition.Optical.dx >> 8;	//upper byte
	//transmit->Data[20] = testPosition.Optical.dx & 0xFF; //lower byte
	//[WIP] Need to check if our comms can handle this before finishing it
	//If it goes well I might abstract the tests to functions that fill the array also
	//TODO: Adam Parlane
}