/*
* testFunctions.c
*
* Author : Adam Parlane (adam.parlane@outlook.com) Github: AdamParlane
* Created: 15/07/2017 3:31:20 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Test functions to communicate with the Swarm GUI for the purpose of testing that
* all robot systems are working correctly
*
*
* Contains the following functions:
* void testManager(struct message_info message)
* void convertData(struct message_info message, uint8_t* data[50])
*
* Functionality of each function is explained before each function
* This .c file should be paired with testFunctions.h
*
*/
#include "testFunctions.h"


/*
* Function: void testManager(struct message_info message)
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
* TWI Multiplexor
* TWI External Connections
* Cameras
*
* How each case works will be exaplined there
* The test MODE controls how the data is sent back
* Test Modes
* DATA RETURN	Data heading to PC (used to stop other robots from getting confused)
* SINGLE SAMPLE Send one sample of the required test back to the PC
* STREAM DATA	Continuously send data to the PC every [streaming interval]
* STOP STREAMING Stop streaming data bacl to the PC
*
*/
void testManager(struct message_info message)
{
	uint8_t receivedTestData[50];
	uint8_t transmitTestData[50];
	uint16_t peripheralReturnData;
	char testType = message.command;
	uint8_t testMode;
	uint8_t transmitTestDataSize;
	convertData(message, &receivedTestData);
	testMode = receivedTestData[1];
	struct Position testPosition;
	
	switch(testType)
	{
		case TEST_PROXIMITY_SENSORS:
		peripheralReturnData = Proximity_Data_Read(receivedTestData[0]);
		transmitTestData[0] = receivedTestData[0];
		transmitTestData[1] = DATA_RETURN; //sending data out
		transmitTestData[2] = peripheralReturnData >> 8; //upper byte
		transmitTestData[3] = peripheralReturnData & 0xFF; //lower byte
		transmitTestDataSize = 4;
		break;
		
		case TEST_LIGHT_SENSORS:
		peripheralReturnData = LightSensor_Data_Read(receivedTestData[0]);
		transmitTestData[0] = receivedTestData[0];
		transmitTestData[1] = DATA_RETURN; //sending data out
		transmitTestData[2] = peripheralReturnData >> 8; //upper byte
		transmitTestData[3] = peripheralReturnData & 0xFF; //lower byte
		transmitTestDataSize = 4;
		break;
		
		case TEST_MOTORS:
		//TO FINSH ADAM
		setTestMotors(receivedTestData);
		transmitTestData[0] = receivedTestData[0];
		transmitTestData[1] = DATA_RETURN;
		transmitTestData[2] = receivedTestData[2];
		transmitTestDataSize = 3;

		break;
		
		case TEST_MOUSE_SENSOR:
		getMouseXY(&testPosition);
		transmitTestData[0] = DATA_RETURN; //sending data out
		transmitTestData[1] = testPosition.opticaldx >> 8; //upper byte
		transmitTestData[2] = testPosition.opticaldx & 0xFF; //lower byte
		transmitTestData[3] = testPosition.opticaldy >> 8; //upper byte
		transmitTestData[4] = testPosition.opticaldy & 0xFF; //lower byte
		transmitTestDataSize = 5;
		break;
		
		case TEST_IMU:
		//TO DO Adam & Matt
		//getIMUQuaterions(&testPosition);
		transmitTestData[0] = DATA_RETURN; //sending data out
		transmitTestData[1] = testPosition.IMUqw >> 8;		//upper byte
		transmitTestData[2] = testPosition.IMUqw & 0xFF;	//lower byte
		transmitTestData[3] = testPosition.IMUqx >> 8;		//upper byte
		transmitTestData[4] = testPosition.IMUqx & 0xFF;	//lower byte
		transmitTestData[5] = testPosition.IMUqy >> 8;		//upper byte
		transmitTestData[6] = testPosition.IMUqy & 0xFF;	//lower byte
		transmitTestData[7] = testPosition.IMUqz >> 8;		//upper byte
		transmitTestData[8] = testPosition.IMUqz & 0xFF;	//lower byte
		transmitTestDataSize = 5;
		break;
		
		case TEST_LINE_FOLLOWERS:
		//TO DO Adam & Paul
		break;
		
		case TEST_FAST_CHARGE_CHIP:
		//TO DO Adam & Esmond
		break;
		
		case TEST_TWI_MULTIPLEXOR:
		TWI0_MuxSwitch(receivedTestData[1]);
		transmitTestData[0] = DATA_RETURN;
		transmitTestData[1] = TWI0_ReadMuxChannel();
		transmitTestDataSize = 2;
		break;
		
		case TEST_TWI_EXTERNAL:
		//TO DO Adam & Paul
		break;
		
		case TEST_CAMERA:
		//TO DO Adam & Brae
		break;

	}
	if(testMode == STREAM_DATA)
	{
		SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,transmitTestData,transmitTestDataSize);  //Send the Message
	}
	if(testMode == SINGLE_SAMPLE)
	{
		SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,transmitTestData,transmitTestDataSize);  //Send the Message
		testMode = STOP_STREAMING;
	}

}

/*
* Function: void convertData(struct message_info message, uint8_t* data[50])
*
* Converts the received message structure and pointer to an array with the required test command data
*
* Input is the message structure from the received data
* after the XBee framing has been stripped
* and a pointer to the array where the new data is to be copied to
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
* TWI Multiplexor
* TWI External Connections
* Cameras
*
* How each case works will be exaplined there
* The test MODE controls how the data is sent back
* Test Modes
* DATA RETURN	Data heading to PC (used to stop other robots from getting confused)
* SINGLE SAMPLE Send one sample of the required test back to the PC
* STREAM DATA	Continuously send data to the PC every [streaming interval]
* STOP STREAMING Stop streaming data bacl to the PC
*
*/
//converts Xbee data to array
void convertData(struct message_info message, uint8_t* data[50])
{
	uint8_t dataByte;
	char messageError;
	MessageBufferOut = message.index;
	for (char i = 0; i < message.length; i++)
	{
		messageError = MessageBufferGet(&dataByte);
		if(messageError == 0)
		{
			data[i] = dataByte;
		}
		else
		return;
	}
}

