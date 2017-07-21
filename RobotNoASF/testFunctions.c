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
* [2] - The test mode, in terms of the robot returning data to the PC or another robot (all of the below cases)
*		This will ALWAYS be DATA_RETURN (0x00)
*		This is to ensure that a robot receiving the message doesnt mistake the command for a REQUEST
*		For data from that robot but rather the robot is to RECEIVE the data
* [3-N]-The Data to be returned to the PC / robot
*		In the case of data with a size larger than bytes this is broken into byte sized pieces
*		The order will be [3] Data1_High, [4] Data1_Low, [5] Data2_High and so on
* The transmit array sie must also be calculated and sent with the XBee Transmit Request
*/
void testManager(struct message_info message)
{
	struct Position testPosition;
	static uint8_t receivedTestData[50]; //array for data coming into the robot AFTER Xbee framing has been stripped
	uint8_t transmitTestData[50]; //array for data to be transmitted to PC BEFORE XBee framing has been added
	uint16_t peripheralReturnData; //the test data returned from eh relevant peripheral
	char testType = message.command;//what peripheral is being tested
	uint8_t testMode;
	uint8_t transmitTestDataSize;//size of the transmit array
	if (newDataFlag)
	{
		convertData(message, receivedTestData);//converts the Xbee data to the receivedTestData array
		newDataFlag = 0;
	}
	testMode = receivedTestData[1];
	//Declare a new instance of the Position structure for test purposes only for mouse and IMU results
	transmitTestData[0] = testType; //First return value is the testType so the PC knows what it is receiving
	switch(testType)
	{
		case TEST_PROXIMITY_SENSORS: //0xE4
		//6 Proximtiy Sensors (A-F) Identified by their Mux channels
		peripheralReturnData = Proximity_Data_Read(receivedTestData[0]);
		transmitTestData[1] = receivedTestData[0];//Transmit the specific proximity sensor ID
		transmitTestData[2] = DATA_RETURN; //sending data out
		transmitTestData[3] = peripheralReturnData >> 8; //upper data byte
		transmitTestData[4] = peripheralReturnData & 0xFF; //lower data byte
		transmitTestDataSize = 5;
		break;
		
		case TEST_LIGHT_SENSORS:
		//2 Light Sensors (LHS & RHS) Identified by their Mux channels
		peripheralReturnData = lightSensRead(receivedTestData[0]);
		transmitTestData[1] = receivedTestData[0];//Transmit the specific light sensor ID
		transmitTestData[2] = DATA_RETURN; //sending data out
		transmitTestData[3] = peripheralReturnData >> 8; //upper byte
		transmitTestData[4] = peripheralReturnData & 0xFF; //lower byte
		transmitTestDataSize = 5;
		break;
		
		case TEST_MOTORS:
		//3 Motors (1:0x01, 2:0x02, 3:0x03)
		//The motors need to be turned on individually at a set direction and speed as commanded by the PC
		//This is done with a different setTestMotors function, found in motorDriver.c
		setTestMotors(receivedTestData); //Turn on the require motor at the set speed and direction
		transmitTestData[1] = receivedTestData[0];//Transmit the specific motor ID
		transmitTestData[2] = DATA_RETURN; //Sending Data Out
		transmitTestData[3] = receivedTestData[2];//Echo's the command
		//TO DO: instead of echo read what motor is on with direction and speed and return it
		//TO DO: turn off motors after test is finished 
		transmitTestDataSize = 4;

		break;
		
		case TEST_MOUSE_SENSOR:
		//Only 1 mouse sensor just trying to attain dx & dy
		//getMouseXY will will the structure testPosition (using pointers) with dx and dy
		getMouseXY(&testPosition);
		transmitTestData[1] = DATA_RETURN; //sending data out
		transmitTestData[2] = testPosition.opticaldx >> 8; //upper byte
		transmitTestData[3] = testPosition.opticaldx & 0xFF; //lower byte
		transmitTestData[4] = testPosition.opticaldy >> 8; //upper byte
		transmitTestData[5] = testPosition.opticaldy & 0xFF; //lower byte
		transmitTestDataSize = 6;
		break;
		
		case TEST_IMU:
		//TO DO Adam & Matt
		//getIMUQuaterions(&testPosition);
		/*transmitTestData[1] = DATA_RETURN; //sending data out
		transmitTestData[2] = testPosition.IMUqw >> 8;		//upper byte
		transmitTestData[3] = testPosition.IMUqw & 0xFF;	//lower byte
		transmitTestData[4] = testPosition.IMUqx >> 8;		//upper byte
		transmitTestData[5] = testPosition.IMUqx & 0xFF;	//lower byte
		transmitTestData[6] = testPosition.IMUqy >> 8;		//upper byte
		transmitTestData[7] = testPosition.IMUqy & 0xFF;	//lower byte
		transmitTestData[8] = testPosition.IMUqz >> 8;		//upper byte
		transmitTestData[9] = testPosition.IMUqz & 0xFF;	//lower byte*/
		transmitTestDataSize = 10;
		break;
		
		case TEST_LINE_FOLLOWERS:
		//TO DO Adam & Paul
		break;
		
		case TEST_FAST_CHARGE_CHIP:
		//TO DO Adam & Esmond
		break;
		
		case TEST_TWI_MULTIPLEXOR:
		//To test the Mux the channel is changed to one set by the PC
		//Then the channel is read off the Mux it should match what was instructed
		//Matching will be checked on the PC side, will appear as an echo if test passes		
		twi0MuxSwitch(receivedTestData[1]);//Set the Mux to the specified channel
		transmitTestData[1] = DATA_RETURN;//sending data out
		transmitTestData[2] = twi0ReadMuxChannel();//Return the channel the Mux is currently set to
		transmitTestDataSize = 3;
		break;
		
		case TEST_TWI_EXTERNAL:
		//TO DO Adam & Paul
		break;
		
		case TEST_CAMERA:
		//TO DO Adam & Brae
		break;

	}
	SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,transmitTestData,transmitTestDataSize);  //Send the Message
}

/*
*
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
* Uses the message index to call the MessageBufferOut
* This ensures that the message copying begins from the correct location
* Simple for loop to copy the array of length message.length
* This array is accessed via pointers and is used by the test Manager
* The Message Buffer Get function returns 0 for success and -1 for failure
* This function will quit on a failed return
*
*/
void convertData(struct message_info message, uint8_t *data)
{
	char dataByte;
	char messageError;
	MessageBufferOut = message.index;//sets message buffer reader to correct start address
	for (char i = 0; i < message.length; i++)//for each entry in the array
	{
		messageError = MessageBufferGet(&dataByte);//retrieve the next byte of received message data
		if(messageError == 0)//if there was NO error
		{
			data[i] = dataByte;//fill the array with the data
		}
		else//if there was an error, exit
		//TO DO: prehaps add some sort of error flagging system??
			return;
	}
}

