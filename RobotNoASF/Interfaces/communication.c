/*
 * communication.c
 *
 *  Author: Mansel Jeffares
 * 	First Build: 10 May 2017
 *	Current Build:  11 July 2017
 *  
 *	Description :
 *		Basic Communication to computer GUI using wireless XBee Module in API Mode
 *
 *	Improvements:
 *		
 */

#include <stdbool.h>
#include <string.h>
#include "sam.h"
#include "communication.h"
#include "../robot_setup.h"

// XBee flow control bytes
#define FRAME_DELIMITER 0x7E
#define ESCAPE_BYTE 0x7D
#define XON 0x11
#define XOFF 0x13

// XBee API Frames
#define AT_COMMAND 0x08
#define AT_COMMAND_QUEUE 0x09
#define ZIGBEE_TRANSMIT_REQUEST 0x10
#define EXPLICIT_ADDRESSING_ZIGBEE_COMMAND_FRAME 0x11
#define REMOTE_COMMAND_REQUEST 0x17
#define CREATE_SOURCE_ROUTE 0x21
#define AT_COMMAND_RESPONSE 0x88
#define MODEM_STATUS 0x8A
#define ZIGBEE_TRANSMIT_STATUS 0x8B
#define ZIGBEE_RECEIVE_PACKET 0x90
#define ZIGBEE_EXPLICIT_RX_INDICATOR 0x91
#define ZIGBEE_IO_DATA_SAMPLE_RX_INDICATOR 0x92
#define XBEE_SENSOR_READ_INDICATOR 0x94
#define NODE_IDENTIFICATION_INDICATOR 0x95
#define REMOTE_COMMAND_RESPONSE 0x97
#define EXTENDED_MODEM_STATUS 0x98
#define OTA_FIRMWARE_UPDATE_STATUS 0xA0
#define ROUTE_RECORD_INDICATOR 0xA1
#define MANY_TO_ONE_ROUTE_REQUEST_INDICATOR 0xA3

// Receive States
#define CASE_START 0
#define CASE_LENGTH_MSB 1
#define CASE_LENGTH_LSB 2
#define CASE_FRAME_TYPE 3
#define CASE_DATA 4
#define CASE_CHECKSUM 5

// Buffer defines
// Due to simplistic implementation of FIFO a buffer the array must be one element larger than the number of elements wanted
#define FRAME_BUFFER_ELEMENTS 1000
#define FRAME_BUFFER_SIZE (FRAME_BUFFER_ELEMENTS + 1)

#define FRAME_BUFFER_INFO_ELEMENTS 50
#define FRAME_BUFFER_INFO_SIZE (FRAME_BUFFER_INFO_ELEMENTS + 1)

#define MESSAGE_BUFFER_ELEMENTS 500
#define MESSAGE_BUFFER_SIZE (MESSAGE_BUFFER_ELEMENTS + 1)

#define MESSAGE_BUFFER_INFO_ELEMENTS 50
#define MESSAGE_BUFFER_INFO_SIZE (MESSAGE_BUFFER_INFO_ELEMENTS + 1)

// Buffer variables
//In is the next free location an element can be put into
//Out is the next element to be read out
uint8_t FrameBuffer[FRAME_BUFFER_SIZE];
int FrameBufferIn, FrameBufferOut, FrameBufferUse;

struct frame_info FrameBufferInfo[FRAME_BUFFER_INFO_SIZE];
int FrameBufferInfoIn, FrameBufferInfoOut, FrameBufferInfoUse;

uint8_t MessageBuffer[MESSAGE_BUFFER_SIZE];
//int MessageBufferIn, MessageBufferOut, MessageBufferUse;

struct message_info MessageBufferInfo[MESSAGE_BUFFER_INFO_SIZE];
int MessageBufferInfoIn, MessageBufferInfoOut, MessageBufferInfoUse;

// Local Function Prototypes
// TODO: This needs fixing. All function prototypes should be in the header file for the module.
// If functions are to be private to a module, then they should be declared as static. Also, 
// MessageBufferGet is being called from testFunctions.c and is generating a warning. -Matt
void FrameBufferInit(void);			//Initialize all usage variables to the beginning of the array
int FrameBufferPut(uint8_t new);	//Adds a new byte to the end of the array
int FrameBufferGet(uint8_t *old);	//Gets the oldest byte from the array

void FrameBufferInfoInit(void);							//Initialize all usage variables to the beginning of the array
int FrameBufferInfoPut(int ind, uint8_t typ, int len);	//Adds a element to the end of the array

void MessageBufferInit(void);		//Initialize all usage variables to the beginning of the array
int MessageBufferPut(uint8_t new);	//Adds a new byte to the end of the array
int MessageBufferGet(char *old);	//Gets the oldest byte from the array

void MessageBufferInfoInit(void);							//Initialize all usage variables to the beginning of the array
int MessageBufferInfoPut(int ind, uint8_t cmd, int len);	//Adds a element to the end of the array

void UART3_Handler(void);									//UART3 Interrupt handler, receives XBee Frames and adds them to the buffer
void UART3_Write(uint8_t data);								//Writes a byte to UART3
void SendXbeeAPIFrame(uint8_t * frame_data, int len);		//Sends an XBee API Frame

char obstacleAvoidanceEnabledFlag = 0;


//Improvements: is it worth defining all these codes, im thinking no
void InterpretSwarmMessage(struct message_info message)
{
	//handles the incoming commands and sets the appropriate states / flags calls functions
	newDataFlag = 1;
	if(message.command >= 0xE0) //test command range 0xE0-0xEF
		mainRobotState = TEST;
	else if (message.command == 0xD0)
	{
		movingFlag = 0;
		stopRobot();
	}
	else if(message.command >= 0xD1 && message.command <= 0xD3) //Manual command range 0xD1-0xD3
		mainRobotState = MANUAL;
	else if (message.command == 0xD4)
		//move robot randomly
		mfRandomMovementGenerator();		
	else if (message.command == 0xD5)
		mainRobotState = DOCKING;
		//0xD6 and D7 are also reserved for docking 
		//at a later date for different methods if required
	else if (message.command == 0xD8)
		obstacleAvoidanceEnabledFlag = 0;
	else if (message.command == 0xD9)
		obstacleAvoidanceEnabledFlag = 1;
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
	for (uint8_t i = 0; i < message.length; i++)//for each entry in the array
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

/*
* Function:
* void getNewCommunications(struct frame_info *frame, struct message_info *message)
*
* Checks for new communications and handlles the interpretation of them
*
* Inputs:
* pointer to frame_info struct and pointer to message_info struct
*
* Returns:
* none
*
* Implementation:
* Checks if the XBee frame buffer is full indicating new data is ready to be read
* If so, interpret the new XBee API frame and use to fill the message buffer
* Then is the message buffer is full interpret the swarm message
*
*/
void getNewCommunications(struct frame_info *frame, struct message_info *message)
{
	if(FrameBufferInfoGetFull(frame) == 0)	//Check for a received XBee Message
	{
		InterpretXbeeAPIFrame(*frame); //Interpret the received XBee Message
		if(MessageBufferInfoGetFull(message) == 0) //Check for a message from the swarm
			InterpretSwarmMessage(*message);//Interpret the message
	}
}

void InterpretXbeeAPIFrame(struct frame_info frame)
{
	//copy information from the frame info structure to local variables
	int index = frame.index;
	uint8_t frame_type = frame.type;
	int length = frame.length;

	//Temporary variable to store value from buffer
	uint8_t temp;

	//Behaviour depends on type of frame received
	switch(frame_type)
	{
		case AT_COMMAND_RESPONSE:
			//XBEE: AT Command Response Received (N/H)
			break;
		
		case MODEM_STATUS:
			//XBEE: Modem Status Received (N/H)
			break;
		
		case ZIGBEE_TRANSMIT_STATUS:
			//XBEE: Transmit Status Received (N/H)
			break;
		
		case ZIGBEE_RECEIVE_PACKET:
			//XBEE: Data Packet Received
			FrameBufferOut = index;	//Update the location we read from within the FrameBuffer

			//The source's 64-Bit address
			FrameBufferGet(&temp);		
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			
			//The source's 16-Bit address
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			
			//Receive options
			FrameBufferGet(&temp);	
			
			//Received Data
			if(FrameBufferGet(&temp) == 0)
			{
				MessageBufferInfoPut(MessageBufferIn,temp,length-12); //Store information about received message
				for(int i = 1; i <= length-12; i++)
				{
					//Take data from FrameBuffer and put it into the MessageBuffer 
					FrameBufferGet(&temp);
					MessageBufferPut(temp);
				}
			}
			break;
		
		case ZIGBEE_EXPLICIT_RX_INDICATOR:
			//XBEE: Explicit Data Packet Received (N/H)
			break;
		
		case ZIGBEE_IO_DATA_SAMPLE_RX_INDICATOR:
			//XBEE: IO Sample Received (N/H)
			break;

		case XBEE_SENSOR_READ_INDICATOR:
			//XBEE: Sensor Read Indicator Received (N/H)
			break;

		case NODE_IDENTIFICATION_INDICATOR:
			//XBEE: Node Identification Indicator Received (N/H)
			break;

		case REMOTE_COMMAND_RESPONSE:
			//XBEE: Remote Command Response Received (N/H)
			break;
		
		case EXTENDED_MODEM_STATUS:
			//XBEE: Extended Modem Status Received (N/H)
			break;
		
		case OTA_FIRMWARE_UPDATE_STATUS:
			//XBEE: OTA Firmware Update Status Received (N/H)
			break;
		
		case ROUTE_RECORD_INDICATOR:
			//XBEE: Route Record Indicator Received (N/H);
			break;
		
		case MANY_TO_ONE_ROUTE_REQUEST_INDICATOR:
			//XBEE: Many To One Route Request Indicator Received (N/H);
			break;

		default:
			//WARNING ERROR XBEE: unhandled message received;
			break;
	}
}

void CommunicationSetup(void)
{
	REG_PMC_PCER0 |= (1 << 16);				//Enable clock access to UART3
	REG_PIOB_PDR |= (1<<11)|(1<<10);		//Enable peripheral control of PB10 (URXD3) and PB11 (UTXD3) both connected to peripheral B
	REG_UART3_MR |= (0<<14) | (0x4<<9);		//set as no parity, normal mode
	REG_UART3_BRGR = 651;					//Set Baud rate for 9600 from a 100MHZ clock
	REG_UART3_CR |= (1<<2)|(1<<3)|(1<<8);	//Reset receiver, transmitter and status bits
	REG_UART3_CR |= (1<<4)|(1<<6);			//Enable receiver and transmitter
	REG_UART3_IER |= (1<<0);				//ENABLE UART RXRDY interrupt
	
	NVIC_EnableIRQ(16);						//ENABLE the UART3 system interrupts

	//Initializes buffers to beginning of arrays
	FrameBufferInit();
	FrameBufferInfoInit();
	MessageBufferInit();
	MessageBufferInfoInit();
}


/******** UART3 Interrupt Handler ********/
void UART3_Handler(void)
{
	uint8_t temp;	//temporary variable to store received byte
		
	static uint8_t receiveState = CASE_START;
	static bool escape = false;		//Flag for escaping received bytes
	static int length;				//Length as reported by XBee Frame	
	static int index;				//Number of bytes received that count towards length of XBee Frame
	static int check;				//Checksum calculation
	static int frame_start_index;	//The position in the FrameBuffer where the data of this XBee Frame is stored
	static int frame_type;			//The type of received XBee Frame
	

	if(REG_UART3_IMR == UART_IMR_RXRDY)	//if we receive data
	{
		temp = REG_UART3_RHR;	//store the incoming data in a temporary variable 

		if(temp == FRAME_DELIMITER && receiveState != CASE_START ) //if we receive a start byte out of sequence
		{
			receiveState = CASE_START;	//reset back to the start state	
		}
		else if(temp == ESCAPE_BYTE) //if the next byte needs to be escaped
		{
			escape = true;	//set the flag
		}
		else if(escape) //if the current byte needs to be escaped
		{
			temp ^= 0x20;	//reverse the escape procedure
			escape = false;	//reset the flag
		}

		if(escape == false)	//we only go through the receive states if the data has been escaped
		{
			switch(receiveState)
			{
				case CASE_START:
					if(temp == FRAME_DELIMITER)
					{
						//reset our book-keeping variables and updates the receive state
						length = 0;
						index = 0;
						check = 0;
						receiveState = CASE_LENGTH_MSB;
					}
					break;

				case CASE_LENGTH_MSB:
					//Calculates the length using the first length byte and updates the receive state
					length = temp*256;
					receiveState = CASE_LENGTH_LSB;
					break;

				case CASE_LENGTH_LSB:
					//Calculates the length using the second length byte and updates the receive state
					length =+ temp;
					receiveState = CASE_FRAME_TYPE;
					break;

				case CASE_FRAME_TYPE:
					frame_type = temp;					//Receives and stores the Frame type
					check += temp;						//Calculates the checksum over the received byte
					index++;							//Updates the number of received bytes that count towards the XBee frame length
					frame_start_index = FrameBufferIn;	//Stores the location of the Frame Data in the FrameBuffer
					receiveState = CASE_DATA;			//Updates the receive state
					break;

				case CASE_DATA:	
					FrameBufferPut(temp); //Stores the Received data into the FrameBuffer
					check += temp; //Calculates the checksum over the received byte
					index++; //Updates the number of received bytes that count towards the XBee frame length

					if(index == length) //Checks if we have received all the data and if we have updates the receive state
					{
						receiveState = CASE_CHECKSUM;
					}
					
					break;
				
				case CASE_CHECKSUM:
					check += temp;		//Calculates the checksum over the received byte
					check &= 0xFF;		//Final Step of checksum calculation for XBee Frame
					if(check == 0xFF)	//Verifies the calculated checksum value
					{
						FrameBufferInfoPut(frame_start_index, frame_type, index -1); //Stores Frame info in buffer
					}
					receiveState = CASE_START; //Resets receive state d
				break;
			}
		}
	}
}

void UART3_Write(uint8_t data)
{
	//TODO: Needs waitForFlag here
	while(!(REG_UART3_SR & (1<<1)));	//wait till TXRDY
	REG_UART3_THR = data;				//place data in TX register
}

void SendXbeeAPIFrame(uint8_t * frame_data, int len)
{
	uint16_t length = len;		//length of API Frame Data
	uint8_t data[length + 4];	//Array to store the full Frame
	uint8_t checksum = 0;		//Variable to store checksum calculation
	uint8_t data_length;

	//Forms XBee Frame Header with start delimiter and length
	data[0] = FRAME_DELIMITER;
	data[1] = (uint8_t) (length >> 7) & 0xFF;
	data[2] = length & 0xFF;

	//Copies frame data into the full frame array
	memcpy(data + 3, frame_data, length);
	
	//Calculates the checksum over the frame data array
	for(int i = 0; i < length; i++)
	{
		checksum += frame_data[i];
	}
	
	data[length+3] = 0xFF - checksum;	//Completes final step in checksum calculation and copies it to the full frame array
	data_length = sizeof(data);			//Gets the length of full message
	UART3_Write(data[0]);				//Writes the Frame Delimiter as this shouldn't be escaped

	//Sends the message out the UART, escaping characters on the fly as needed
	for(int i = 1; i <data_length; i++)
	{
		//Checks for bytes that need to be escaped
		if(data[i] == 0x7E || data[i] == 0x7D || data[i] == 0x11 || data[i] == 0x13) 
		{
			UART3_Write(ESCAPE_BYTE);	//Writes the escape byte
			UART3_Write(data[i]^0x20);	//Writes the escaped byte
		}
		else
		{
			UART3_Write(data[i]);	//Writes the byte
		}
	}
}

//Sends an XBee Transmit Request Frame
void SendXbeeAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, uint8_t *data, uint8_t  bytes)
{
	//array to store message
	uint8_t frame_data[bytes + 14];

	frame_data[0] = ZIGBEE_TRANSMIT_REQUEST;	//Frame type
	frame_data[1] = 150;						//frame ID (assigned arbitrary at the moment)

	//Destination 64-Bit Address
	frame_data[2] = (destination_64 & (0xFF00000000000000)) >> 56;
	frame_data[3] = (destination_64 & (0xFF000000000000)) >> 48;
	frame_data[4] = (destination_64 & (0xFF0000000000)) >> 40;
	frame_data[5] = (destination_64 & (0xFF00000000)) >> 32;
	frame_data[6] = (destination_64 & (0xFF000000)) >> 24;
	frame_data[7] = (destination_64 & (0xFF0000)) >> 16;
	frame_data[8] = (destination_64 & (0xFF00)) >> 8;
	frame_data[9] = destination_64 & (0xFF);

	//Destination 16-Bit Address
	frame_data[10] = (destination_16 & (0xFF00)) >> 8;
	frame_data[11] = destination_16 & (0xFF);

	//Broadcast radius
	frame_data[12] = 0x00;  

	//Options
	frame_data[13] = 0x00;  

	//Copies message data to frame array
	memcpy(frame_data + 14, data, bytes);

	//Sends the message
	SendXbeeAPIFrame(frame_data,bytes+14);
}


/****** Buffer functions *******/

void FrameBufferInit(void)
{
	//Initialize all usage variables to the beginning of the array
	FrameBufferIn = 0;
	FrameBufferOut = 0;
	FrameBufferUse = 0;
}

int FrameBufferPut(uint8_t new)
{
	//Check if the Buffer is full
	if(FrameBufferIn == (( FrameBufferOut - 1 + FRAME_BUFFER_SIZE) % FRAME_BUFFER_SIZE))
	{
		return -1; // FrameBuffer Full
	}

	//Put the new byte in to the buffer
	FrameBuffer[FrameBufferIn] = new;

	//Update our buffer variables
	FrameBufferIn = (FrameBufferIn + 1) % FRAME_BUFFER_SIZE;
	FrameBufferUse++;
	return 0; // No errors
}

int FrameBufferGet(uint8_t *old)
{
	//Check to see if the buffer if empty
	if(FrameBufferIn == FrameBufferOut)
	{
		return -1; // FrameBuffer Empty - nothing to get
	}
	
	//fetch the info struct from the buffer
	*old = FrameBuffer[FrameBufferOut];

	//Update our buffer variables
	FrameBufferOut = (FrameBufferOut + 1) % FRAME_BUFFER_SIZE;
	FrameBufferUse--;
	return 0; // No errors
}

void FrameBufferInfoInit(void)
{
	//Initialize all usage variables to the beginning of the array
	FrameBufferInfoIn = 0;
	FrameBufferInfoOut = 0;
	FrameBufferInfoUse = 0;
}

int FrameBufferInfoPut(int ind, uint8_t typ, int len)
{
	//Check if the Buffer is full
	if(FrameBufferInfoIn == (( FrameBufferInfoOut -1 + FRAME_BUFFER_INFO_SIZE) % FRAME_BUFFER_INFO_SIZE))
	{
		return -1; // FrameBufferInfo Full
	}

	//Put the new data in to the buffer
	FrameBufferInfo[FrameBufferInfoIn].index = ind;
	FrameBufferInfo[FrameBufferInfoIn].type = typ;
	FrameBufferInfo[FrameBufferInfoIn].length = len;

	//Update our buffer variables
	FrameBufferInfoIn = (FrameBufferInfoIn + 1) % FRAME_BUFFER_INFO_SIZE;
	FrameBufferInfoUse++;
	return 0; // No errors
}

int FrameBufferInfoGetFull(struct frame_info * info)
{
	//Check to see if the buffer if empty
	if(FrameBufferInfoIn == FrameBufferInfoOut)
	{
		return -1; // FrameBufferInfo Empty - nothing to get
	}

	//fetch the info struct from the buffer
	*info = FrameBufferInfo[FrameBufferInfoOut];

	//Update our buffer variables
	FrameBufferInfoOut = (FrameBufferInfoOut + 1) % FRAME_BUFFER_INFO_SIZE;
	FrameBufferInfoUse--;
	return 0; // No errors
}

void MessageBufferInit(void)
{
	//Initialize all usage variables to the beginning of the array
	MessageBufferIn = 0;
	MessageBufferOut = 0;
	MessageBufferUse = 0;
}

int MessageBufferPut(uint8_t new)
{
	//Check if the Buffer is full
	if(MessageBufferIn == (( MessageBufferOut - 1 + MESSAGE_BUFFER_SIZE) % MESSAGE_BUFFER_SIZE))
	{
		return -1; // MessageBuffer Full
	}

	//Put the new byte in to the buffer
	MessageBuffer[MessageBufferIn] = new;

	//Update our buffer variables
	MessageBufferIn = (MessageBufferIn + 1) % MESSAGE_BUFFER_SIZE;
	MessageBufferUse++;
	return 0; // No errors
}

int MessageBufferGet(char *old)
{
	//Check to see if the buffer if empty
	if(MessageBufferIn == MessageBufferOut)
	{
		return -1; // MessageBuffer Empty - nothing to get
	}

	//fetch the byte from the buffer
	*old = MessageBuffer[MessageBufferOut];

	//Update our buffer variables
	MessageBufferOut = (MessageBufferOut + 1) % MESSAGE_BUFFER_SIZE;
	MessageBufferUse--;
	return 0; // No errors
}

void MessageBufferInfoInit(void)
{
	//Initialize all usage variables to the beginning of the array
	MessageBufferInfoIn = 0;
	MessageBufferInfoOut = 0;
	MessageBufferInfoUse = 0;
}

int MessageBufferInfoPut(int ind, uint8_t cmd, int len)
{
	//Check if the Buffer is full
	if(MessageBufferInfoIn == (( MessageBufferInfoOut - 1 + MESSAGE_BUFFER_INFO_SIZE) % MESSAGE_BUFFER_INFO_SIZE))
	{
		return -1; // MessageBufferInfo Full
	}
	
	//Put the new data in to the buffer
	MessageBufferInfo[MessageBufferInfoIn].index = ind;
	MessageBufferInfo[MessageBufferInfoIn].command = cmd;
	MessageBufferInfo[MessageBufferInfoIn].length = len;

	//Update our buffer variables
	MessageBufferInfoIn = (MessageBufferInfoIn + 1) % MESSAGE_BUFFER_INFO_SIZE;
	MessageBufferInfoUse++;
	return 0; // No errors
}

int MessageBufferInfoGetFull(struct message_info * info)
{
	//Check to see if the buffer if empty
	if(MessageBufferInfoIn == MessageBufferInfoOut)
	{
		return -1; // MessageBufferInfo Empty - nothing to get
	}

	//fetch the info struct from the buffer
	*info = MessageBufferInfo[MessageBufferInfoOut];

	//Update our buffer variables
	MessageBufferInfoOut = (MessageBufferInfoOut + 1) % MESSAGE_BUFFER_INFO_SIZE;
	MessageBufferInfoUse--;
	return 0; // No errors
}
