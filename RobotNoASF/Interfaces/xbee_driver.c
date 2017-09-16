/*
* xbee_driver.c
*
* Author : Mansel Jeffares/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 11 July 2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Communication to computer GUI using wireless XBee Module in API Mode
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void xbeeInit(void)
* void xbeeCopyData(struct MessageInfo message, uint8_t *data)
* void xbeeInterpretAPIFrame(struct FrameInfo frame)
* void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16,
*                                 uint8_t *data, uint8_t  bytes)
* static void xbeeSendAPIFrame(uint8_t * frame_data, int len)
* int xbeeFrameBufferInfoGetFull(struct FrameInfo * info)
* int xbeeMessageBufferInfoGetFull(struct MessageInfo * info)
* static void xbeeFrameBufferInit(void)
* int xbeeFrameBufferPut(uint8_t new)
* static int xbeeFrameBufferGet(uint8_t *old)
* static void xbeeFrameBufferInfoInit(void)
* int xbeeFrameBufferInfoPut(int ind, uint8_t typ, int len)
* static void xbeeMessageBufferInit(void)
* static int xbeeMessageBufferPut(uint8_t new)
* static int xbeeMessageBufferGet(char *old)
* static void xbeeMessageBufferInfoInit(void)
* static int xbeeMessageBufferInfoPut(int ind, uint8_t cmd, int len)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "xbee_driver.h"
#include "uart_interface.h"

#include "../Functions/motion_functions.h"	//For interpretswarmmsg.. will be moved

#include <string.h>

//////////////[Global Variables]////////////////////////////////////////////////////////////////////
//Buffer variables
//In is the next free location an element can be put into
//Out is the next element to be read out
uint8_t FrameBuffer[FRAME_BUFFER_SIZE];
int FrameBufferIn;
int FrameBufferOut;
int FrameBufferUse;

struct FrameInfo FrameBufferInfo[FRAME_BUFFER_INFO_SIZE];
int FrameBufferInfoIn;
int FrameBufferInfoOut;
int FrameBufferInfoUse;

uint8_t MessageBuffer[MESSAGE_BUFFER_SIZE];
int MessageBufferIn;
int MessageBufferOut;
int MessageBufferUse;

struct MessageInfo MessageBufferInfo[MESSAGE_BUFFER_INFO_SIZE];
int MessageBufferInfoIn;
int MessageBufferInfoOut;
int MessageBufferInfoUse;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
//Private function prototypes. See below for function descriptions
static void xbeeSendAPIFrame(uint8_t * frame_data, int len);
static void xbeeFrameBufferInit(void);
static int xbeeFrameBufferGet(uint8_t *old);
static void xbeeFrameBufferInfoInit(void);
static void xbeeMessageBufferInit(void);
static int xbeeMessageBufferPut(uint8_t new);
static int xbeeMessageBufferGet(char *old);
static void xbeeMessageBufferInfoInit(void);
static int xbeeMessageBufferInfoPut(int ind, uint8_t cmd, int len);

/*
* Function:
* void xbeeInit(void)
*
* Sets up UART3 and the required buffers
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
void xbeeInit(void)
{
	uart3Init();

	//Initializes buffers to beginning of arrays
	xbeeFrameBufferInit();
	xbeeFrameBufferInfoInit();
	xbeeMessageBufferInit();
	xbeeMessageBufferInfoInit();
}

/*
* Function:
* void xbeeCopyData(struct MessageInfo message, uint8_t* data[50])
*
* Copies the received message structure and pointer to an array with the required test command
* data
*
* Input is the message structure from the received data
* after the XBee framing has been stripped
* and a pointer to the array where the new data is to be copied to
*
* No Return Values
*
* Implementation:
* Uses the message index to call the MessageBufferOut
* This ensures that the message copying begins from the correct location
* Simple for loop to copy the array of length message.length
* This array is accessed via pointers and is used by the test Manager
* The Message Buffer Get function returns 0 for success and -1 for failure
* This function will quit on a failed return
*
*/
void xbeeCopyData(struct MessageInfo message, uint8_t *data)
{
	char dataByte;
	char messageError;
	MessageBufferOut = message.index;//sets message buffer reader to correct start address
	for (uint8_t i = 0; i < message.length; i++)//for each entry in the array
	{
		messageError = xbeeMessageBufferGet(&dataByte);	//retrieve the next byte of received message 
														//data
		if(messageError == 0)//if there was NO error
		{
			data[i] = dataByte;//fill the array with the data
		}
		else//if there was an error, exit
			//TO DO: perhaps add some sort of error flagging system??
			return;
	}
}

/*
* Function:
* void xbeeInterpretAPIFrame(struct FrameInfo Xbeeframe)
*
* Interprets and acts on a received xbee message
*
* Inputs:
* struct FrameInfo Xbeeframe:
*   TODO: Mansel input description
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
void xbeeInterpretAPIFrame(struct FrameInfo Xbeeframe)
{
	//copy information from the frame info structure to local variables
	int index = Xbeeframe.index;
	uint8_t frame_type = Xbeeframe.type;
	int length = Xbeeframe.length;

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
			xbeeFrameBufferGet(&temp);		
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			
			//The source's 16-Bit address
			xbeeFrameBufferGet(&temp);
			xbeeFrameBufferGet(&temp);
			
			//Receive options
			xbeeFrameBufferGet(&temp);	
			
			//Received Data
			if(xbeeFrameBufferGet(&temp) == 0)
			{
				xbeeMessageBufferInfoPut(MessageBufferIn,temp,length-12); //Store information about received message
				for(int i = 1; i <= length-12; i++)
				{
					//Take data from FrameBuffer and put it into the MessageBuffer 
					xbeeFrameBufferGet(&temp);
					xbeeMessageBufferPut(temp);
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

/*
* Function:
* void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, 
*                                 uint8_t *data, uint8_t  bytes)
*
* Forms and sends XBee Transmit Request Frame
*
* Inputs:
* uint64_t destination_64:
*   TODO: Mansel input description
* uint16_t destination_16:
*   TODO: Mansel input description
* uint8_t *data:
*   TODO: Mansel input description
* uint8_t  bytes:
*   TODO: Mansel input description
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
void xbeeSendAPITransmitRequest(uint64_t destination_64, uint16_t destination_16,
								uint8_t *data, uint8_t  bytes)
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
	xbeeSendAPIFrame(frame_data,bytes+14);
}

/*
* Function:
* static void xbeeSendAPIFrame(uint8_t * frame_data, int len)
*
* Sends an XBee API Frame
*
* Inputs:
* uint8_t *frame_data:
*   TODO: Mansel input description
* int len:
*   TODO: Mansel input description
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static void xbeeSendAPIFrame(uint8_t *frame_data, int len)
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
	uart3Write(data[0]);				//Writes the Frame Delimiter as this shouldn't be escaped

	//Sends the message out the UART, escaping characters on the fly as needed
	for(int i = 1; i <data_length; i++)
	{
		//Checks for bytes that need to be escaped
		if(data[i] == 0x7E || data[i] == 0x7D || data[i] == 0x11 || data[i] == 0x13)
		{
			uart3Write(ESCAPE_BYTE);	//Writes the escape byte
			uart3Write(data[i]^0x20);	//Writes the escaped byte
		}
		else
		{
			uart3Write(data[i]);	//Writes the byte
		}
	}
}

/*
* Function:
* int xbeeFrameBufferInfoGetFull(struct FrameInfo * info)
*
* Gets the information about the oldest frame from the buffer
*
* Inputs:
* struct FrameInfo * info:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
int xbeeFrameBufferInfoGetFull(struct FrameInfo * info)
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

/*
* Function:
* int xbeeMessageBufferInfoGetFull(struct MessageInfo * info)
*
* Gets the information about the oldest message from the buffer
*
* Inputs:
* struct MessageInfo * info:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
int xbeeMessageBufferInfoGetFull(struct MessageInfo * info)
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

/*
* Function:
* static void xbeeFrameBufferInit(void)
*
* Initialize all usage variables to the beginning of the array
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static void xbeeFrameBufferInit(void)
{
	//Initialize all usage variables to the beginning of the array
	FrameBufferIn = 0;
	FrameBufferOut = 0;
	FrameBufferUse = 0;
}

/*
* Function:
* int xbeeFrameBufferPut(uint8_t new)
*
* Adds a new byte to the end of the array
*
* Inputs:
* uint8_t new:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
int xbeeFrameBufferPut(uint8_t new)
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

/*
* Function:
* static int xbeeFrameBufferGet(uint8_t *old)
*
* Gets the oldest byte from the array
*
* Inputs:
* uint8_t *old:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static int xbeeFrameBufferGet(uint8_t *old)
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

/*
* Function:
* static void xbeeFrameBufferInfoInit(void)
*
* Initialize all usage variables to the beginning of the array
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static void xbeeFrameBufferInfoInit(void)
{
	//Initialize all usage variables to the beginning of the array
	FrameBufferInfoIn = 0;
	FrameBufferInfoOut = 0;
	FrameBufferInfoUse = 0;
}

/*
* Function:
* int xbeeFrameBufferInfoPut(int ind, uint8_t typ, int len)
*
* Adds a element to the end of the array
*
* Inputs:
* int ind:
*   TODO: Mansel input desc
* uint8_t typ:
*   TODO: Mansel input desc
* int len:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
int xbeeFrameBufferInfoPut(int ind, uint8_t typ, int len)
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

/*
* Function:
* static void xbeeMessageBufferInit(void)
*
* Initialize all usage variables to the beginning of the array
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static void xbeeMessageBufferInit(void)
{
	//Initialize all usage variables to the beginning of the array
	MessageBufferIn = 0;
	MessageBufferOut = 0;
	MessageBufferUse = 0;
}

/*
* Function:
* static int xbeeMessageBufferPut(uint8_t new)
*
* Adds a new byte to the end of the array
*
* Inputs:
* uint8_t new:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static int xbeeMessageBufferPut(uint8_t new)
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

/*
* Function:
* static void xbeeFrameBufferInit(void)
*
* Gets the oldest byte from the array
*
* Inputs:
* char *old:
*   TODO: Mansel input desc
*
* Returns:
* TODO: Mansel return desc
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static int xbeeMessageBufferGet(char *old)
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

/*
* Function:
* static void xbeeMessageBufferInfoInit(void)
*
* Initialize all usage variables to the beginning of the array
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static void xbeeMessageBufferInfoInit(void)
{
	//Initialize all usage variables to the beginning of the array
	MessageBufferInfoIn = 0;
	MessageBufferInfoOut = 0;
	MessageBufferInfoUse = 0;
}

/*
* Function:
* static int xbeeMessageBufferInfoPut(int ind, uint8_t cmd, int len)
*
* Adds a element to the end of the array
*
* Inputs:
* int ind:
*   TODO: Mansel input description
* uint8_t cmd:
*   TODO: Mansel input description
* int len:
*   TODO: Mansel input description
*
* Returns:
* none
*
* Implementation:
* TODO: Mansel implementation description
*
*/
static int xbeeMessageBufferInfoPut(int ind, uint8_t cmd, int len)
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
