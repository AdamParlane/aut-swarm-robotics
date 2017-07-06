/*
 * communication.c
 *
 *  Author: Mansel Jeffares
 * 	First Build: 10 May 2017
 *	Current Build:  21 May 2017
 *  
 *	Description :
 *		Basic Communication to computer GUI using wireless XBee Module in API Mode
 *
 *	Improvements:
 *		Many
 */

 #include <stdbool.h>
 #include <string.h>
 #include "sam.h"
 #include "communication.h"
 #include "robotdefines.h"

 // X-Bee flow control bytes
#define FRAME_DELIMITER 0x7E
#define ESCAPE_BYTE 0x7D
#define XON 0x11
#define XOFF 0x13

// X-Bee API Frames
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
#define FRAME_BUFFER_ELEMENTS 1000
#define FRAME_BUFFER_SIZE (FRAME_BUFFER_ELEMENTS + 1)

#define FRAME_BUFFER_INFO_ELEMENTS 50
#define FRAME_BUFFER_INFO_SIZE (FRAME_BUFFER_INFO_ELEMENTS + 1)

#define MESSAGE_BUFFER_ELEMENTS 500
#define MESSAGE_BUFFER_SIZE (MESSAGE_BUFFER_ELEMENTS + 1)

#define MESSAGE_BUFFER_INFO_ELEMENTS 50
#define MESSAGE_BUFFER_INFO_SIZE (MESSAGE_BUFFER_INFO_ELEMENTS + 1)

// Buffer variables
uint8_t FrameBuffer[FRAME_BUFFER_SIZE];
int FrameBufferIn, FrameBufferOut, FrameBufferUse;

struct frame_info FrameBufferInfo[FRAME_BUFFER_INFO_SIZE];
int FrameBufferInfoIn, FrameBufferInfoOut, FrameBufferInfoUse;

uint8_t MessageBuffer[MESSAGE_BUFFER_SIZE];
int MessageBufferIn, MessageBufferOut, MessageBufferUse;

struct message_info MessageBufferInfo[MESSAGE_BUFFER_INFO_SIZE];
int MessageBufferInfoIn, MessageBufferInfoOut, MessageBufferInfoUse;

// Local Function Prototypes
void FrameBufferInit(void);
int FrameBufferPut(uint8_t new);
int FrameBufferGet(uint8_t *old);
int FrameBufferPeek(uint8_t *old);
void FrameBufferInfoInit(void);
int FrameBufferInfoPut(int ind, uint8_t typ, int len);
int FrameBufferInfoGet(int * ind, uint8_t * typ, int * len);
//int FrameBufferInfoGetFull(struct frame_info * info);
void MessageBufferInit(void);
int MessageBufferPut(uint8_t new);
int MessageBufferGet(uint8_t *old);
int MessageBufferPeek(uint8_t *old);
void MessageBufferInfoInit(void);
int MessageBufferInfoPut(int ind, uint8_t cmd, int len);
int MessageBufferInfoGet(int * ind, uint8_t * cmd, int * len);
//int MessageBufferInfoGetFull(struct message_info * info);
void UART3_Handler(void);
void UART3_Write(uint8_t data);
void SendXbeeAPIFrame(uint8_t * frame_data, int len);


void InterpretSwarmMessage(struct message_info message)
{
	int index = message.index;
	uint8_t message_command = message.command;
	int length = message.length;

	uint8_t data[50];
	uint16_t temp;

	switch(message_command)
	{
		case COMMUNICATION_TEST:
			data[0] = 0x00;
			SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,data,1);
			break;

		case BATTERY_DATA:
			temp = ADC_ReadCH(BV);
			data[0] = BATTERY_DATA;
			data[1] = (temp & (0xFF00)) >> 8;
			data[2] = temp & (0xFF);
			SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,data,3);
			//to get voltage multiply by 5/1000
			break;
		
		case NAV_IMU_QW:
			
			break;

		default:

			break;
	}
}



void InterpretXbeeAPIFrame(struct frame_info frame)
{
	int index = frame.index;
	uint8_t frame_type = frame.type;
	int length = frame.length;

	uint8_t temp;

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
			FrameBufferOut = index;
			FrameBufferGet(&temp);		
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			
			FrameBufferGet(&temp);
			FrameBufferGet(&temp);
			
			FrameBufferGet(&temp);	
			
			if(FrameBufferGet(&temp) == 0)
			{
				MessageBufferInfoPut(MessageBufferIn,temp,1);
				for(int i = 1; i <= length-12; i++)
				{
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

	FrameBufferInit();
	FrameBufferInfoInit();
	MessageBufferInit();
	MessageBufferInfoInit();
}


/******** UART3 Interrupt Handler ********/
void UART3_Handler(void)
{
	uint8_t temp;

	static bool escape = false;
	static uint8_t receiveState = CASE_START;
	static int length;
	static int index;
	static int check;
	static int frame_start_index;
	static int frame_type;
	

	if(REG_UART3_IMR == UART_IMR_RXRDY)	//if we receive data
	{
		temp = REG_UART3_RHR;

		if(temp == FRAME_DELIMITER && receiveState != CASE_START)
		{
			receiveState = CASE_START;

			//XXXX error partial message received will need some kinda of handling to remove them from the buffer (update the tail and the head)
			//XXX this might do it but needs testing
			/*
			if(FrameBufferIn > index)
			{
				FrameBufferIn -= FrameBufferIn - index;
			}
			else
			{
				FrameBufferIn -= 50 - FrameBufferIn + index;
			}
			*/			
		}
		else if(temp == ESCAPE_BYTE)
		{
			escape = true;
		}
		else if(escape)
		{
			temp ^= 0x20;
			escape = false;
		}

		if(escape == false)
		{
			switch(receiveState)
			{
				case CASE_START:
					if(temp == FRAME_DELIMITER)
					{
						receiveState = CASE_LENGTH_MSB;
						length = 0;
						index = 0;
						check = 0;
					}
					break;

				case CASE_LENGTH_MSB:
					length = temp*256;
					receiveState = CASE_LENGTH_LSB;
					break;

				case CASE_LENGTH_LSB:
					length =+ temp;
					receiveState = CASE_FRAME_TYPE;
					break;

				case CASE_FRAME_TYPE:
					frame_type = temp;
					check += temp;
					
					index++;
					frame_start_index = FrameBufferIn;
					receiveState = CASE_DATA;
					break;

				case CASE_DATA:	
					FrameBufferPut(temp);
					check += temp;
					index++;

					if(index == length)
					{
						receiveState = CASE_CHECKSUM;
					}
					
					break;
				
				case CASE_CHECKSUM:
					check += temp;
					check &= 0xFF;
					if(check == 0xFF)
					{
						receiveState = CASE_START;
						FrameBufferInfoPut(frame_start_index, frame_type, index -1);
					}
				break;
			}
		}
	}
}

void UART3_Write(uint8_t data)
{
	while(!(REG_UART3_SR & (1<<1)));	//wait till TXRDY
	REG_UART3_THR = data;				//place data in TX register
}

void SendXbeeAPIFrame(uint8_t * frame_data, int len)
{
	uint16_t length = len;

	uint8_t data[length + 4];
	uint8_t checksum = 0;

	data[0] = FRAME_DELIMITER;
	data[1] = (uint8_t) (length >> 7) & 0xFF;
	data[2] = length & 0xFF;

	memcpy(data + 3, frame_data, length);
	

	for(int i = 0; i < length; i++)
	{
		checksum += frame_data[i];
	}

	data[length+3] = 0xFF - checksum;

	uint8_t data_length = sizeof(data);

	UART3_Write(data[0]);

	for(int i = 1; i <data_length; i++)
	{
		if(data[i] == 0x7E || data[i] == 0x7D || data[i] == 0x11 || data[i] == 0x13)
		{
			UART3_Write(ESCAPE_BYTE);
			UART3_Write(data[i]^0x20);
		}
		else
		{
			UART3_Write(data[i]);
		}
	}
}


void SendXbeeAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, uint8_t *data, uint8_t  bytes)
{
	uint8_t frame_data[bytes + 14];

	frame_data[0] = ZIGBEE_TRANSMIT_REQUEST;
	frame_data[1] = 150;						//frame ID (assigned arbitrary at the moment)

	frame_data[2] = (destination_64 & (0xFF00000000000000)) >> 56;
	frame_data[3] = (destination_64 & (0xFF000000000000)) >> 48;
	frame_data[4] = (destination_64 & (0xFF0000000000)) >> 40;
	frame_data[5] = (destination_64 & (0xFF00000000)) >> 32;
	frame_data[6] = (destination_64 & (0xFF000000)) >> 24;
	frame_data[7] = (destination_64 & (0xFF0000)) >> 16;
	frame_data[8] = (destination_64 & (0xFF00)) >> 8;
	frame_data[9] = destination_64 & (0xFF);


	frame_data[10] = (destination_16 & (0xFF00)) >> 8;
	frame_data[11] = destination_16 & (0xFF);

	frame_data[12] = 0x00;  //broadcast radius

	frame_data[13] = 0x00;  //options

	memcpy(frame_data + 14, data, bytes);

	SendXbeeAPIFrame(frame_data,bytes+14);
}




///******** UART3 Transmit Function *******/
//void sendInstruction(uint8_t txrobotID, uint8_t txinstruction, uint8_t txpacketSize)
//{
	//uint8_t x;
	//txBuffer[0] = 0xC4;						//Open header (will need to confirm these addresses)
	//txBuffer[1] = 0x3B;						//Open header
	//txBuffer[2] = txrobotID;				//Robot ID number
	//txBuffer[3] = txinstruction;			//Command to send
	//txBuffer[4] = txpacketSize;				//Number of data bytes
	//txBuffer[txpacketSize + 5] = 0xA5;		//Close header (will need to confirm these addresses)
	//for (x = 0; x < (txpacketSize+6); x++ )
	//{
		//while (!TXRDY);
		//REG_UART3_THR = txBuffer[x];
	//}
//}


// Buffer functions XXXX this can be greatly improved

void FrameBufferInit(void)
{
	FrameBufferIn = FrameBufferOut = 0;
	FrameBufferUse = 0;
}

int FrameBufferPut(uint8_t new)
{
	if(FrameBufferIn == (( FrameBufferOut - 1 + FRAME_BUFFER_SIZE) % FRAME_BUFFER_SIZE))
	{
		return -1; // FrameBuffer Full
	}

	FrameBuffer[FrameBufferIn] = new;
	FrameBufferIn = (FrameBufferIn + 1) % FRAME_BUFFER_SIZE;
	FrameBufferUse++;
	return 0; // No errors
}

int FrameBufferGet(uint8_t *old)
{
	if(FrameBufferIn == FrameBufferOut)
	{
		return -1; // FrameBuffer Empty - nothing to get
	}

	*old = FrameBuffer[FrameBufferOut];
	FrameBufferOut = (FrameBufferOut + 1) % FRAME_BUFFER_SIZE;
	FrameBufferUse--;
	return 0; // No errors
}

int FrameBufferPeek(uint8_t *old)
{
	if(FrameBufferIn == FrameBufferOut)
	{
		return -1; // FrameBuffer Empty - nothing to get
	}

	*old = FrameBuffer[FrameBufferOut];
	return 0; // No errors
}

void FrameBufferInfoInit(void)
{
	FrameBufferInfoIn = FrameBufferInfoOut = 0;
	FrameBufferInfoUse = 0;
}

int FrameBufferInfoPut(int ind, uint8_t typ, int len)
{
	
	if(FrameBufferInfoIn == (( FrameBufferInfoOut -1 + FRAME_BUFFER_INFO_SIZE) % FRAME_BUFFER_INFO_SIZE))
	{
		return -1; // FrameBufferInfo Full
	}

	FrameBufferInfo[FrameBufferInfoIn].index = ind;
	FrameBufferInfo[FrameBufferInfoIn].type = typ;
	FrameBufferInfo[FrameBufferInfoIn].length = len;
	FrameBufferInfoIn = (FrameBufferInfoIn + 1) % FRAME_BUFFER_INFO_SIZE;
	FrameBufferInfoUse++;
	return 0; // No errors
}

int FrameBufferInfoGet(int * ind, uint8_t * typ, int * len)
{
	if(FrameBufferInfoIn == FrameBufferInfoOut)
	{
		return -1; // FrameBufferInfo Empty - nothing to get
	}

	*ind = FrameBufferInfo[FrameBufferInfoOut].index;
	*typ = FrameBufferInfo[FrameBufferInfoOut].type;
	*len = FrameBufferInfo[FrameBufferInfoOut].length;
	FrameBufferInfoOut = (FrameBufferInfoOut + 1) % FRAME_BUFFER_INFO_SIZE;
	FrameBufferInfoUse--;
	return 0; // No errors
}

int FrameBufferInfoGetFull(struct frame_info * info)
{
	if(FrameBufferInfoIn == FrameBufferInfoOut)
	{
		return -1; // FrameBufferInfo Empty - nothing to get
	}

	*info = FrameBufferInfo[FrameBufferInfoOut];

	FrameBufferInfoOut = (FrameBufferInfoOut + 1) % FRAME_BUFFER_INFO_SIZE;
	FrameBufferInfoUse--;
	return 0; // No errors
}

void MessageBufferInit(void)
{
	MessageBufferIn = MessageBufferOut = 0;
	MessageBufferUse = 0;
}

int MessageBufferPut(uint8_t new)
{
	if(MessageBufferIn == (( MessageBufferOut - 1 + MESSAGE_BUFFER_SIZE) % MESSAGE_BUFFER_SIZE))
	{
		return -1; // MessageBuffer Full
	}

	MessageBuffer[MessageBufferIn] = new;
	MessageBufferIn = (MessageBufferIn + 1) % MESSAGE_BUFFER_SIZE;
	MessageBufferUse++;
	return 0; // No errors
}

int MessageBufferGet(uint8_t *old)
{
	if(MessageBufferIn == MessageBufferOut)
	{
		return -1; // MessageBuffer Empty - nothing to get
	}

	*old = MessageBuffer[MessageBufferOut];
	MessageBufferOut = (MessageBufferOut + 1) % MESSAGE_BUFFER_SIZE;
	MessageBufferUse--;
	return 0; // No errors
}

int MessageBufferPeek(uint8_t *old)
{
	if(MessageBufferIn == MessageBufferOut)
	{
		return -1; // MessageBuffer Empty - nothing to get
	}

	*old = MessageBuffer[MessageBufferOut];
	return 0; // No errors
}



void MessageBufferInfoInit(void)
{
	MessageBufferInfoIn = MessageBufferInfoOut = 0;
	MessageBufferInfoUse = 0;
}

int MessageBufferInfoPut(int ind, uint8_t cmd, int len)
{
	if(MessageBufferInfoIn == (( MessageBufferInfoOut - 1 + MESSAGE_BUFFER_INFO_SIZE) % MESSAGE_BUFFER_INFO_SIZE))
	{
		return -1; // MessageBufferInfo Full
	}

	MessageBufferInfo[MessageBufferInfoIn].index = ind;
	MessageBufferInfo[MessageBufferInfoIn].command = cmd;
	MessageBufferInfo[MessageBufferInfoIn].length = len;
	MessageBufferInfoIn = (MessageBufferInfoIn + 1) % MESSAGE_BUFFER_INFO_SIZE;
	MessageBufferInfoUse++;
	return 0; // No errors
}

int MessageBufferInfoGet(int * ind, uint8_t * cmd, int * len)
{
	if(MessageBufferInfoIn == MessageBufferInfoOut)
	{
		return -1; // MessageBufferInfo Empty - nothing to get
	}

	*ind = MessageBufferInfo[MessageBufferInfoOut].index;
	*cmd = MessageBufferInfo[MessageBufferInfoOut].command;
	*len = MessageBufferInfo[MessageBufferInfoOut].length;
	MessageBufferInfoOut = (MessageBufferInfoOut + 1) % MESSAGE_BUFFER_INFO_SIZE;
	MessageBufferInfoUse--;
	return 0; // No errors
}

int MessageBufferInfoGetFull(struct message_info * info)
{
	if(MessageBufferInfoIn == MessageBufferInfoOut)
	{
		return -1; // MessageBufferInfo Empty - nothing to get
	}

	*info = MessageBufferInfo[MessageBufferInfoOut];

	MessageBufferInfoOut = (MessageBufferInfoOut + 1) % MESSAGE_BUFFER_INFO_SIZE;
	MessageBufferInfoUse--;
	return 0; // No errors
}



/*
struct FIFO
{
	int size, in, out, use;
	void * buffer;
};

void FIFO_init(struct FIFO buf, void * data, int elements)
{
	buf.size = elements + 1;
	buf.in = buf.out = 0;
	buf.buffer = data;
}

int FIFO_put(struct FIFO buf, uint new)
{
	if(buf.in == ((buf.out - 1 + buf.size) % buf.size))
	{
		return -1; // Buffer Full
	}
	*buf.buffer[buf.in] = new;
	buf.in = (buf.in + 1) % buf.size;
	buf.size++;
	return 1;	// No Errors
}
*/


/*
int FrameBufferPut(uint8_t new)
{
	if(FrameBufferIn == (( FrameBufferOut - 1 + FRAME_BUFFER_SIZE) % FRAME_BUFFER_SIZE))
	{
		return -1; // FrameBuffer Full
	}

	FrameBuffer[FrameBufferIn] = new;
	FrameBufferIn = (FrameBufferIn + 1) % FRAME_BUFFER_SIZE;
	FrameBufferUse++;
	return 0; // No errors
}

int FrameBufferGet(uint8_t *old)
{
	if(FrameBufferIn == FrameBufferOut)
	{
		return -1; // FrameBuffer Empty - nothing to get
	}

	*old = FrameBuffer[FrameBufferOut];
	FrameBufferOut = (FrameBufferOut + 1) % FRAME_BUFFER_SIZE;
	FrameBufferUse--;
	return 0; // No errors
}
*/