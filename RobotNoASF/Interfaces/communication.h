/*
 * communication.h
 *
 *  Author: Mansel Jeffares
 * 	First Build: 10 May 2017
 *	Current Build:  11 July 2017
 *  
 *	Description :
 *		Basic Communication to computer GUI using wireless XBee Module in API Mode
 *
 */

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "../robot_setup.h"

struct message_info
{
	int index;
	uint8_t command;
	int length;
};

//64-bit robot addresses
#define COORDINATOR_64 0x0000000000000000
#define BROADCAST_64 0x000000000000FFFF

//16-bit robot addresses
#define ALL_ROUTERS_16 0xFFFC
#define ALL_NON_SLEEPING_DEVICES_16 0xFFFD
#define UNKNOWN_16 0xFFFE
#define ALL_DEVICES_16 0xFFFF

/******* Message codes *******/
//System Messages 0x0* 
#define COMMUNICATION_TEST	0x00
#define BATTERY_DATA		0x01
//Navigation system Messages 0x1*
#define NAV_IMU_QW			0x10
#define NAV_IMU_QX			0x11
#define NAV_IMU_QY			0x12
#define NAV_IMU_QZ			0x13
#define NAV_IMU_DYAW		0x14
#define NAV_OPT_DX			0x15
#define NAV_OPT_DY			0x16
#define NAV_OPT_VEL			0x17
#define NAV_OPT_HDG			0x18
//More to come

int MessageBufferIn, MessageBufferOut, MessageBufferUse;
//Structures for information about the primary buffers
struct frame_info
{
	int index;
	uint8_t type;
	int length;
};
//////////////[Global variables]////////////////////////////////////////////////////////////////////
extern char obstacleAvoidanceEnabledFlag;

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
*/
void convertData(struct message_info message, uint8_t *data);

/**** Public Function Prototypes ****/
void CommunicationSetup(void);								// Sets up UART3 and the required buffers
void InterpretSwarmMessage(struct message_info message);	// Interprets and acts on a received swarm messages
void InterpretXbeeAPIFrame(struct frame_info frame);		// Interprets and acts on a received xbee message
void SendXbeeAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, uint8_t *data, uint8_t  bytes);	// Forms and sends XBee Transmit Request Frame
int FrameBufferInfoGetFull(struct frame_info * info);		// Gets the information about the oldest frame from the buffer
int MessageBufferInfoGetFull(struct message_info * info);	// Gets the information about the oldest message from the buffer

#endif /* COMMUNICATION_H_ */