/*
 * communication.h
 *
 * Created: 21/05/2017 6:02:32 p.m.
 *  Author: OEM
 */ 


#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "robotdefines.h"

//64bit robot addresses
#define COORDINATOR_64 0x0000000000000000
#define BROADCAST_64 0x000000000000FFFF

//16bit robot addresses
#define ALL_ROUTERS_16 0xFFFC
#define ALL_NON_SLEEPING_DEVICES_16 0xFFFD
#define UNKNOWN_16 0xFFFE
#define ALL_DEVICES_16 0xFFFF

//////// Message codes /////////////
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

// Buffer Structures
struct frame_info
{
	int index;
	uint8_t type;
	int length;
};

struct message_info
{
	int index;
	uint8_t command;
	int length;
};

// Public Function Prototypes
void CommunicationSetup(void);
void InterpretSwarmMessage(struct message_info message);
void InterpretXbeeAPIFrame(struct frame_info frame);
void SendXbeeAPITransmitRequest(uint64_t destination_64, uint16_t destination_16, uint8_t *data, uint8_t  bytes);
int FrameBufferInfoGetFull(struct frame_info * info);
int MessageBufferInfoGetFull(struct message_info * info);

#endif /* COMMUNICATION_H_ */