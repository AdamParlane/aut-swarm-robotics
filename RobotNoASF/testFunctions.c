/*
* testFunctions.c
*
* Created: 13/07/2017 12:01:50 PM
*  Author: adams
*/

#include "testFunctions.h"

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
	if(testMode == STREAM_DATA)// && ms100Flag)
	{
		SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,transmitTestData,transmitTestDataSize);  //Send the Message
	}
	if(testMode == SINGLE_SAMPLE)
	{
		SendXbeeAPITransmitRequest(BROADCAST_64,UNKNOWN_16,transmitTestData,transmitTestDataSize);  //Send the Message
		testMode = STOP_STREAMING;
	}

}


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

void setTestMotors(uint8_t motorData[])
{
	if(motorData[0] == MOTOR_1 && (motorData[1] & 0x80))//check if bit 7 is set meaning forward
	{
		FIN_1_High;
		RIN_1_Low;
		REG_PWM_CUPD1 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_1 && ~(motorData[1] & 0x80))
	{
		FIN_1_Low;
		RIN_1_High;
		REG_PWM_CUPD1 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_2 && (motorData[1] & 0x80))//check if bit 7 is set meaning forward
	{
		FIN_2_High;
		RIN_2_Low;
		REG_PWM_CUPD2 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_2 && ~(motorData[1] & 0x80))
	{
		FIN_2_Low;
		RIN_2_High;
		REG_PWM_CUPD2 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_3 && (motorData[1] & 0x80))//check if bit 7 is set meaning forward
	{
		FIN_3_High;
		RIN_3_Low;
		REG_PWM_CUPD3 = (motorData[1] & 0x7F);
	}
	else if(motorData[0] == MOTOR_1 && ~(motorData[1] & 0x80))
	{
		FIN_3_Low;
		RIN_3_High;
		REG_PWM_CUPD3 = (motorData[1] & 0x7F);
	}					
}