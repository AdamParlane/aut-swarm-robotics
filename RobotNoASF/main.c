#include "robot_defines.h"


void setup(void);

#define		batteryLow	1
#define		streamIntervalFlag	1


/******** Global Variables ********/
uint8_t SBtest, SBtest1;
uint16_t DBtest, DBtest1, DBtest2;

int main(void)
{
	setup();

	//Comms
	struct frame_info frame;
	struct message_info message;
	//Optical
	struct Position robotPosition;
	robotPosition.x = 0;
	robotPosition.y = 0;
	while(1)
	{
		switch (robotState)
		{
			case TEST:
			if(message.command == STOP_STREAMING)
				robotState = IDLE;
			if(streamIntervalFlag)
				testManager(message);	
			break;
			
			case TEST_ALL:
			break;
			
			case MANUAL:
			break;
			
			case DOCKING:
			//if battery low or manual command set
				dockRobot();
			break;
			
			case FORMATION:
			//placeholder
			break;
			
			case IDLE:
			//idle
			break;
		}
		
		if(FrameBufferInfoGetFull(&frame) == 0)	//Check for a received XBee Message
		{
			InterpretXbeeAPIFrame(frame); //Interpret the received XBee Message

			if(MessageBufferInfoGetFull(&message) == 0) //Check for a message from the swarm
			{
				InterpretSwarmMessage(message);	//Interpret the message
			}
		}
	}
}

/*
* Function:
* void setup(void)
*
* The initialisation routine for all hardware in the robot.
*
* Inputs:
* none
*
* Returns:
* none
*
* Implementation:
* Contains functions which systematically set up each peripheral and hardware device connected to
* the robot's micro controller.
*
* Improvements:
* 
*
*/
void setup(void)
{
	REG_WDT_MR = WDT_MR_WDDIS; 			//Disable system watchdog timer.

	masterClockInit();					//Initialise the master clock to 100MHz
	pioInit();							//Initialise the PIO controllers
	adcSingleConvInit();				//Initialise ADC for single conversion mode
	ledInit();							//Initialise the LEDs on the mid board
	motor_init();						//Initialise the motor driver chips
	SPI_Init();							//Initialise SPI for talking with optical sensor
	CommunicationSetup();				//Initialise communication system
	twi0Init();							//Initialise TWI0 interface
	twi2Init();							//Initialise TWI2 interface
	lightSensInit(MUX_LIGHTSENS_R);		//Initialise Right Light/Colour sensor
	lightSensInit(MUX_LIGHTSENS_L);		//Initialise Left Light/Colour sensor
	proxSensInit(MUX_PROXSENS_A);		//Initialise proximity sensor on panel A
	proxSensInit(MUX_PROXSENS_B);		//Initialise proximity sensor on panel B
	proxSensInit(MUX_PROXSENS_C);		//Initialise proximity sensor on panel C
	proxSensInit(MUX_PROXSENS_D);		//Initialise proximity sensor on panel D
	proxSensInit(MUX_PROXSENS_E);		//Initialise proximity sensor on panel E
	proxSensInit(MUX_PROXSENS_F);		//Initialise proximity sensor on panel F
	fcInit();							//Initialise the fast charge chip
	imuInit();							//Initialise IMU
	//mouseInit();						//May require further testing - Adam
}

