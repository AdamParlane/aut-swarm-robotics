#include "robotdefines.h"


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

void setup(void)
{
	REG_WDT_MR = (1 << 15); 				//WDT Disable

	/******** CLOCK Setup @ 100MHz ********/
	REG_EFC_FMR = ((1<<26) | (5<<8));		//Set Flash Wait State for 100MHz
	REG_PMC_WPMR = 0x504D4300;				//Disable PMC write protect
	REG_CKGR_MOR |= (0x37<<16) | (0x14<<8); //Set 5ms main xtal osc. Start up time. Start Up Time = 8 * MOSCXTST / SLCK => MOSCXTST = 20
	REG_CKGR_MOR |= (0x37<<16) | (1<<0);	//Enable the external crystal connected to XIN and XOUT
	while(!(REG_PMC_SR & 0x01));			//Wait for the main crystal oscillator to stabilize
	REG_CKGR_MOR |= (0x37<<16) | (1<<24);	//MAINCK source set to external xtal
	while(!(REG_PMC_SR & 0x10000));			//Wait for the source changeover to be complete
	REG_CKGR_MOR = 0x01371401;				//Disable the RC oscillator
	REG_CKGR_PLLAR |= (1<<29) | (0x03<<0) | (0x18<<16) | (0x3F<<8); //Sets PLL to Divide by 3, Multiply by 25 and wait 63 SLCK cycles
	while(!(REG_PMC_SR & 0x02));			//Wait for PLL LOCKA bit to be set
	REG_PMC_MCKR = (2<<0);					//Set PLLA_CLK as MCK
	while(!(REG_PMC_SR & 0x08));			//Wait for MCK ready
	
	/******** PIO Controller Setup ********/
	REG_PMC_PCER0 |= (1<<11);	//Enable clock access to PIO controller A
	REG_PMC_PCER0 |= (1<<12);	//Enable clock access to PIO controller B
	REG_PMC_PCER0 |= (1<<13);	//Enable clock access to PIO controller C
	REG_PIOA_WPMR = 0x50494F00; //Disable PIOA write protect
	REG_PIOB_WPMR = 0x50494F00; //Disable PIOB write protect
	REG_PIOC_WPMR = 0x50494F00; //Disable PIOC write protect
	
	motor_init();
	SPI_Init();
	//mouseInit(); //May require further testing - Adam
	CommunicationSetup();
	initImu();
	twi0Init();

	/******** LED Setup ********/
	REG_PIOA_PER |= ((1<<28) | (1<<27));	//Enable PIO control of D1 & D3.
	REG_PIOC_PER |= (1<<8);					//Enable PIO control of D2
	REG_PIOA_OER |= ((1<<28) | (1<<27));	//Set D1 & D3 as outputs
	REG_PIOC_OER |= (1<<8);					//Set D2 as an output
	D1off;									//D1 starts up off
	D2off;									//D2 starts up off
	D3off;									//D3 starts up off
	
	/******** ADC SETUP, 10 bit default SINGLE CONVERSION SINGLE CHANNEL MODE  ********/
	REG_ADC_WPMR = 0x41444300;						//Disable ADC write protect
	REG_PMC_PCER0 |= (1<<29);						//Enable peripheral clock on ADC
	REG_ADC_MR |= ((49<<8) | (3<<16) | (2<<28));	//Prescale ADC conversion by 49 (100MHZ/((49+1)x2))=1MHZ. Startup time is 24 ADC clock cycles. Field 28 must be programmed with value 2.
	


	LightSensor_Setup(MUX_LIGHTSENS_R);
	LightSensor_Setup(MUX_LIGHTSENS_L);
	Proximity_Setup(MUX_PROXSENS_A);
	Proximity_Setup(MUX_PROXSENS_B);
	Proximity_Setup(MUX_PROXSENS_C);
	Proximity_Setup(MUX_PROXSENS_D);
	Proximity_Setup(MUX_PROXSENS_E);
	Proximity_Setup(MUX_PROXSENS_F);
	fcInit(); //Sets Voltage and Current registers on FCC
}

