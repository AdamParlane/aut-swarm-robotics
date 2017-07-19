#include "robotdefines.h"


void setup(void);

#define		batteryLow	1
#define		streamIntervalFlag	1
//enum ROBOT_STATES{TEST, MANUAL, FORMATION, DOCKING}; //main loop functionality
//enum FORMATION_STATES{FOLLOW_THE_LEADER, V, HORIZONTAL_LINE}; //subset of formation (more proof of concept at this stage than actual formations)
//char robotState = MANUAL;
//char formationState = FOLLOW_THE_LEADER;

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
		//Run the testManager if there is a new test command OR its time to stream more data
		if(testCommandFlag == 1 || (message.command == STREAM_DATA && streamIntervalFlag))
		{
			testCommandFlag = 0;
			testManager(message);
		}
		if(batteryLow)
		{
			dockRobot();
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
	
	/****************TWI2(ID22) SETUP***************/
	REG_PMC_PCER0
	|=	(1<<ID_TWI2);						//Enable clock access to TWI2, Peripheral TWI2_ID = 22
	REG_PIOB_PDR
	|=	PIO_PDR_P0							//Enable peripheralB control of PB0 (TWD2)
	|	PIO_PDR_P1;							//Enable peripheralB control of PB1 (TWCK2)
	REG_PIOB_ABCDSR
	|=	PIO_ABCDSR_P0						//Set peripheral B
	|	PIO_ABCDSR_P1;
	REG_TWI2_CR
	=	TWI_CR_SWRST;						//Software Reset
	
	//TWI2 Clock Waveform Setup.
	//1.3uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
	//0.6uSec = ((x * 2^CKDIV)+4) * 10nSec[100MHz]
	REG_TWI2_CWGR
	|=	TWI_CWGR_CKDIV(1)					//Clock speed 400000, fast mode
	|	TWI_CWGR_CLDIV(63)					//Clock low period 1.3uSec
	|	TWI_CWGR_CHDIV(28);					//Clock high period  0.6uSec
	REG_TWI2_CR
	|=	TWI_CR_MSEN							//Master mode enabled
	|	TWI_CR_SVDIS;						//Slave disabled

	/****************TIMER0 SETUP***************/
	//Timer0 is used for delay_ms and get_ms functions required by the imu driver
	//TC Channel Mode Register (Pg877)
	REG_PMC_PCER0
	|=	(1<<ID_TC0);						//Enable TC clock (ID_TC0 is the peripheral identifier for timer counter 0)
	NVIC_EnableIRQ(ID_TC0);
	REG_TC0_CMR0
	|=	TC_CMR_TCCLKS_TIMER_CLOCK3			//Prescaler MCK/32
	|	TC_CMR_WAVE							//Waveform mode
	|	TC_CMR_WAVSEL_UP_RC;				//Clear on RC compare
	//TC interrupt enable register
	REG_TC0_IER0
	|=	TC_IER_CPCS;						//Enable Register C compare interrupt
	//Set Register C
	REG_TC0_RC0
	=	3125;								//Trigger once every 1/1000th of a second (100Mhz/32/1000)
	//Clock control register
	REG_TC0_CCR0
	|=	TC_CCR_CLKEN						//Enable the timer clk.
	|	TC_CCR_SWTRG;

	LightSensor_Setup(Mux_RHS_LightSens);
	LightSensor_Setup(Mux_LHS_LightSens);
	Proximity_Setup(Mux_ProximityA);
	Proximity_Setup(Mux_ProximityB);
	Proximity_Setup(Mux_ProximityC);
	Proximity_Setup(Mux_ProximityD);
	Proximity_Setup(Mux_ProximityE);
	Proximity_Setup(Mux_ProximityF);
	FastChargeController_Setup(); //Sets Voltage and Current registers on FCC
}

