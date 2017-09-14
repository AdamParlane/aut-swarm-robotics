/*
* optInterface.c
*
* Author : Adam Parlane (adam.parlane@outlook.com) Github: AdamParlane
* Created: 15/04/2017 3:31:20 PM
* 
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* SPI driver for optical mouse sensor ADNS-7530 used for navigation
* in the 2017 swarm robotics project for Mark Beckerleg, AUT
*
* Mouse Sensor Data sheet: http://www.pixart.com.tw/upload/ADNS-7530%20DS_S_V1.0_20130514110834.pdf
*
* Contains the following functions: 
* void SPI_Write(char writeAddress, char spiData);
* char SPI_Read(char readAddress);
* void SPI_Init(void);
* void mouseInit(void);
* int mouseTestBasic(void);
* void Get_Mouse_XY(Position *sys);
* 
*
* Functionality of each function is explained before each function
* This .c file should be paired with optInterface.h
*
*/ 

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include <math.h>
#include "opt_interface.h"

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function: void SPI_Init(void)
*
* Initializes the SPI to communicate with the optical mouse sensor
*
* No input or return values
*
* Implementation:
* Enable clock access to SPI using REG_PMC
* Disable SPI write protect by unlocking with SPI write protect key
* Reset the SPI software
* Setup I/O pins:
*	MISO PA12
*	MOSI PA13
*	SCLK PA14
*	Chip Select (Active Low) PB14 - VERSION1
							 PA30 - VERSION2
* Put the micro in SPI master mode using the SPI_MR register
* Set salve to PB14
* Set CPOL = 1, 500k baud rate and a 6us timeout in CSR1
*
* NOTE that the current spi.h library for the ATSAM4N8C has a mistake
* The chip select register has only 1 address when there are 4 CSRs (0-3)
* To fix this
*	#define REG_SPI_CSR1                  (0x40008034U) //< \brief (SPI) Chip Select Register 
*	should be added to the upper part of the defines and
*   #define REG_SPI_CSR1 (*(__IO uint32_t*)0x40008034U) /< \brief (SPI) Chip Select Register 
*	should be added to the lower part of the defines
* This will correctly declare the address for the CSR1 register
* In this Swarm robotics project this is already handled by the included spi.h header file
* If this was not included it either needs to be added 
* or the host PC version of spi.h for the SAM4N8C needs to be updated
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void SPI_Init(void)
{
	REG_PMC_PCER0 |= PMC_PCER0_PID21;		//Enable clock access to SPI
	REG_SPI_WPMR |= (SPI_KEY<<8);
	REG_SPI_WPMR &= ~(1<<0);				//Disable SPI write protect
	
	REG_SPI_CR |= SPI_CR_SWRST;				//Software reset
	//Give control of MOSI, MISO & SCLK pins to SPI
	REG_PIOA_PDR |= PIO_PDR_P12;			//Give control of MISO to SPI
	REG_PIOA_PDR |= PIO_PDR_P13;			//Give control of MOSI to SPI
	REG_PIOA_PDR |= PIO_PDR_P14;			//Give control of SCLK to SPI
	REG_SPI_MR |= SPI_MR_MSTR;				//SPI in Master Mode
	REG_PIOA_PDR |= PIO_PDR_P30; //Give control of NPCS2 (on PA30) to SPI
	//set fixed peripheral select(peripheral chosen in SP_MR.PCS instead of SPI_THR.PCS)	
	REG_SPI_MR &= ~SPI_MR_PS;
	REG_SPI_MR |= SPI_MR_PCS(0b1011); //set slave to NPCS2 (only works while SPI_MR_PS = 0)
	REG_SPI_CSR2 |= (1<<0) | (0xF0<<8) | (0x17<<24); // CPOL=1, 500k baud (2us period), 6us DLYBCT	
	REG_SPI_CR |= SPI_CR_SPIEN; //Enable SPI
}

/*
* Function: void mouseInit(void)
*
* Initializes the mouse sensor using SPI as per the data sheet
*
* No input or return values
*
* Implementation:
* Primarily involves writing values to reserved registers as instructed
* Firstly the chip select (active low) is driven high then low to select the chip
* Power up reset command is set
* Observation register is cleared
* Observation Register is read to ensure bits 0-3 have been set
* Several reserved registers are written to
* Laser Current is set to full with range of 4-10mA, inverse of laser current must also be set 
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void mouseInit(void)
{
	//default CPI = 800, run rate = 8ms
	//temporary variables used to read the observation register on mouse startup
	char dummyVar;
	
	//Reset SPI Port
	REG_PIOA_SODR |= (1<<30);				//Drive NCS High
	REG_PIOA_CODR |= (1<<30);				//Drive NCS Low	

	//initialize mouse sensor
	SPI_Write(OPT_PWR_UP_RESET, 0x5A);		//Power Up Reset
	delay_ms(1);
	SPI_Write(OPT_OBSERVATION, 0x00);		//clear observation register
	delay_ms(1);								//wait at least 1 frame
	dummyVar = SPI_Read(OPT_OBSERVATION);	//read observation register to check bits 0-3 are set	
	while((dummyVar & 0x0F) != 0x0F)			//check if bits 0-3 have been set
	{
		dummyVar = SPI_Read(OPT_OBSERVATION);
		delay_ms(1);
	}
	dummyVar = SPI_Read(OPT_MOTION);
	dummyVar = SPI_Read(OPT_DELTA_X_L);
	dummyVar = SPI_Read(OPT_DELTA_Y_L);
	dummyVar = SPI_Read(OPT_DELTA_XY_H);
	SPI_Write(0x3C, 0x27);					//reserved 
	SPI_Write(0x22, 0x0A);					//reserved 
	SPI_Write(0x21, 0x01);					//reserved 
	SPI_Write(0x3C, 0x32);					//reserved 
	SPI_Write(0x23, 0x20);					//reserved 
	SPI_Write(0x3C, 0x05);					//reserved 
	SPI_Write(0x37, 0xB9);					//reserved 
	SPI_Write(OPT_LSRPWR_CFG0, 0xFF);		//set laser current to full
	SPI_Write(OPT_LSRPWR_CFG1, 0x00);		//complement of set laser current to full
	SPI_Write(OPT_LASER_CTRL0, 0xC0);		//set laser current range to 4-10mA
	SPI_Write(OPT_LASER_CTRL1, 0x3F);		//complement of set laser current range to 4-10mA
	delay_ms(10);						//allow everything to settle after being initialized
}

/*
* Function: void getMouseXY(RobotGlobalStructure *sys)
*
* Reads the deltaX and deltaY from the mouse sensor
* Writes the received deltaX and deltaY into the position structure using pointers
* 
* Input is a pointer to the position structure instance
* 
* No return value
*
* Implementation:
* Reads the motion register (0x02) to check if motion has been detected since last reading
* If motion has occurred read the delta x and y registers
* dx and dy are 12 bit values and therefore take up 1.5 registers each
* the top 4 bits of each value share a register
* the lowest 8 bit values are read directly
* the top register is read and then split into its x and y components
* this is then combined with the lower values and given the correct sign (using 2s complement)
* before being written to the opticaldx and opticaldy members of the position structure
*
* Improvements:
* Needs more descriptive var names
*
*/
void getMouseXY(RobotGlobalStructure *sys)
{
	uint16_t Xtemp = 0, Ytemp = 0;
	int16_t Xx, Yy; 
	char topX, topY, data2, data3, data4, data5;	
	data2 = SPI_Read(OPT_MOTION);
	if(data2 & (1<<7))
	{
		data3 = SPI_Read(OPT_DELTA_X_L);	//delta x low
		data4 = SPI_Read(OPT_DELTA_Y_L);	//delta y low
		data5 = SPI_Read(OPT_DELTA_XY_H);	//delta xy high
		topX = (data5 & (0xF0)) >> 4;		//only read the 4 MSB of data5
		topY = data5 & (0x0F);				//only read the 4 LSB of data5
		Xtemp = data3 | (topX << 8);
		Ytemp = data4 | (topY << 8);
		Xx = Xtemp << 4;
		Yy = Ytemp << 4;
		//if(Xtemp & (1<<12))					//if MSB of X is set (for 2s complement)
			//Xtemp ^= 0b1000100000000000;	//Make the 2s complement bit be MSB of short
		/*
		I think that the data stored in sys->pos.Optical should be the raw data from the optical sensor
		(without resoloutio conversion) Then the converted data can be stored in sys->pos.x, y, etc
		where the real world units go
		*/
		
		sys->pos.Optical.dx = (float)(Xx * RESOLUTION);
		//if(Ytemp & (1<<12))					//if MSB of Y is set (for 2s complement)
			//Ytemp ^= 0b1000100000000000;	//Make the 2s complement bit be MSB of short
		sys->pos.Optical.dy = (float)(Yy * RESOLUTION);
		//sys->opticalX += sys->Optical.dx;
		//sys->opticalY += sys->Optical.dy;
	}
	else
	{
		sys->pos.Optical.dx = 0;
		sys->pos.Optical.dy = 0;
	}
}

/*
* Function: char mouseTestBasic(void)
*
* Tests the mouse sensor by checking that it returns the correct values
* from information registers
* This will only conclude that the mouse is initialized and communicating correctly
* To check measurements use mouseTestAdvanced which will retrieve measurements
*
* Expected values are as below:
* Product ID			0x31
* Inverse Product ID	0xCE
* Revision ID			0x03
* Inverse Revision ID	0xFC
*
* Returns 1 if all values were correct
* Returns 0 if any value was incorrect, this indicates a problem with the mouse 
* or communication with the mouse
*
* Implementation:
* Uses SPI_Read to read all 4 registers
* Compares the returned values with the expected values
* Returns either 1 (all match) or 0 (at least 1 didn't match)
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
char mouseTestBasic(void)
{
	char prodID, invsProdID, revID, invsRevID;
	prodID = SPI_Read(OPT_PRODUCT_ID);		//should be 0x31
	invsProdID = SPI_Read(OPT_INVS_PROD_ID);//should be 0xCE
	revID = SPI_Read(OPT_REVISION_ID);		//should be 0x03
	invsRevID = SPI_Read(OPT_INVS_REV_ID);	//should be 0xFC
	if(prodID == 0x31 && invsProdID == 0xCE && revID == 0x03 && invsRevID == 0xFE)
		return 1;
	else
		return 0;
}

/*
* Function: void SPI_Write(char, char)
*
* Simple function to write to the SPI
* 1st Argument is the address (char) of the peripheral to write to
* In this program this is exclusively a register on the mouse sensor
* 2nd Argument is the data (char) to be written to the register in argument 1
*
* No return value
*
* Implementation:
* Waits for the transmit data register empty flag to be set
* Loads the Transmit data register with the address of the register to be written to,
* bit 7 is 1 to indicate writing
* Waits for the transmit data register empty flag to be set
* Loads the Transmit data register with the data to written
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void SPI_Write(char writeAddress, char spiData)
{
	//TODO: Needs waitForFlag here
	while(!(REG_SPI_SR & SPI_SR_TDRE));		//Wait for address to move out of TDR
	//Load TDR with peripheral register to be written to.
	REG_SPI_TDR |= (writeAddress |= (1<<7));//Puts 1 into bit 7 to indicates writing to register
	//wait for received data to be ready to be read
	//TODO: Needs waitForFlag here
	while(!(REG_SPI_SR & SPI_SR_TDRE));		//Wait for address to move out of TDR
	REG_SPI_TDR |= spiData;					//Load data to be sent
}


/*
* Function: char SPI_Read(char)
*
* Simple function to read the SPI
* Argument is the address of the peripheral to read from
* In this program this is exclusively a register on the mouse sensor
* 
* Returns a char with the data from the requested register
*
* Implementation:
* Reads the received data register to ensure there is no data to be read
* Loads the Transmit data register with the address of the register to be read
* Reads the return data once the received flag is set and discards it
* This first bit of data will be incorrect due to the way SPI pushes data from the slave to master
* Repeat the process
* When the flag is set the received data register will contain the desired data
* 
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
char SPI_Read(char readAddress)
{
	char data;
	data = REG_SPI_RDR;						//Read the RDR to ensure that the RDRF flag is reset.
	//Load TDR with peripheral register to be read from. 
	REG_SPI_TDR |= (readAddress);			//0 in bit 7 which indicates a reading operation.
	//TODO: Needs waitForFlag here
	while(!(REG_SPI_SR & (1<<0)));			//Wait for first RDRF flag.
	data = REG_SPI_RDR;						//First lot of data which will be incorrect. 
	REG_SPI_TDR |= (readAddress);			//Load TDR again.
	//TODO: Needs waitForFlag here
	while(!(REG_SPI_SR & (1<<0)));			//Wait for second RDRF flag.
	data = REG_SPI_RDR;						//Read the correct data
	return data;
}

/*
* Function: void getMouseSQUAL(void)
*
* Reads the surface quality form the mouse used for debugging and tuning the mouse
*
* Returns a char with the surface quality
*
* Implementation:
* max value is 242 (perfect surface)
* values approaching 0 indicate no surface below the sensor
* Optimal results are achieved when lens is 2.4mm from surface
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void getMouseSQUAL(void)
{
	uint8_t squal = SPI_Read(OPT_SQUAL);
}