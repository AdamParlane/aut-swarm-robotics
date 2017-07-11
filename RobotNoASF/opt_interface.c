/*
 * optInterface.c
 *
 * Created: 15/03/2017 3:31:20 PM
 *
 *
 * Sets up the mouse sensor using SPI and receives dx, dy vectors from it
 * Translates this into absolute x,y coordinates of the robot
 * In Dev mode will also send these coordinates to via UART to a simple windows forms application for viewing of results
 *
 * Author : adamParlane
 */ 

#include "spi.h"
#include "sam.h"
#include <math.h>
#include "opt_interface.h"


void SPI_Init(void)
{
	REG_PMC_PCER0 |= PMC_PCER0_PID21; //Enable clock access to SPI
	REG_SPI_WPMR |= (0x535049<<8); //SPI write protect key
	REG_SPI_WPMR &= ~(1<<0); //Disable SPI write protect
	
	REG_SPI_CR |= SPI_CR_SWRST; //Software reset
	//Give control of MOSI, MISO & SCLK pins to SPI
	REG_PIOA_PDR |= PIO_PDR_P12; //Give control of MISO to SPI
	REG_PIOA_PDR |= PIO_PDR_P13; //Give control of MOSI to SPI
	REG_PIOA_PDR |= PIO_PDR_P14; //Give control of SCLK to SPI
	REG_PIOB_PER |= (1<<14);		//Enable PIO control of PB12
	REG_PIOB_OER |= (1<<14);		//Set PB12 as output
	REG_SPI_CR |= SPI_CR_SPIEN; //enable SPI	
	//REG_PIOB_PDR |= PIO_PDR_P14; //Give control of NPCS1 (on PB14/Pin 99) to SPI
	REG_SPI_MR |= SPI_MR_MSTR; //SPI in Master Mode
	REG_SPI_MR &= ~SPI_MR_PS; //set fixed peripheral select(peripheral chosen in SP_MR.PCS instead of SPI_THR.PCS)
	REG_SPI_MR |= SPI_MR_PCS(0b1101); //set slave to NPCS1 (only works while SPI_MR_PS = 0)	
	REG_SPI_CSR1 |= (1<<0) | (0xF0<<8) | (0x17<<24); // CPOL=1, 500k baud (2us period), 6us DLYBCT
}

void Mouse_Init(void)
{
	char tempObs, temp02, temp03, temp04, temp05; //temporary variables used to read the observation register on mouse startup
	
	//Reset SPI Port
	REG_PIOB_SODR |= (1<<14);	//Drive NCS High	
	REG_PIOB_CODR |= (1<<14);	//Drive NCS Low
	
	//initialize mouse sensor
	SPI_Write(0x3A, 0x5A);		//Power Up Reset
	delay();
	SPI_Write(0x2E, 0x00);		//clear observation register
	delay();					//wait at least 1 frame
	tempObs = SPI_Read(0x2E);	//read observation register to check all bits 0-3 are set	
	while((tempObs & 0x0F) != 0x0F) //check if bits 0-3 have been set
	{
		tempObs = SPI_Read(0x2E);
		delay();
	}
	temp02 = SPI_Read(0x02);
	temp03 = SPI_Read(0x03);
	temp04 = SPI_Read(0x04);
	temp05 = SPI_Read(0x05);
	SPI_Write(0x3C, 0x27);		//reserved but the instructions say to write it
	SPI_Write(0x22, 0x0A);		//reserved but the instructions say to write it
	SPI_Write(0x21, 0x01);		//reserved but the instructions say to write it
	SPI_Write(0x3C, 0x32);		//reserved but the instructions say to write it
	SPI_Write(0x23, 0x20);		//reserved but the instructions say to write it
	SPI_Write(0x3C, 0x05);		//reserved but the instructions say to write it
	SPI_Write(0x37, 0xB9);		//reserved but the instructions say to write it
	SPI_Write(0x1C, 0xFF);		//set laser current to full
	SPI_Write(0x1D, 0x00);		//complement of set laser current to full so that it works
	SPI_Write(0x1A, 0xC0);		//set laser current range to 4-10mA
	SPI_Write(0x1F, 0x00);		//complement of set laser current range to 4-10mA so that it works
	delay();					//allow everything to settle after being initialized
}

void Get_Mouse_XY(struct Position *mousepPos)
{
	int Xtemp = 0, Ytemp = 0;
	char topX, topY, data2, data3, data4, data5;
	data2 = SPI_Read(0x02);
	if(data2 & (1<<7))
	{
		data3 = SPI_Read(0x03);//delta x low
		data4 = SPI_Read(0x04);//delta y low
		data5 = SPI_Read(0x05);//delta xy high
		topX = (data5 & (0xF0)) >> 4; //only read the 4 MSB of data5
		topY = data5 & (0x0F); //only read the 4 LSB of data5
		Xtemp = data3 | (topX << 8);
		Ytemp = data4 | (topY << 8);
		if(Xtemp & (1<<12))//if MSB of X is set (for 2s complement)
		{
			Xtemp -= 4096;
		}
		mousepPos->dx = Xtemp * resolution;
		if(Ytemp & (1<<12))//if MSB of Y is set (for 2s complement)
		{
			Ytemp -= 4096;
		}
		mousepPos->dy = Ytemp * resolution;
		mousepPos->x += mousepPos->dx;
		mousepPos->y += mousepPos->dy;
	}
}

int Mouse_Test(void)
{
	//this function will check the mouse sensor is returning the correct values
	//Function will return 1 if everything is okay, and 0 if there is a fault
	char prodID, invsProdID, revID, invsRevID;
	//check that product ID and revision ID are correct and match there inverse values also
	prodID = SPI_Read(0x00);	//should be 0x31
	invsProdID = SPI_Read(0x3F);//should be 0xCE
	revID = SPI_Read(0x01);		//should be 0x03
	invsRevID = SPI_Read(0x3E);	//should be 0xFC
	if(prodID == 0x31 && invsProdID == 0xCE && revID == 0x03 && invsRevID == 0xFE)
		return 1;
	else
		return 0;
}

void SPI_Write(char writeAddress, char spiData)
{
	while(!(REG_SPI_SR & SPI_SR_TDRE));			//Wait for address to move out of TDR
	REG_SPI_TDR |= (writeAddress |= (1<<7));	//Load TDR with peripheral register to be written to. Puts 1 into bit 7 to indicates writing to register
	//wait for received data to be ready to be read
	while(!(REG_SPI_SR & SPI_SR_TDRE));			//Wait for address to move out of TDR
	REG_SPI_TDR |= spiData;						//Load data to be sent
}

char SPI_Read(char readAddress)
{
	char data;
	data = REG_SPI_RDR;							//Read the RDR to ensure that the RDRF flag is reset.
	REG_SPI_TDR |= (readAddress);				//Load TDR with peripheral register to be read from. 0  in bit 7 which indicates a reading operation.
	while(!(REG_SPI_SR & (1<<0)));				//Wait for first RDRF flag.
	data = REG_SPI_RDR;							//First lot of data which will be incorrect. Its only being read to reset the RDRF flag.
	REG_SPI_TDR |= (readAddress);				//Load TDR again.
	while(!(REG_SPI_SR & (1<<0)));				//Wait for second RDRF flag.
	data = REG_SPI_RDR;							//Read the correct data
	return data;
}

void delay (void)
{
	for (volatile uint16_t x=0; x<65535;x++)
	{
		
	}
}