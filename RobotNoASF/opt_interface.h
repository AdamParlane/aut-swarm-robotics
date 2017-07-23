/*
* optInterface.h
*
* Author : Adam Parlane (adam.parlane@outlook.com) Github: AdamParlane
* Created: 15/04/2017 3:45:31 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Header file for SPI driver for optical mouse sensor ADNS-7530 used for navigation
* in the 2017 swarm robotics project for Mark Beckerleg, AUT
*
* Mouse Sensor Datasheet: //http://www.pixart.com.tw/upload/ADNS-7530%20DS_S_V1.0_20130514110834.pdf
*
* Contains the following functions:
* void SPI_Write(char writeAddress, char spiData);
* char SPI_Read(char readAddress);
* void SPI_Init(void);
* void mouseInit(void);
* int mouseTestBasic(void);
* void Get_Mouse_XY(struct Position *mousePos);
* void delay (void);
*
* Functionality of each function is explained before each function
* In the this file that is a summary of purpose, input and return values
* In the .c value comments contain an in depth explanation of how the function works
* This .h file should be paired with optInterface.c
* 
*/

#ifndef OPT_INTERFACE_H_
#define OPT_INTERFACE_H_

#include "robot_defines.h"

/************* Defines ***************/
#define RESOLUTION			0.00125

#define SPI_KEY				0x535049 //SPI write protect key for SAM4N8C

//Mouse sensor relevant register addresses
#define OPT_PRODUCT_ID		0x00
#define OPT_REVISION_ID		0x01
#define OPT_MOTION			0x02
#define OPT_DELTA_X_L		0x03
#define OPT_DELTA_Y_L		0x04
#define OPT_DELTA_XY_H		0x05
#define OPT_LASER_CTRL0		0x1A
#define OPT_LSRPWR_CFG0		0x1C
#define OPT_LSRPWR_CFG1		0x1D
#define OPT_LASER_CTRL1		0x1F
#define OPT_OBSERVATION		0x2E
#define OPT_PWR_UP_RESET	0x3A
#define OPT_INVS_REV_ID		0x3E
#define OPT_INVS_PROD_ID	0x3F


/******** Function Prototypes ********/

/*
* Function: void SPI_Init(void)
*
* Initializes the SPI to communicate with the optical mouse sensor
*
* No input or return values
*
*/
void SPI_Init(void);

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
*/
void SPI_Write(char writeAddress, char spiData);

/*
* Function: char SPI_Read(char)
*
* Simple function to read the SPI
* Argument is the address of the peripheral to read from
* In this program this is exclusively a register on the mouse sensor
*
* Returns a char with the data from the requested register
*
*/
char SPI_Read(char readAddress);

/*
* Function: void mouseInit(void)
*
* Initializes the mouse sensor using SPI as per the data sheet
*
* No input or return values
*
*/
void mouseInit(void);

/*
* Function: char mouseTestBasic(void)
*
* Tests the mouse sensor by checking that it returns the correct values
* from information registers
* This will only conclude that the mouse is initialized and communicating correctly
* To check measurements use mouseTestAdvanced which will retrieve measurements
*
*/
char mouseTestBasic(void);

/*
* Function: void getMouseXY(struct Position*)
*
* Reads the deltaX and deltaY from the mouse sensor
* Writes the received deltaX and deltaY into the position structure using pointers
*
* Input is a pointer to the position structure instance
*
* No return value
*
*/
void getMouseXY(struct Position *mousePos);

/*
* Function: void delay(void)
*
* Simple Delay using a for loop called throughout the mouse sensor setup
* To meet mouseInit timing requirements
* Delay is 1/CLK(MHz) * 65535
*
* This program uses a 100MHz Clock making the delay approx 0.66ms
*
* No return value
*/
void delay (void);

#endif /* OPT_INTERFACE_H_ */