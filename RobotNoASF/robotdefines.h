
#ifndef ROBOTDEFINES_H_
#define ROBOTDEFINES_H_

//Contains the misc defines that havnt been modularised as of yet (6/7)
//Also has all the headers so it can just be included in every header giving access to everything

/* CONTENTS
	- DEFINES
	- GLOBAL VARIABLES
*/
#include "spi.h"
#include "sam.h"
//#include "robotdefines.h" //Not sure if this line is constructive!
#include "imu_interface.h"
#include "opt_interface.h"
#include "motor_driver.h"
#include "twi0_interface.h"
#include "communication.h"

/*######## DEFINES ########*/

/******** LEDS ********/
#define	D1off 	REG_PIOA_CODR |= (1<<28)
#define	D2off	REG_PIOC_CODR |= (1<<8)
#define	D3off 	REG_PIOA_CODR |= (1<<27)
#define	D1on 	REG_PIOA_SODR |= (1<<28)
#define	D2on 	REG_PIOC_SODR |= (1<<8)
#define	D3on 	REG_PIOA_SODR |= (1<<27)

/******** ADC ********/
#define ADCstartconversion	REG_ADC_CR |= (1<<1)
/*** Line Followers ***/
#define LF0 13	// Far left
#define LF1 15	// Center left
#define LF2 0	// Center right
#define LF3 8	// Far right
/*** Fast Charge Chip ***/
#define BV 14	// Battery voltage level
#define BT 9	// Battery temperature

/******** UART ********/
#define TXRDY (REG_UART3_SR & (1<<1))		//UART TX READY flag

/******STRUCTURES******/
struct Position
//struture to store all the robot side navigation / positioning data
//this will be written to by getMouseXY, getEulerAngles, and another navigation function which combines them
//The struture will store the relevant info from both key sensors and fuse them in an additional function
{
	float opticaldx;
	float opticaldy;
	float opticalx;
	float opticaly;
	float IMUqw;
	float IMUqx;
	float IMUqy;
	float IMUqz;
	float x;
	float y;
	float h;
};

struct Command
//structure to receive the command and interpret it to something useful
{
	char messageClass;
	char commandCode;
	char command[10];
};


/*************FUNCTIONS*************/
uint16_t ADC_ReadCH(uint8_t channel);

#endif /* ROBOTDEFINES_H_ */