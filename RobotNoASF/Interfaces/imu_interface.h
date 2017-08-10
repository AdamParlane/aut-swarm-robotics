/*
* imu_interface.h
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 28/04/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Description:
* imu_interface provides functions that allow both retrieval of data from IMU as
* well as functions required by the IMU DMP driver. Additionally it will provide
* the setup routine for TWI2 on robot V1. There are two different versions of imuInit(),
* twiWriteImu(..) and twiReadImu(..); one for each revision of the PCB because V2 has the IMU
* connected to TWI0 instead of TWI2 on the V1
*
* More info:
* https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
* https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
* DMP manual requires registration on Invensense website and permission. Or you
* can just email me.
*
* Functions:
* int imuInit(void)
* int imuDmpInit(void)
* unsigned char imuDmpStop(void)
* unsigned char imuDmpStart(void)
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
* unsigned short invRow2Scale(const signed char *row)
* void getEulerAngles(struct Position *imuData)
* uint8_t imuReadFifo(struct Position *imuData)
* uint8_t imuCommTest(void)
*
*/

#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "../robot_defines.h"									//System header

///////////////Defines//////////////////////////////////////////////////////////////////////////////
////MPU9250 register addresses
#define IMU_WHOAMI_REG			0x75

////IMU external interrupt pin define
#if defined ROBOT_TARGET_V2
//////Port that the IMU interrupt is on
#define IMU_INT_PORT		(PIOA)
//////Pin that the IMU interrupt is on
#define IMU_INT_PIN			(PIO_PA5)
//////IMU interrupt state (1 means data waiting to be read)
#define imuIntState			(IMU_INT_PORT->PIO_PSR & IMU_INT_PIN)
#endif


///////////////Enumerations/////////////////////////////////////////////////////////////////////////
enum axes
{
	X,
	Y,
	Z,
	W
};

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function: int imuInit(void)
*
* Initialise the IMU. Masterclock and TWI2 MUST be setup first.
*
* No input values
*
* Returns:
* an integer that is the sum of the error values returned by the IMU and DMP drivers. (Should be
* zero if no problems encountered)
*
*/
int imuInit(void);

/*
* Function: int imuDmpInit(void)
*
* Initialises the DMP system and starts it on the IMU
*
* No input values
*
* Returns:
* an integer that is the sum of the error values returned by the IMU and DMP drivers. (Should be
* zero if no problems encountered)
*
*/
int imuDmpInit(void);

/*
* Function:
* unsigned char imuDmpStop(void)
*
* If Digital Motion Processing is running on the IMU then stop it. imuDmpInit() MUST be run
* first! (only once)
*
* Inputs:
* none
*
* Returns:
* 1 if the DMP was running before disabling it.
*
*/
unsigned char imuDmpStop(void);

/*
* Function:
* unsigned char imuDmpStart(void)
*
* If Digital Motion Processing is not running on the IMU then Start it. imuDmpInit() MUST be run
* first! (only once)
*
* Inputs:
* none
*
* Returns:
* 1 if the DMP was already running before starting it.
*
*/
unsigned char imuDmpStart(void);

/*
* Function:
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
*
* Converts the orientation matrix to a scalar value for passing to the IMU by the INV driver
*
* Inputs:
* TODO: Inputs description for invOrientationMatrixToScalar()
*
* Returns:
* TODO: Returns desc for invOrientationMatrixToScalar()
*
*/
unsigned short invOrientationMatrixToScalar(const signed char *mtx);

/*
* Function:
* unsigned short invRow2Scale(const signed char *row)
*
* TODO: short description for invRow2Scale()
*
* Inputs:
* TODO: Inputs desc for invRow2Scale()
*
* Returns:
* TODO: Return desc for invRow2Scale()
*
*/
unsigned short invRow2Scale(const signed char *row);

/*
* Function: void getEulerAngles(struct Position *imuData)
*
* Convert Quaternion numbers from the IMU to Euler rotational angles
*
* Inputs:
* ptQuat is a 4 element numeric array that holds the 4 parts of a quaternion complex number:
* x(i), y(j), z(k), w(omega). Presumably ptQuat is a rate of change of orientation, not an absolute
* orientation value.
*
* Returns:
* eulerAngle is a pointer to an euler_packet_t structure that has three elements: yaw, pitch and
* roll.
*
*/
void getEulerAngles(struct Position *imuData);

/*
* Function:
* void imuReadFifo(void)
*
* Will read data from the IMU's FIFO buffer and store data in the given Position structure
*
* Inputs:
* struct Position *imuData:
*   Pointer to the global robotPosition structure. This is where the read data will be stored
*
* Returns:
* 0 on success; non-zero otherwise
*
*/
uint8_t imuReadFifo(struct Position *imuData);

/*
* Function:
* char imuCommTest(void)
*
* Accesses the IMU on TWI2 to retrieve test character from test register....
*
* Inputs:
* none
*
* Returns:
* should return 0x77 if communication working.
*
*/
uint8_t imuCommTest(void);

#endif /* IMU_INTERFACE_H_ */