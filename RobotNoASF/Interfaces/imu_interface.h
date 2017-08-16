/*
* imu_interface.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
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
* void imuGetEulerAngles(struct Position *imuData)
* uint8_t imuReadFifo(struct Position *imuData)
* uint8_t imuCommTest(void)
* void imuApplyYawCorrection(float correctHeading, struct Position *imuData)
*
*/

#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"									//System header

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
////MPU9250 register addresses
#define IMU_WHOAMI_REG			0x75

////IMU conversion factors:
/////Depending on the sensitivity set for the gyro (from datasheet pg8) will convert gyro
/////output to degrees per second
#define IMU_GYRO_FS_250_CONV		0.0076335877862595	//250dps	(1/131)
#define IMU_GYRO_FS_500_CONV		0.0152671755725191	//500dps	(1/65.5)
#define IMU_GYRO_FS_1000_CONV		0.0304878048780488	//1000dps	(1/32.8)
#define IMU_GYRO_FS_2000_CONV		0.0609756097560976	//2000dps	(1/16.4)
/////Depending on the sensitivity set in imuInit() for the accel (from datasheet pg9) will convert 
/////accel output to Gs
#define IMU_ACCEL_AFS_2_CONV_G		0.00006103515625	//+-2G		(1/16384)
#define IMU_ACCEL_AFS_4_CONV_G		0.0001220703125		//+-4G		(1/8192)
#define IMU_ACCEL_AFS_8_CONV_G		0.000244140625		//+-8G		(1/4096)
#define IMU_ACCEL_AFS_16_CONV_G		0.00048828125		//+-16G		(1/2048)
#define IMU_ACCEL_AFS_2_CONV_MS2	0.0005987548828125	//+-2G		(9.81/16384)
#define IMU_ACCEL_AFS_4_CONV_MS2	0.001197509765625	//+-4G		(9.81/8192)
#define IMU_ACCEL_AFS_8_CONV_MS2	0.00239501953125	//+-8G		(9.81/4096)
#define IMU_ACCEL_AFS_16_CONV_MS2	0.0047900390625		//+-16G		(9.81/2048)
////Conversion factors used by the program
#define IMU_GYRO_CONV				IMU_GYRO_FS_1000_CONV	//Convert to degrees per second
#define IMU_ACCEL_CONV_G			IMU_ACCEL_AFS_2_CONV_G	//Convert to Gs
#define IMU_ACCEL_CONV_MS2			IMU_ACCEL_AFS_2_CONV_MS2//Convert to ms^2

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

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
* Function: void imuGetEulerAngles(struct Position *imuData)
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
void imuGetEulerAngles(struct Position *imuData);

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

/*
* Function:
* void imuApplyYawCorrection(float correctHeading, struct Position *imuData)
*
* Takes a 'correct' heading and uses it to modify the onboard heading to match.
*
* Inputs:
* float correctHeading
*   Correct heading of the robot (from webcam) (between -180 and 180)
* struct Position *imuData
*   Pointer to the robotPosition structure
*
* Returns:
* none
*
*/
void imuApplyYawCorrection(float correctHeading, struct Position *imuData);

/*
* Function:
* float imuWrapAngle(float angleDeg)
*
* Will take any angle in degrees and convert it to its equivalent value between -180 and 180 degrees
*
* Inputs:
* float angleDeg
*   Angle to wrap
*
* Returns:
* Wrapped equivalent of the given angle
*
*/
float imuWrapAngle(float angleDeg);

#endif /* IMU_INTERFACE_H_ */