/*
* imu_interface.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 28/04/2017
*void getEulerAngles(struct Position *imuData)
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
* void imuDmpStop(void)
* void imuDmpStart(void)
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
* unsigned short invRow2Scale(const signed char *row)
* void getEulerAngles(struct Position *imuData)
* uint8_t imuCommTest(void)
*
*
*/

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "imu_interface.h"
#include <tgmath.h>				//Required for atan2 in GetEulerAngles()
#include "twimux_interface.h"	//twi and multiplexer
#include "../robot_defines.h"

//Invensense Direct Motion Processing Driver Files
#include "../IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.h"//Direct Motion Processing setup functions
#include "../IMU-DMP/inv_mpu_CUSTOM.h"//IMU basic setup and initialisation functions

///////////////Global Vars//////////////////////////////////////////////////////////////////////////
uint8_t checkImuFifo	= 0;	//A flag to determine that the IMU's FIFO is ready to be read again
extern uint32_t systemTimestamp;

///////////////Functions////////////////////////////////////////////////////////////////////////////
/*
* Function: int imuInit(void)
*
* Initialise TIMER0 and the IMU. Masterclock MUST be setup first.
*
* No input values
*
* Returns:
* an integer that is the sum of the error values returned by the IMU and DMP drivers. (Should be
* zero if no problems encountered)
*
* Implementation:
* MASTER CLOCK NEEDS TO BE SETUP FOR 100MHZ FIRST.
* Next, the IMU driver is initialised. The driver is told which sensors want to be used as well as
* the desired sample rates.
*
*/
int imuInit(void)
{
	int result = 0;		//Return value (when not 0, errors are present)
	
	//MICROCONTROLLER HW SETUP
#if defined ROBOT_TARGET_V2	
	//Setup PIO for IMU hardware interrupt
	IMU_INT_PORT->PIO_PER		//Enable the pin
	|=	IMU_INT_PIN;
	IMU_INT_PORT->PIO_ODR		//Make input.
	|= IMU_INT_PIN;
#endif

	//IMU INITIALISATION
	//Initialise the IMU's driver	
	result += mpu_init(0);								//Initialise the MPU with no interrupt CBs
	result += mpu_set_int_level(1);						//Make interrupt level active high
	
	result += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);// Wake up all sensors
	result += mpu_set_sample_rate(800);					// Set 800Hz samplerate (for accel and gyro)											
	result += mpu_set_compass_sample_rate(100);			// Set 100Hz compass sample rate (max)
	
	return result;
}

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
* Implementation:
* imuInit() NEEDS TO BE RUN FIRST.
* gyro_orientation is a matrix that modifies the output of the IMU to suit its physical orientation.
* Next the Direct Motion Processing firmware is loaded into the IMU. The orientation matrix is
* converted to scalar format and sent to the IMU. Then the DMP is told to send low power quaternion
* data obtained from 6 axes (3x accelerometer axes + 3x gyro axes)
* Next the update rate of the first in first out buffer is set, and the DMP system is started on
* the IMU.
*
*/
int imuDmpInit(void)
{
	int result = 0;			//If > 0 then error has occurred
	//Orientation correction matrix for the IMU
	static signed char gyro_orientation[9] =
	{	-1,	 0,	 0,
		0,	-1,	 0,
		0,	 0,	 1
	};
	result += dmp_load_motion_driver_firmware();		// Load the DMP firmware
	//Send the orientation correction matrix
	result += dmp_set_orientation(invOrientationMatrixToScalar(gyro_orientation));
	result += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
									DMP_FEATURE_SEND_CAL_GYRO);
	result += dmp_set_fifo_rate(10);					//10Hz update rate from the FIFO
	result += dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);//Use continuous interrupts rather than
														//gesture based (pg10 in DMP manual)
	result += mpu_set_dmp_state(1);						//Start DMP (also starts IMU interrupt)
	return result;
}

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
* Implementation:
* Both imuInit() and imuDmpInit() have to have been run first before this function will can run.
* First check if the DMP is running. If so then stop it, otherwise exit the function.
*
* Improvements:
* Have a global flag that indicates that the IMU has been initialised and the DMP firmware loaded
* so that this function won't run without that having being done so. Have an error return value.
*
*/
unsigned char imuDmpStop(void)
{
	unsigned char dmpEnabled = 0;
		
	mpu_get_dmp_state(&dmpEnabled);				//See if DMP was running
	if (dmpEnabled == 1)						//If it was
		mpu_set_dmp_state(0);					//Stop DMP
	return dmpEnabled;
}

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
* Implementation:
* Both imuInit() and imuDmpInit() have to have been run first before this function will can run.
* First check if the DMP isn't running. If so then start it, otherwise exit the function.
*
* Improvements:
* Have a global flag that indicates that the IMU has been initialised and the DMP firmware loaded
* so that this function won't run without that having being done so. Have an error return value.
*
*/
unsigned char imuDmpStart(void)
{
	unsigned char dmpEnabled = 0;
	
	mpu_get_dmp_state(&dmpEnabled);				//See if DMP was already running
	if (dmpEnabled == 0)						//If it wasn't
		mpu_set_dmp_state(1);					//Start DMP
	return dmpEnabled;	
}

/*
* Function:
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
*
* Converts the orientation matrix to a scalar value for passing to the IMU by the INV driver
*
* Inputs:
* TODO: input descriptions for invOrientationMatrixToScalar function
*
* Returns:
* TODO: return value description for invOrientationMatrixToScalar function
*
* Implementation:
* TODO: Implementation description for invOrientationMatrixToScalar function
*
*/
unsigned short invOrientationMatrixToScalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = invRow2Scale(mtx);
    scalar |= invRow2Scale(mtx + 3) << 3;
    scalar |= invRow2Scale(mtx + 6) << 6;

    return scalar;
}

/*
* Function:
* unsigned short invRow2Scale(const signed char *row)
*
* TODO: Short description for invRow2Scale() function
*
* Inputs:
* TODO: Inputs description for invRow2Scale()
*
* Returns:
* TODO: Return description for invRow2Scale()
*
* Implementation:
* TODO: Implementation Description for invRow2Scale()
*
*/
unsigned short invRow2Scale(const signed char *row)
{
	unsigned short b;

	if (row[0] > 0)
	b = 0;
	else if (row[0] < 0)
	b = 4;
	else if (row[1] > 0)
	b = 1;
	else if (row[1] < 0)
	b = 5;
	else if (row[2] > 0)
	b = 2;
	else if (row[2] < 0)
	b = 6;
	else
	b = 7;      // error
	return b;
}

/*
* Function: void getEulerAngles(struct Position *imuData)
*
* Convert Quaternion numbers from the IMU to Euler rotational angles
*
* Inputs:
* struct Position *imuData
*   Holds the address to the global robotPosition structure that holds all positional data
*
* Returns:
* Loads Yaw, Pitch and Roll data back into robotPosition.
*
* Implementation:
* TO COME
*
*/
void getEulerAngles(struct Position *imuData)
{
	double w = imuData->imuQW;
	double x = imuData->imuQX;
	double y = imuData->imuQY;
	double z = imuData->imuQZ;
	double sqw = w*w;
	double sqx = x*x;
	double sqy = y*y;
	double sqz = z*z;
	double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	double test = x*y + z*w;
	if (test > 0.499*unit) { // singularity at north pole
		imuData->imuPitch = 2 * atan2(x,w);
		imuData->imuYaw = M_PI/2;
		imuData->imuRoll = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		imuData->imuPitch = -2 * atan2(x,w);
		imuData->imuYaw = M_PI/2;
		imuData->imuRoll = 0;
		return;
	}
	imuData->imuPitch = (atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw))*180/M_PI;
	imuData->imuYaw = (asin(2*test/unit))*180/M_PI;
	imuData->imuRoll = (atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw))*180/M_PI;
}

/*
* Function:
* uint8_t imuCommTest(void)
*
* Accesses the IMU on TWI2 to retrieve test character from test register....
*
* Inputs:
* none
*
* Returns:
* should return 0x71 if communication working.
*
* Implementation:
* - Disable DMP if necessary
* - Send byte to WHO AM I register on IMU
* - Re-enable DMP if enabled beforehand
* - Return value retrieved from IMU. Should return 0x71 if communication successful.
*
*/
uint8_t imuCommTest(void)
{
	int dmpEnabled = 0;
	unsigned char returnVal = 0;
	
	dmpEnabled = imuDmpStop();		//Stop DMP. Returns 1 if DMP was running.
		
	//Request test byte
#if defined ROBOT_TARGET_V1
	twi2Read(TWI2_IMU_ADDR, IMU_WHOAMI_REG, 1, &returnVal);
#endif
#if defined ROBOT_TARGET_V2
	twi0Read(TWI2_IMU_ADDR, IMU_WHOAMI_REG, 1, &returnVal);
#endif

	if (dmpEnabled == 1)			//If DMP was running before this function began
		imuDmpStart();				//Restart the DMP
		
	return returnVal;				//return 0x71 on success
}





