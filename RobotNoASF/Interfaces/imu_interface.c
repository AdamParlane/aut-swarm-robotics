/*
* imu_interface.c
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
* int imuDmpInit(RobotGlobalStructure *sys)
* void imuDmpStop(void)
* void imuDmpStart(void)
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
* unsigned short invRow2Scale(const signed char *row)
* uint8_t imuReadFifo(RobotGlobalStructure *sys)
* uint8_t imuCommTest(void)
* void imuApplyYawCorrection(float correctHeading, RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"
#include "twimux_interface.h"	//twi and multiplexer
#include "timer_interface.h"
#include "imu_interface.h"
#include "../Functions/navigation_functions.h"

//Invensense Direct Motion Processing Driver Files
#include "../IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.h"//Direct Motion Processing setup functions
#include "../IMU-DMP/inv_mpu_CUSTOM.h"//IMU basic setup and initialisation functions

///////////////Global Vars//////////////////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
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
	//Setup PIO for IMU hardware interrupt
	IMU_INT_PORT->PIO_PER		//Enable the pin
	|=	IMU_INT_PIN;
	IMU_INT_PORT->PIO_ODR		//Make input.
	|= IMU_INT_PIN;

	//IMU INITIALISATION
	//Initialise the IMU's driver	
	result += mpu_init(0);								//Initialise the MPU with no interrupt CBs
	result += mpu_set_int_level(1);						//Make interrupt level active high
	result += mpu_set_gyro_fsr(1000);					//1000dps (Gyro sensitivity)
	result += mpu_set_accel_fsr(2);						//+-2G (Accelerometer sensitivity)
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
int imuDmpInit(char calibrateGyro)
{
	int result = 0;			//If > 0 then error has occurred
	
	//Orientation correction matrix for the IMU. Allows the output to be corrected, no matter
	//how the IMU is orientated relative to the robot.
	static signed char gyro_orientation[9] =
	//  X    Y   Z
	{  -1,	 0,	 0,  //X
		0,	 1,	 0,  //Y
		0,	 0,	 -1  //Z
	};
	//Both X and Z axis are inverted because the chip is mounted upside down.
	
	result += dmp_load_motion_driver_firmware();		// Load the DMP firmware
	//Send the orientation correction matrix
	result += dmp_set_orientation(invOrientationMatrixToScalar(gyro_orientation));
	result += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
									DMP_FEATURE_SEND_CAL_GYRO);
	
	result += dmp_enable_6x_lp_quat(1);
	
	result += dmp_set_fifo_rate(200);			//200Hz update rate from the FIFO as per
												//datasheet (improves accuracy)
	result += dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);//Use continuous interrupts rather than
														//gesture based (pg10 in DMP manual)
	result += mpu_set_dmp_state(1);						//Start DMP (also starts IMU interrupt)
	
	if(calibrateGyro)
	{	
		result += dmp_enable_gyro_cal(1);				//Enable gyro calibration
		delay_ms(8000);
	}
	
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
* These next two functions converts the orientation matrix (see
* gyro_orientation) to a scalar representation for use by the DMP.
* NOTE: These functions are borrowed from Invensense's MPL.
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
* Function:
* void imuReadFifo(void)
*
* Will read data from the IMU's FIFO buffer and store data in the given PositionGroup structure
*
* Inputs:
* RobotGlobalStructure *sys:
*   Pointer to the global sys->pos. structure. This is where the read data will be stored
*
* Returns:
* 0 on success, non zero otherwise
*
* Implementation:
* Sets up variables required as parameters for dmp_read_fifo() (see below for descriptions).
* Calls dmp_read_fifo() which will attempt to read the data from the FIFO buffer. If this fails, 
* this function will exit with non zero value. On success, the sensors parameter is checked against
* bit masks to determine what data was stored in the FIFO so it can be stored in the PositionGroup
* structure. This should save a little bit of time if only some of the data has been sent.
* Next the more parameter is checked. If it is non-zero then there is more data in the FIFO buffer
* still so dmp_read_fifo is called again. When more is finally zero, the timer between the last FIFO
* read and this one is calculated and stored and the timestamp of the current read is stored.
*
*/
uint8_t imuReadFifo(RobotGlobalStructure *sys)
{
	short gyroData[3];					//Stores raw gyro data from IMU (PRY)->(XYZ)
	short accelData[3];					//Stores raw accelerometer data from IMU (XYZ)
	long quatData[4];					//Stores fused quaternion data from IMU (XYZW)
	unsigned long sensorTimeStamp;		//Stores the data Timestamp
	short sensors;						//Says which sensor data was in the FIFO
	unsigned char more;					//Not 0 when there is more data in FIFO after read
	do
	{
		if(dmp_read_fifo(gyroData, accelData, quatData, &sensorTimeStamp, &sensors, &more))
			return 1;					//If FIFO read function returns non zero then error has 
										//occurred, so exit this function with non zero->
		if(sensors & INV_WXYZ_QUAT)		//If quaternion data was in the FIFO
		{
			sys->pos.IMU.qx = quatData[X];	//Store quats in global data structure
			sys->pos.IMU.qy = quatData[Y];
			sys->pos.IMU.qz = quatData[Z];
			sys->pos.IMU.qw = quatData[W];

		}
		if(sensors & INV_XYZ_ACCEL)		//If accelerometer data was in the FIFO
		{
			sys->pos.IMU.accelX = accelData[X]*IMU_ACCEL_CONV_MS2;
			sys->pos.IMU.accelY = accelData[Y]*IMU_ACCEL_CONV_MS2;
			sys->pos.IMU.accelZ = accelData[Z]*IMU_ACCEL_CONV_MS2;
		}
		if(sensors & INV_XYZ_GYRO)		//If gyro data was in the FIFO
		{
			sys->pos.IMU.gyroX = gyroData[X]*IMU_GYRO_CONV;
			sys->pos.IMU.gyroY = gyroData[Y]*IMU_GYRO_CONV;
			sys->pos.IMU.gyroZ = gyroData[Z]*IMU_GYRO_CONV;
		}

	} while(more);						//If there is still more in the FIFO then do it again->
	sys->pos.deltaTime = sensorTimeStamp - sys->pos.timeStamp;
	sys->pos.timeStamp = sensorTimeStamp;
	return 0;
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
* - Return value retrieved from IMU. Should return 0x71 (0x73?) if communication successful.
*
*/
uint8_t imuCommTest(void)
{
	int dmpEnabled = 0;
	unsigned char returnVal = 0;
	
	dmpEnabled = imuDmpStop();		//Stop DMP. Returns 1 if DMP was running.
		
	//Request test byte
	twi0Read(TWI2_IMU_ADDR, IMU_WHOAMI_REG, 1, &returnVal);

	if (dmpEnabled == 1)			//If DMP was running before this function began
		imuDmpStart();				//Restart the DMP
		
	return returnVal;				//return 0x71 (0x73?) on success
}

/*
* Function:
* void imuApplyYawCorrection(float correctHeading, RobotGlobalStructure *sys)
*
* Takes a 'correct' heading and uses it to modify the onboard heading to match.
*
* Inputs:
* float correctHeading
*   Correct heading of the robot (from webcam) (between -180 and 180)
* RobotGlobalStructure *sys
*   Pointer to the sys->pos. structure
*
* Returns:
* none
*
* Implementation:
* First the function checks that correctHeading is between -180 and 180 and corrects it if
* necessary. Then it looks at the difference between the heading provided and the heading reported
* by the IMU and adds the difference to pos.facingOffset to correct it. Finally, it makes sure that
* pos.facingOffset is between -180 and 180 and corrects it if necessary.
* 
* Improvements:
* Will most likely move this from imu_interface to the Navigation module when its created.
*
*/
void imuApplyYawCorrection(int16_t correctHeading, RobotGlobalStructure *sys)
{
	//Make sure correctHeading is in range
	correctHeading = nfWrapAngle(correctHeading);
	//Take difference and apply it to pos.facingOffset.
	sys->pos.facingOffset = correctHeading - sys->pos.IMU.yaw;
	//Wrap pos.facingOffset so its always between -180 and 180 degrees
	sys->pos.facingOffset = nfWrapAngle(sys->pos.facingOffset);
}
