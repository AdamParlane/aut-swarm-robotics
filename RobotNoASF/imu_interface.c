/*
* imu_interface.c
*
* Author : Matthew Witt (wittsend86@gmail.com)
* Created: 28/04/2017
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Description:
* imu_interface provides functions that allow both retrieval of data from IMU as
* well as functions required by the IMU DMP driver. Additionally it will provide
* the setup routine for TWI2
*
* IMU Datasheet:
* https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
* https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
* DMP manual requires registration on Invensense website and permission. Or you
* can just email me.
* 
* Functions:
* int initImu(void);
* char twi_write_imu(unsigned char slave_addr, unsigned char reg_addr, 
*						unsigned char length, unsigned char const *data);
* char twi_read_imu(unsigned char slave_addr, unsigned char reg_addr, 
*						unsigned char length,	unsigned char *data);
* int get_ms(uint32_t *timestamp);
* int delay_ms(uint32_t period_ms);
* unsigned short invOrientationMatrixToScalar(const signed char *mtx);
* unsigned short invRow2Scale(const signed char *row);
* void getEulerAngles(long *ptQuat, euler_packet_t *eulerAngle);
* void TC0_Handler();
*
*
*/

////////////////Includes/////////////////
#include "imu_interface.h"
#include <tgmath.h>				//Required for atan2 in GetEulerAngles()
#include "sam.h"				//System header
//Invensense Direct Motion Processing Driver Files
#include "IMU-DMP/inv_mpu_dmp_motion_driver_CUSTOM.h"//Direct Motion Processing setup functions
#include "IMU-DMP/inv_mpu_CUSTOM.h"//IMU basic setup and initialisation functions

//Flags and system globals
uint32_t systemTimestamp = 0,	//Number of ms since powerup
check_IMU_FIFO	= 0;	//At what time will the IMUs FIFO next be read?


/*
* Function: int initImu(void)
*
* Initialise the IMU. Masterclock and TWI2 MUST be setup first.
*
* No input values
*
* Returns:
* an integer that is the sum of the error values returned by the IMU and DMP drivers. (Should be
* zero if no problems encountered)
*
* Implementation:
* Master clock needs to be setup for 100MHz first. TWI2 setup will be added in here.
* accel_fsr, gyro_rate, gyro_fsr store retrieved sample rates from IMU after settings have been
* written to it to confirm successful write-out.
* gyro_orientation is a matrix that modifies the output of the IMU to suit its physical orientation.
* The IMU drive is initialised first. The driver is told which sensors want to be used as well as
* the desired sample rates. The configuration is read back for debug purposes.
* Next the Direct Motion Processing firmware is loaded into the IMU. The orientation matrix is
* converted to scalar format and sent to the IMU. Then the DMP is told to send low power quaternion
* data obtained from 6 axes (3x accelerometer axes + 3x gyro axes)
* Next the update rate of the first in first out buffer is set, and the DMP system is started on
* the IMU.
*
*/
int initImu(void)
{
	unsigned char accel_fsr;
	unsigned short gyro_rate, gyro_fsr;
	int result = 0;
	
	//Orientation correction matrix for the IMU
	static signed char gyro_orientation[9] =
	{	-1,	 0,	 0,
		0,	-1,	 0,
		0,	 0,	 1
	};

	//Initialise the IMU's driver	
	result += mpu_init(0);								// Initialise the MPU with no interrupts
	result += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);// Wake up all sensors
	result += mpu_set_sample_rate(200);					// Set 200Hz samplerate (for accel and gyro)											
	result += mpu_set_compass_sample_rate(100);			// Set 100Hz compass sample rate (max)
	
	//Read back configuration in case it was set improperly.
	result += mpu_get_sample_rate(&gyro_rate);
	result += mpu_get_gyro_fsr(&gyro_fsr);
	result += mpu_get_accel_fsr(&accel_fsr);
	
	result += dmp_load_motion_driver_firmware();		// Load the DMP firmware
	//Send the orientation correction matrix
	result += dmp_set_orientation(invOrientationMatrixToScalar(gyro_orientation));
	//result += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | 
	//								DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
	result += dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT);//Enable 6 axis low power quaternions
	result += dmp_set_fifo_rate(200);					//200Hz update rate from the FIFO
	result += mpu_set_dmp_state(1);						//Start DMP
	return result;
}

/*
* Function:
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
*
* Converts the orientation matrix to a scalar value for passing to the IMU by the INV driver
*
* Inputs:
* TODO
*
* Returns:
* TODO
*
* Implementation:
* TODO
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
* !!!!!Not sure
*
* Inputs:
* TODO
*
* Returns:
* TODO
*
* Implementation:
* TODO
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
* Function: void getEulerAngles(long *ptQuat, euler_packet_t *eulerAngle)
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
* Implementation:
* TO COME
*
*/
void getEulerAngles(long *ptQuat, euler_packet_t *eulerAngle)
{
	double w = ptQuat[3];
	double x = ptQuat[0];
	double y = ptQuat[1];
	double z = ptQuat[2];
	double sqw = w*w;
	double sqx = x*x;
	double sqy = y*y;
	double sqz = z*z;
	double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	double test = x*y + z*w;
	if (test > 0.499*unit) { // singularity at north pole
		eulerAngle->pitch = 2 * atan2(x,w);
		eulerAngle->yaw = M_PI/2;
		eulerAngle->roll = 0;
		return;
	}
	if (test < -0.499*unit) { // singularity at south pole
		eulerAngle->pitch = -2 * atan2(x,w);
		eulerAngle->yaw = M_PI/2;
		eulerAngle->roll = 0;
		return;
	}
	eulerAngle->pitch = (atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw))*180/M_PI;
	eulerAngle->yaw = (asin(2*test/unit))*180/M_PI;
	eulerAngle->roll = (atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw))*180/M_PI;
}

/*
* Function: int get_ms(uint32_t *timestamp)
*
* Required by the IMU drivers (hence naming convention). Outputs the system uptime generated from 
* Timer0.
*
* Inputs:
* address of an integer where the timestamp will be stored
*
* Returns:
* function will return 1 if invalid pointer is passed, otherwise a 0 on success
*
* Implementation:
* Retrieves the value stored in systemTimestamp (stores the number of millisecs that have elapsed
* since power on) and drops it at the address given by *timestamp. if *timestamp is an invalid
* address then returns a 1.
*
*/
int get_ms(uint32_t *timestamp)
{
	if(!timestamp)
		return 1;	
	*timestamp = systemTimestamp;
	return 0;
}


/*
* Function: int delay_ms(uint32_t period_ms)
*
* Required by the IMU drivers (hence naming convention). Halts execution for desired number of
* milliseconds.
*
* Inputs:
* period_ms is the number of milliseconds to wait
*
* Returns:
* Always returns 0
*
* Implementation:
* Stores systemTimestamp at the start of the function, then waits until systemTimestamp has
* increased by the amount given in period_ms before continuing.
*
*/
int delay_ms(uint32_t period_ms)
{
	uint32_t startTime = systemTimestamp;
	while(systemTimestamp < (startTime + period_ms));
	return 0;
}

/*
* Function: char twi_write_imu(unsigned char slave_addr, unsigned char reg_addr, 
*								unsigned char length, unsigned char const *data)
*
* Required by the IMU drivers (hence naming convention). Writes the specified number of bytes to a
* register on the given TWI device.
*
* Inputs:
* slave_addr is the address of the device to be written to on TWI2. The address varies even for the
* IMU driver because the IMU and compass have different TWI slave addresses. reg_addr is the
* 8bit address of the register being written to. length is the number of bytes to be written. *data 
* points to the data bytes to be written.
*
* Returns:
* returns 0 on success.
*
* Implementation:
* Master mode on TWI2 is enabled, TWI2 is prepared for transmission ie slave and register addresses
* are set and register address size is set to 1 byte. Next, transmission takes place but there are
* slightly different procedures for single and multi byte transmission. On single byte
* transmission, the STOP state is set in the TWI control register immediately after the byte to be
* sent is loaded into the transmission holding register. On multi-byte transmission, the STOP
* flag isn't set until all bytes have been sent and the transmission holding register is clear.
*
*/
char twi_write_imu(unsigned char slave_addr, unsigned char reg_addr, 
					unsigned char length, unsigned char const *data)
{
	//note txcomp MUST = 1 before writing (according to datasheet)
	REG_TWI2_CR |= TWI_CR_MSEN | TWI_CR_SVDIS;	//Enable master mode
	REG_TWI2_MMR
		=	TWI_MMR_DADR(slave_addr)			//Slave device address
		|	TWI_MMR_IADRSZ_1_BYTE;				//Set register address length to 1 byte
	REG_TWI2_IADR = reg_addr;					//set register address to write to

	if(length == 1)
	{
		REG_TWI2_THR = data[0];					//set up data to transmit
		REG_TWI2_CR = TWI_CR_STOP;				// Send a stop bit
		while(!IMU_TXRDY);						//while Transmit Holding Register not ready. wait.
	} else {
		for(unsigned char b = 0; b < length; b++)//Send data bit by bit until data length is reached
		{
			REG_TWI2_THR = data[b];				//set up data to transmit
			while(!IMU_TXRDY);					//while Transmit Holding Register not ready. wait.
		}
	
		REG_TWI2_CR = TWI_CR_STOP;				// Send a stop bit
	}
	while(!IMU_TXCOMP);							//while transmit not complete. wait.
	return 0;
}

/*
* Function: char twi_read_imu(unsigned char slave_addr, unsigned char reg_addr,
*								unsigned char length, unsigned char const *data)
*
* Required by the IMU drivers (hence naming convention). Reads the specified number of bytes from a
* register on the given TWI device.
*
* Inputs:
* slave_addr is the address of the device to be read from on TWI2. The address varies even for the
* IMU driver because the IMU and compass have different TWI slave addresses. reg_addr is the address
* of the register being read from. length is the number of bytes to be read. The IMU automatically 
* increments the register address when reading more than one byte. *data points to the location in 
* memory where the retrieved data will be stored.
*
* Returns:
* returns 0 on success.
*
* Implementation:
* Master mode on TWI2 is enabled, TWI2 is prepared for transmission ie slave and register addresses
* are set and register address size is set to 1 byte. Next, reception takes place but there are
* different procedures for single and multi byte reception. On single byte reception, the START and
* STOP flags are set simultaneously in TWI2's control register to indicate that only one byte will
* be read before communication is stopped. With multi-byte reception, the START flag is set 
* initially, and the STOP flag in the control register is set when the second to last byte has been
* received (ie there will only be one byte left to receive after the STOP flag is set)
*
* Improvements:
* Could use a timeout feature with the return of a non-zero value if the slave device doesn't
* reply in time (TXCOMP loops). This would stop the code hanging in an endless loop if the IMU 
* decides to stop talking.
*
*/
char twi_read_imu(unsigned char slave_addr, unsigned char reg_addr, 
					unsigned char length, unsigned char *data)
{
	REG_TWI2_CR |= TWI_CR_MSEN | TWI_CR_SVDIS;	//Enable master mode
	REG_TWI2_MMR
		=	TWI_MMR_DADR(slave_addr)			//Slave device address
		|	(TWI_MMR_MREAD)						//Set to read from register
		|	TWI_MMR_IADRSZ_1_BYTE;				//Register addr byte length (0-3)
	REG_TWI2_IADR = reg_addr;					//set up address to read from
	
	if (length == 1)							//If only ready one byte, then START and STOP bits need to be set at the same time
	{
		REG_TWI2_CR
			=	TWI_CR_START
			|	TWI_CR_STOP;					//Send a START and STOP condition as required (single byte read)	
		while(!IMU_RXRDY);						//while Receive Holding Register not ready. wait.
		data[0] = REG_TWI2_RHR;					//store data received		
		while(!IMU_TXCOMP);						//while transmit not complete. wait.
		return 0;
	} else {
		REG_TWI2_CR = TWI_CR_START;				//Send start bit
		for(unsigned char b = 0; b < length; b++)
		{
			while(!IMU_RXRDY);
			data[b] = REG_TWI2_RHR;
			if(b == length-2)
				REG_TWI2_CR = TWI_CR_STOP;	//Send stop on reception of 2nd to last byte
		}
		while(!IMU_TXCOMP);							//while transmit not complete. wait.
	}
	return 0;
}

/*
* Function: void TC0_Handler()
*
* Interrupt handler for Timer0. Is used to help implement get_ms() and delay_ms() functions
* required by the IMU driver. Is also used to trigger reading the IMU's FIFO buffer (until 
* hardware interrupts are implemented). The only interrupt on Timer0 is on Register C compare,
* which will trigger an interrupt once every millisecond
*
* Inputs:
* none
*
* Returns:
* Increments systemTimestamp once every millisecond.
*
* Implementation:
* If the RC compare flag is set then it increments the systemTimestamp, and also checks if 5ms
* has elapsed. If so, will set a flag to read from the IMU's FIFO buffer (unimplemented)
*
*/
void TC0_Handler()
{
	//The interrupt handler for timer counter 0
	//Triggers every 1ms
	if(REG_TC0_SR0 & TC_SR_CPCS)									//If RC compare flag
	{
		systemTimestamp++;
		//Read IMUs FIFO every 5ms. In future this will be done from an external interrupt.
		if(systemTimestamp >= (check_IMU_FIFO + 5))					
		{
			check_IMU_FIFO = systemTimestamp;
		}
	}
}


