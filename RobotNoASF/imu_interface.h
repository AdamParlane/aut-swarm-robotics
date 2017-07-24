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
* the setup routine for TWI2
*
* More info:
* https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
* https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
* DMP manual requires registration on Invensense website and permission. Or you
* can just email me.
*
* Functions:
* int imuInit(void)
* char twi_write_imu(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length, unsigned char const *data)
* char twi_read_imu(unsigned char slave_addr, unsigned char reg_addr,
*						unsigned char length,	unsigned char *data)
* int get_ms(uint32_t *timestamp)
* int delay_ms(uint32_t period_ms)
* unsigned short invOrientationMatrixToScalar(const signed char *mtx)
* unsigned short invRow2Scale(const signed char *row)
* void getEulerAngles(long *ptQuat, euler_packet_t *eulerAngle)
* uint8_t imuCommTest(void)
* void TC0_Handler()
*
*/

#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_

///////////////Includes/////////////////////////////////////////////////////////////////////////////
#include "sam.h"									//System header

///////////////Defines//////////////////////////////////////////////////////////////////////////////
//TWI2 Status registers for the IMU driver
#define IMU_TXCOMP	(REG_TWI2_SR & TWI_SR_TXCOMP)	
#define IMU_RXRDY	(REG_TWI2_SR & TWI_SR_RXRDY)	//if 1, RHR has new byte to be read
#define IMU_TXRDY	(REG_TWI2_SR & TWI_SR_TXRDY)	//if 1, THR is empty or NACK error occurred
#define IMU_NACK	(REG_TWI2_SR & TWI_SR_NACK)		//Check TWI2 Status register for Not Acknowledgd

//Some other status flags that were taken from the ASF TWI library. I don't think these are
//needed any longer, but will keep for now
#define TWI_SUCCESS             0
#define TWI_INVALID_ARGUMENT    1
#define TWI_ARBITRATION_LOST    2
#define TWI_NO_CHIP_FOUND       3
#define TWI_RECEIVE_OVERRUN     4
#define TWI_RECEIVE_NACK        5
#define TWI_SEND_OVERRUN        6
#define TWI_SEND_NACK           7
#define TWI_BUSY                8
#define TWI_ERROR_TIMEOUT       9

////TWI2 slave device addresses
#define TWI2_IMU_ADDR			0x68

////MPU9250 register addresses
#define IMU_WHOAMI_REG			0x75

//Structure that stores converted Euler angles of rotation. Parameter of GetEulerAngles
typedef struct euler_packet 
{
	double pitch;
	double roll;
	double yaw;
} euler_packet_t;

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
*/
unsigned short invOrientationMatrixToScalar(const signed char *mtx);

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
*/
unsigned short invRow2Scale(const signed char *row);

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
*/
void getEulerAngles(long *ptQuat, euler_packet_t *eulerAngle);

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
*/
int get_ms(uint32_t *timestamp);

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
*/
int delay_ms(uint32_t period_ms);

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
*/
char twi_write_imu(unsigned char slave_addr, unsigned char reg_addr, 
					unsigned char length, unsigned char const *data);

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
*/
char twi_read_imu(unsigned char slave_addr, unsigned char reg_addr, 
					unsigned char length, unsigned char *data);

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
*/
void TC0_Handler();

#endif /* IMU_INTERFACE_H_ */