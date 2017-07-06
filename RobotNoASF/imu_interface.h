/*
 * twi_driver.h
 *
 * Created: 28/04/2017 3:43:52 PM
 *  Author: Matthew Witt
 */ 

#ifndef TWI_DRIVER_H_
#define TWI_DRIVER_H_

#include "robotdefines.h"

//Setup the LED PIO (Temporary for seeing if timer is working)
#define LED1					(PIO_PA27)					//Pin that the blue LED is on
#define LEDOFF(pin)				(REG_PIOA_SODR |= pin)		//Active low
#define LEDON(pin)				(REG_PIOA_CODR |= pin)

//TWI2 Status registers for the IMU driver
#define IMU_TXCOMP				(REG_TWI2_SR & TWI_SR_TXCOMP)
#define IMU_RXRDY				(REG_TWI2_SR & TWI_SR_RXRDY)
#define IMU_TXRDY				(REG_TWI2_SR & TWI_SR_TXRDY)
#define IMU_NACK				(REG_TWI2_SR & TWI_SR_NACK)

#define TWI_SUCCESS              0
#define TWI_INVALID_ARGUMENT     1
#define TWI_ARBITRATION_LOST     2
#define TWI_NO_CHIP_FOUND        3
#define TWI_RECEIVE_OVERRUN      4
#define TWI_RECEIVE_NACK         5
#define TWI_SEND_OVERRUN         6
#define TWI_SEND_NACK            7
#define TWI_BUSY                 8
#define TWI_ERROR_TIMEOUT        9

//Stores converted Euler angles of rotation
typedef struct euler_packet {
	double pitch;
	double roll;
	double yaw;
} euler_packet_t;

void init_imu(void);
char twi_write_imu(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
char twi_read_imu(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
int get_ms(uint32_t *timestamp);
int delay_ms(uint32_t period_ms);
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
unsigned short inv_row_2_scale(const signed char *row);
void GetEulerAngles(long *ptQuat, euler_packet_t *eulerAngle);
void TC0_Handler();

#endif /* TWI_DRIVER_H_ */