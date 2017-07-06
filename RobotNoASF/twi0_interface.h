/*
 * twi0_interface.h
 *
 * Created: 6/07/2017 12:06:55 p.m.
 *  Author: Adam Parlane
 */ 

//This file will contain all the relevant setup and functions relating to 
//TWI mux, proximity, line, light sensors


#ifndef TWI0_INTERFACE_H_
#define TWI0_INTERFACE_H_

#include "sam.h"
#include "robotsetup.h"

/******** TWI ********/
/*** General Commands ***/
#define twi0RXRDY REG_TWI0_SR & (1<<1)
#define twi0TXRDY REG_TWI0_SR & (1<<2)
#define twi0TXCOMP REG_TWI0_SR & (1<<0)
#define twi0NACK REG_TWI0_SR & (1<<8)		//Check TWI0 Status register for Not Acknowledged

/*** Device/Slave Addresses ***/
#define TWI0_Mux_Address 0b1110000			//Mux Address 000
#define TWI0_LightSensorAddress 0x10		//Light sensors
#define TWI0_ProximitySensorAddress 0x39	//Proximity sensors
#define TWI0_FastChargeChipAddress 0x6B		//Battery Charger (Fast Charge Controller)

/*** Fast Charge Controller ***/
/* Registers */
#define statusReg		0x00	// Status register (Contains timer reset bit)
#define controlReg		0x02	// Control register (Contains CE bit)
#define battVReg		0x03	// Battery Voltage register
#define chargeReg		0x05	// Charge Current register
#define NTCmonitorReg	0x07	// Register for TS fault bits B2 & B1, 00=normal, 01=nil charge, 10=1/2 current, 11=Vreg reduced.
/* Values for registers */
#define watchdreset		0x80	// Polls the timer reset bit to stop the watchdog from expiring (statusReg)
#define initControl		0x04    // Ensures that CE bit is clear in case safety timer has gone off in previous charge.
#define initBattV		0x66	// Vreg = 4.0v, input current = 2.5A (battVReg)
#define initCharge		0xFA	// charge current set to max Ic=2875mA, termination current Iterm=100mA (default) (chargeReg)

/*** I2C Mux Channel ***/
/* Only one channel is active at a time */
#define Mux_RHS_LightSens 0b11111000	//Mux Channel 0, Side Panel A
#define Mux_LHS_LightSens 0b11111001	//Mux Channel 1, Side Panel A
#define Mux_ProximityA 0b11111010		//Mux Channel 2, Side Panel A
#define Mux_ProximityB 0b11111111		//Mux Channel 7, Side Panel B
#define Mux_ProximityC 0b11111110		//Mux Channel 6, Side Panel C
#define Mux_ProximityD 0b11111101		//Mux Channel 5, Side Panel D
#define Mux_ProximityE 0b11111100		//Mux Channel 4, Side Panel E
#define Mux_ProximityF 0b11111011		//Mux Channel 3, Side Panel F

/*** Light Sensor ***/
/* Register addresses for reading Colour */
#define LightSensorRed 0x08
#define LightSensorGreen 0x09
#define LightSensorBlue 0x0A
#define LightSensorWhite 0x0B	//White light is detected at the tower for docking alignment
/* Register address for configuration */
#define LightSens_Config 0x00
/* General Commands */
#define LightSens_DetectLowLux 0b00000110		//40ms integration time, Trigger one time, Force Mode
#define LightSens_DetectMedLux 0b00100110		//80ms integration time, Trigger one time, Force Mode
#define LightSens_DetectHighLux 0b01000110		//160ms integration time, Trigger one time, Force Mode
#define LightSens_DetectMaxLux 0b01010110		//1280ms integration time, Trigger one time, Force Mode
#define LightSens_Auto_LowLux 0b00000000		//40ms integration time, No Trigger, Auto Mode
#define LightSens_Disable 0b00000011			//Force Mode, Disable Sensor

/*** Proximity Sensor ***/
/* Register addresses for reading data. (NOTE: Ch0 and Ch1 are two 16-bit registers) */
#define Proximity_Ch0_DATA_L 0x14	//Ch0 ADC low data register
#define Proximity_Ch0_DATA_H 0x15	//Ch0 ADC high data register
#define Proximity_Ch1_DATA_L 0x16	//Ch1 ADC low data register
#define Proximity_Ch1_DATA_H 0x17	//Ch1 ADC high data register
#define Proximity_DataLow 0x18		//Proximity ADC low data register
#define Proximity_DataHigh 0x19		//Proximity ADC high data register
#define Proximity_Status 0x13		//Device status
/* Register addresses for configuration, all R/W */
#define Proximity_Enable 0x00 //Enable of states and interrupts
#define Proximity_PTime 0x02 //Proximity ADC time
#define Proximity_WaitTime 0x03 //Wait time
#define Proximity_Config 0x0D //Configuration
#define Proximity_PPulse 0x0E //Proximity pulse count register
#define Proximity_GainControl 0x0F//Gain Control register. (Gain Defaults to recommended 1x at power up)
#define Proximity_Offset 0x1E //Proximity offset register
#define Proximity_Command_REG_1Byte 0x80 //Sets read/write protocol, (repeated byte, writes to the same register)
#define Proximity_Command_REG_Increment 0xA0 //Sets read/write protocol, (auto increment registers to read successive bytes)
/* Command Codes for proximity sensor register config */
#define WTIME  0xFF // 2.73 ms, minimum Wait time
#define PTIME  0xFF // 2.73 ms, minimum proximity integration time (recommended)
#define PPULSE 0x08 // recommended proximity pulse count is 8 Pulses
#define PDiode 0b00100000//LED = 100mA, Proximity diode select, Proximity gain x1, recommended settings
#define PEnable 0b00000101 //Power ON, Proximity Enable
#define PDisable 0x00	//Power OFF, Proximity Disable

void twi0Init(void);
void FastChargeController_Setup(void);
void LightSensor_Setup(uint8_t channel);
void Proximity_Setup(uint8_t channel);
uint16_t Proximity_Data_Read(uint8_t channel);
uint16_t LightSensor_Data_Read(uint8_t channel);
void TWI0_MuxSwitch(uint8_t channel);
uint8_t TWI0_ReadMuxChannel(void);
void TWI0_Write(uint8_t SlaveAddress, uint8_t intAddress, uint8_t Data);
uint8_t TWI0_ReadSB(uint8_t SlaveAddress, uint8_t intAddress);
uint16_t TWI0_ReadDB(uint8_t SlaveAddress, uint8_t intAddress);
void FastChargeController_WatchDogReset(void);


#endif /* TWI0_INTERFACE_H_ */