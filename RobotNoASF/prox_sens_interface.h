/*
 * prox_sens_interface.h
 *
 * Created: 11/07/2017 10:05:52 AM
 *  Author: Matthew
 */ 


#ifndef PROX_SENS_INTERFACE_H_
#define PROX_SENS_INTERFACE_H_

///////////////////Defines//////////////////////////
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

///////////////////Functions////////////////////////////
void Proximity_Setup(uint8_t channel);
uint16_t Proximity_Data_Read(uint8_t channel);



#endif /* PROX_SENS_INTERFACE_H_ */