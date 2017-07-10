
#include "twi0_interface.h"



/******** Fast Charge Controller Registry Setup ********/
void FastChargeController_Setup(void)
{
	//Chip disable (CD) line on PB2 set to enable
	REG_PIOB_PER |= (1<<2);		//Give control of PB2 to PIOB controller
	REG_PIOB_OER |= (1<<2);		//Set PB2 as an output
	REG_PIOB_CODR |= (1<<2);	//Set PB2 to low
	TWI0_Write(TWI0_FastChargeChipAddress, controlReg, initControl);	// Ensures that CE bit is clear in case safety timer has gone off in previous charge.
	TWI0_Write(TWI0_FastChargeChipAddress, battVReg, initBattV);		// Vreg = 4.0v, input current = 2.5A
	TWI0_Write(TWI0_FastChargeChipAddress, chargeReg, initCharge);		// charge current set to max Ic=2875mA, termination current Iterm=100mA (default)
}

/******** Light Sensor Registry Setup ********/
//White Light Detection Enabled with low lux (40ms integration time), No Trigger, Auto Mode
//Only sets up one light sensor at a time, not both
void LightSensor_Setup(uint8_t channel)
{
	TWI0_MuxSwitch(channel); //Set multiplexer address to correct device
	TWI0_Write(TWI0_LightSensorAddress, LightSens_Config, LightSens_Auto_LowLux);
}


/******** Light Sensor Data Read ********/
//Retrieves the White Light (16-bit) data from the selected Light Sensor
uint16_t LightSensor_Data_Read(uint8_t channel)
{
	uint16_t data;
	TWI0_MuxSwitch(channel);	//Set multiplexer address to correct device
	data = TWI0_ReadDB(TWI0_LightSensorAddress, LightSensorWhite);
	return data;
}


void FastChargeController_WatchDogReset(void)
{
	TWI0_Write(TWI0_FastChargeChipAddress, statusReg, watchdreset);	//Resets the FCC watch dog timer. Must be done once every 30s or else registers will reset
}