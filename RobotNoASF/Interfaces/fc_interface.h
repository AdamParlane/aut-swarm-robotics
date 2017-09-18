/*
* fc_interface.h
*
* Author : Esmond Mather, Matthew Witt and Adam Parlane
* Created: 11/07/2017 10:23:08 AM
*
* Project Repository:https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines register addresses and states for the fast charge chip. Defines functions for accessing
* and controlling the fast charge chip.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* TI Battery charger Datasheet:http://www.ti.com/lit/ds/symlink/bq24160.pdf
*
* Functions:
* void fcInit(void)
* void fcWatchdogReset(void)
* uint16_t adcBatteryVoltage(void)
*
*/

#ifndef FC_INTERFACE_H_
#define FC_INTERFACE_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Fast charge chip registers

////Control and Status Register
#define FC_STATUS_REG	0x00	//Status register addr (Contains timer reset bit)
//////Bit field read macros
#define fcStatusTmrRst(byte)		((byte & 0x80)>>7)	//Reads TMR_RST
#define fcStatusStat(byte)			((byte & 0x70)>>4)	//Reads STAT code
#define fcStatusSupplySel(byte)		((byte & 0x04)>>4)	//Reads SUPPLY_SEL bit
#define fcStatusFault(byte)			((byte & 0x03)>>0)	//Reads FAULT code
////////STAT Codes
#define FC_STATUS_BF_STAT_NVSD		0x00	//No valid source detected
#define FC_STATUS_BF_STAT_INRDY		0x01	//IN ready
#define FC_STATUS_BF_STAT_USBRDY	0x02	//USB ready
#define FC_STATUS_BF_STAT_CHRGIN	0x03	//Charging from IN
#define FC_STATUS_BF_STAT_CHRGUSB	0x04	//Charging from USB
#define FC_STATUS_BF_STAT_CHRGDONE	0x05	//Charge Done
#define FC_STATUS_BF_STAT_FAULT		0x07	//Fault
////////FAULT Codes
#define FC_STATUS_BF_FAULT_NORM		0x00	//Normal
#define FC_STATUS_BF_FAULT_TS		0x01	//Thermal shutdown
#define FC_STATUS_BF_FAULT_BTF		0x02	//Battery temp fault
#define FC_STATUS_BF_FAULT_WDTE		0x03	//Watchdog timer expired
#define FC_STATUS_BF_FAULT_STE		0x04	//Safety timer expired
#define FC_STATUS_BF_FAULT_INSF		0x05	//IN supply fault
#define FC_STATUS_BF_FAULT_USBSF	0x06	//USB supply fault
#define FC_STATUS_BF_FAULT_BF		0x07	//Battery Fault
////////Supply select bit
#define FC_STATUS_BF_SUPPLY_SEL_IN	0x00	//IN has precedence over USB source
#define FC_STATUS_BF_SUPPLY_SEL_USB	0x01	//USB has precedence over IN source

////Battery/Supply Status Register
#define FC_SUPPLY_REG				0x01

////Control Register
#define FC_CONTROL_REG				0x02	//Control register (Contains CE bit)

////Control/Battery Voltage Register
#define FC_BATVOL_REG				0x03	//Address

////Vender/Part/Revision Register
#define FC_VERSION_REG				0x04	//Address

////Battery Termination/Fast Charge Current Register
#define FC_CHARGE_REG				0x05	//Address

////Safety Timer/NTC Monitor Register
#define FC_NTCMON_REG	0x07	//Register for TS fault bits B2 & B1, 00=normal, 01=nil charge, 
								//10=1/2 current, 11=Vreg reduced.


//Register Initialisation States
#define FC_STATUS_WDRESET 0x80	//Polls the timer reset bit to stop the watchdog from expiring
								//(FC_STATUS_REG)
#define FC_CONTROL_CHARGE_ENABLE	0x0C//Enables CE bit, EN_STAT bit (so we can see status of chip)
								//and enables charge current termination
#define FC_CONTROL_CHARGE_DISABLE	0x0E//Same as above but disables CE bit
#define FC_BATTVOL_INIT	0x8E	//Vreg = 4.2v, input current = 2.5A (FC_BATVOL_REG)
//#define FC_CHARGE_INIT	0xFA	//charge current set to max Ic=2875mA, termination current
#define FC_CHARGE_INIT	0xD7								//Iterm=450mA (default) (FC_CHARGE_REG)

//Fast charge status
#define FC_POWER_CONNECTED		FC_STATUS_BF_STAT_INRDY
#define FC_BATTERY_CHARGING		FC_STATUS_BF_STAT_CHRGIN
#define FC_BATTERY_CHARGED		FC_STATUS_BF_STAT_CHRGDONE

//Union used to abstract the bit masking
//At this stage just for the battery charging status
//But I might look into using it more else where - AP
typedef union
{
	struct  
	{
		unsigned char b0 : 1;
		unsigned char b1 : 1;
		unsigned char b2 : 1;
		unsigned char b3 : 1;
		unsigned char b4 : 1;
		unsigned char b5 : 1;
		unsigned char b6 : 1;
		unsigned char b7 : 1;		
	}bit;
	unsigned char status;
}Register;

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void fcInit(void)
*
* Initialises chip disable PIO output on microntroller and loads initial settings into fast charge
* chip. TWI0 must be initialised first.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void fcInit(void);

/*
* Function:
* void fcWatchdogReset(void)
*
* Resets the watchdog timer on the fast charge chip. Presumable this is so that the chip knows that
* it is being monitored by its master. If watchdog timer is not reset in 30sec, the chip resets to
* default settings.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void fcWatchdogReset(void);

/*
* Function:
* uint8_t fcVersionRead(void)
*
* Returns revision number from version register on FC chip
*
* Inputs:
* none
*
* Returns:
* Returns a 3 bit value with the version number of the chip
*   0x00: Rev 1.0
*   0x01: Rev 1.1
*   0x02: Rev 2.0
*   0x03: Rev 2.1
*   0x04: Rev 2.2
*   0x05: Rev 2.3
*   0x06+: Future revisions
*
*/
uint8_t fcVersionRead(void);

/*
* Function:
* uint8_t fcState(void)
*
* Returns battery charging status
*
* Inputs:
* none
*
* Returns:
* CHARGING for when a valid charging source is connected
* CHARGED for when the battery is charged
* the value of the status control register if there is an error or no charging
*
*/
uint8_t fcState(void);

uint8_t fcEnableCharging(uint8_t enable);

#endif /* FC_INTERFACE_H_ */