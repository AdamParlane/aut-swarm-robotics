/*
* uart_interface.h
*
* Author : Mansel Jeffares/Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 27/08/2017 4:15:01 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines and function prototypes for the UART interface
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void uart3Init(void)
* uint8_t uart3Write(uint8_t data)
*
*/

#ifndef UART_INTERFACE_H_
#define UART_INTERFACE_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Time out settings (ms)
#define UART_TXRDY_TIMEOUT	5

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void uart3Init(void)
*
* Initialises UART3 peripheral, ready for use. Used by the Xbee on the robot.
*
* Inputs:
* none
*
* Returns:
* none
*
*/
void uart3Init(void);

/*
* Function:
* uint8_t uart3Write(uint8_t data)
*
* Allows a byte to be written to UART3
*
* Inputs:
* uint8_t data:
*   The byte to be written to the UART
*
* Returns:
* 0 on success, otherwise non zero if failed
*
*/
uint8_t uart3Write(uint8_t data);								//Writes a byte to UART3





#endif /* UART_INTERFACE_H_ */