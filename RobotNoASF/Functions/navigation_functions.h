/*
* navigation_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 18/08/2017 9:21:05 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Defines and function prototypes for the navigation functions.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t nfRetrieveNavData(void)
* void nfGetEulerAngles(struct Position *imuData)
*
*/

#ifndef NAVIGATION_FUNCTIONS_H_
#define NAVIGATION_FUNCTIONS_H_

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t nfRetrieveNavData(void)
*
* Checks if the IMU's FIFO interrupt flag has been set, and if so, will read data from the IMU's
* FIFO buffer, convert the retrieved quaternions to Euler angles and store them and retrieve data
* from the mouse sensor and store it.
*
* Inputs:
* none
*
* Returns:
* 0 if data was retrieved (checkImuFifo flag was set), otherwise returns 1.
*
* Implementation:
* imuReadFifo() reads the data from the IMU's FIFO buffer and stores the data in robotPosition.
* nfGetEulerAngles() takes the quaternion data stored in robotPosition and converts it to useful
* Euler angles (yaw, pitch and roll) and stores it back in robotPosition.
* getMouseXY() retrieves latest data from the mouse sensor and stored it in robotPosition.
*
*/
uint8_t nfRetrieveNavData(void);

/*
* Function: void nfGetEulerAngles(struct Position *imuData)
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
void nfGetEulerAngles(struct Position *imuData);

#endif /* NAVIGATION_FUNCTIONS_H_ */