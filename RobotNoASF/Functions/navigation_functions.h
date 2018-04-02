/*
* navigation_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 18/08/2017 9:21:05 AM
*
* Project Repository: https://github.com/wittsend/aut-swarm-robotics
*
* Defines and function prototypes for the navigation functions.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
* void nfGetEulerAngles(RobotGlobalStructure *sys)
* float nfWrapAngle(float angleDeg)
* void nfDMPEnable(char enable RobotGlobalStructure *sys)
* void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys)
*
*/

#ifndef NAVIGATION_FUNCTIONS_H_
#define NAVIGATION_FUNCTIONS_H_

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
*
* Checks if the IMU's FIFO interrupt flag has been set, and if so, will read data from the IMU's
* FIFO buffer, convert the retrieved quaternions to Euler angles and store them and retrieve data
* from the mouse sensor and store it.
*
* Inputs:
* none
*
* Returns:
* 0 if data was retrieved (sys.flags.imuCheckFifo flag was set), otherwise returns 1.
*
* Implementation:
* imuReadFifo() reads the data from the IMU's FIFO buffer and stores the data in sys->pos..
* nfGetEulerAngles() takes the quaternion data stored in sys->pos. and converts it to useful
* Euler angles (yaw, pitch and roll) and stores it back in sys->pos..
* getMouseXY() retrieves latest data from the mouse sensor and stored it in sys->pos..
*
*/
uint8_t nfRetrieveNavData(RobotGlobalStructure *sys);

/*
* Function: void nfGetEulerAngles(RobotGlobalStructure *sys)
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
void nfGetEulerAngles(RobotGlobalStructure *sys);

/*
* Function:
* void nfProcessOpticalData(RobotGlobalStructure *sys)
*
* Performs processing on optical mouse data to retrieve real absolute x and y and heading
* information
*
* Inputs:
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure which is where the mouse data and calculated data is
*   is retrieved and stored
*
* Returns:
* none
*
*/
void nfProcessOpticalData(RobotGlobalStructure *sys);

/*
* Function:
* float nfWrapAngle(float angleDeg)
*
* Will take any angle in degrees and convert it to its equivalent value between -180 and 180 degrees
*
* Inputs:
* float angleDeg
*   Angle to wrap
*
* Returns:
* Wrapped equivalent of the given angle
*
*/
float nfWrapAngle(float angleDeg);

/*
* Function:
* void nfDMPEnable(char enable, RobotGlobalStructure *sys)
*
* Enables the DMP on the IMU and resets the sys.pos.IMU.dmpEnabled flag. Provides a wrapper to
* enable the DMP
*
* Inputs:
* char enable:
*   1 to enable DMP and 0 to disable;
* RobotGlobalStructure *sys:
*   Pointer to the global robot data structure
*
* Returns:
* None
*
*/
void nfDMPEnable(char enable, RobotGlobalStructure *sys);

/*
* Function:
* void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys)
*
* Takes the raw data buffer containing position information and updates the robots current position
*
* Inputs:
* uint8_t *rawData
*   Pointer to the data buffer array retrieved from the xbee
* RobotGlobalStructure *sys
*   Pointer to the global robot data structure
*
* Returns:
* none
*
*/
void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys);

//Temp nav testing function
uint8_t nfOpticalTesting(uint8_t speed, uint8_t distance, RobotGlobalStructure *sys);

#endif /* NAVIGATION_FUNCTIONS_H_ */