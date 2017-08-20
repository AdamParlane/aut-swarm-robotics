/*
 * navigation_functions.c
 *
 * Created: 18/08/2017 9:20:51 AM
 *  Author: Matthew
 */ 
/*
* navigation_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 18/08/2017 9:21:05 AM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Holds functions that periodically retrieve data from the navigation sensors (IMU and mouse) and
* store them in a global data structure for easy access by other parts of the program. Also
* contains functions for converting data from one form to another.
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* uint8_t nfRetrieveNavData(void)
* void nfGetEulerAngles(struct Position *imuData)
* float nfWrapAngle(float angleDeg)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "navigation_functions.h"

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//robotPosition is the global data structure that holds all of the robots navigation info. When
//needed it is usually passed to functions as a pointer to avoid duplication.
struct Position robotPosition =
{
	.x = 0,					//Resets robot position
	.y = 0,					//Resets robot position
	.imuYawOffset = 180		//Ensures that whatever way the robot is facing when powered
							//on is 0 degrees heading.
};

//Read data flag that is set by the external interrupt from the IMU on the V2 or by timer on the V1.
//Is defined in imu_interface.
extern uint8_t checkImuFifo;

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
uint8_t nfRetrieveNavData(void)
{
	if(checkImuFifo)
	{
	//On V1s there is no external interrupt. In order to read data from the FIFO at the right time
	//we check bit 1 in the DMP interrupt status register. If it is set, then we can read from the 
	//FIFO.
#if defined ROBOT_TARGET_V1
		short interruptStatus = 0;
		mpu_get_int_status(&interruptStatus);	//Check the interrupt status register
		if(interruptStatus & 0x02)				//If bit 2 is set
		{
			imuReadFifo(&robotPosition);		//Read IMU's FIFO buffer
			nfGetEulerAngles(&robotPosition);	//Convert IMU quats to Euler angles
			getMouseXY(&robotPosition);			//Update mouse sensor data while at it
			checkImuFifo = 0;					//Reset interrupt flag			
		}
#endif
	//No need to read interrupt registers on the V2 as we have the external interrupt to tell us
	//when to read the FIFO.
#if defined ROBOT_TARGET_V2
		imuReadFifo(&robotPosition);		//Read IMU's FIFO buffer
		nfGetEulerAngles(&robotPosition);	//Convert IMU quats to Euler angles
		getMouseXY(&robotPosition);			//Update mouse sensor data while at it
		checkImuFifo = 0;					//Reset interrupt flag
#endif
		return 0;
	} else
		return 1;
}

/*
* Function: void nfGetEulerAngles(struct Position *imuData)
*
* Convert Quaternion numbers from the IMU to Euler rotational angles
*
* Inputs:
* struct Position *imuData
*   Holds the address to the global robotPosition structure that holds all positional data
*
* Returns:
* Loads Yaw, Pitch and Roll data back into robotPosition.
*
* Implementation:
* After the quaternions have been converted to Euler angles, the Yaw offset is applied which is a
* heading correction obtained from the PC. Once this has been applied, the Yaw value is checked to
* ensure it is still in range (-180<Yaw<180) and corrected if necessary.
*
*/
void nfGetEulerAngles(struct Position *imuData)
{
	float w = imuData->imuQW;				//Pull quaternions from IMU
	float x = imuData->imuQX;
	float y = imuData->imuQY;
	float z = imuData->imuQZ;
	float sqw = w*w;						//Pre-calculate squares
	float sqx = x*x;
	float sqy = y*y;
	float sqz = z*z;
	float unit = sqx + sqy + sqz + sqw;	//Should equal 1, otherwise is correction factor
	float test = x*y + z*w;
	if (test > 0.499*unit)					// singularity at north pole
	{
		imuData->imuRoll = 2 * atan2(x,w);
		imuData->imuPitch = M_PI/2;
		imuData->imuYaw = 0;
		return;
	}
	if (test < -0.499*unit)					// singularity at south pole
	{
		imuData->imuRoll = -2 * atan2(x,w);
		imuData->imuPitch = M_PI/2;
		imuData->imuYaw = 0;
		return;
	}
	imuData->imuRoll = (atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw))*180/M_PI;
	imuData->imuPitch = (asin(2*test/unit))*180/M_PI;
	imuData->imuYaw = (atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw))*180/M_PI;
	//Factor in the Yaw offset (Heading correction from the PC)
	imuData->imuYaw += imuData->imuYawOffset;
	//Wrap imuYaw so its always between -180 and 180 degrees
	imuData->imuYaw = nfWrapAngle(imuData->imuYaw);
}

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
* Implementation:
* TODO:Uses modulus to return the remainder of the given angle divided by 180. If the given angle 
* was less than -180 then this is the new angle. Otherwise if the original angle is greater than 180
* then the remainder has 180 subtracted from it and this becomes the new value. In any other case
* (Which is just if the input angle is less than 180 and greater than -180) just return the input
* value because it is already in range.
*
*/
float nfWrapAngle(float angleDeg)
{
	while(angleDeg > 180.0)
		angleDeg -= 360.0;
	while(angleDeg < -179.99)
		angleDeg += 360.0;
	return angleDeg;
}