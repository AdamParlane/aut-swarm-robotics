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
* uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
* void nfGetEulerAngles(RobotGlobalStructure *sys)
* float nfWrapAngle(float angleDeg)
* void nfDMPEnable(char enable RobotGlobalStructure *sys)
* void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys)
*
*/

//////////////[Includes]////////////////////////////////////////////////////////////////////////////
#include "../robot_setup.h"

#include "../IMU-DMP/inv_mpu_CUSTOM.h"

#include "../Interfaces/pio_interface.h"	//temp used by nfOpticalTesting
#include "../Interfaces/imu_interface.h"
#include "../Interfaces/opt_interface.h"
#include "../Interfaces/timer_interface.h"	//temp used by nfOpticalTesting

#include "motion_functions.h"	//Temp used by nfOpticalTesting
#include "navigation_functions.h"

#include <math.h>				//Required for round() in nfProcessOpticalData()
#include <tgmath.h>				//Required for atan2 in nfGetEulerAngles()

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////

//////////////[Global variables]////////////////////////////////////////////////////////////////////
//sys->pos. is the global data structure that holds all of the robots navigation info. When
//needed it is usually passed to functions as a pointer to avoid duplication.
//PositionGroup sys->pos. =


//Read data flag that is set by the external interrupt from the IMU on the V2 or by timer on the V1.
//Is defined in imu_interface.


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
uint8_t nfRetrieveNavData(RobotGlobalStructure *sys)
{
	if(sys->flags.imuCheckFifo)
	{
		if(sys->pos.IMU.pollEnabled)				//If polling enabled for IMU
		{
			if(!sys->pos.IMU.dmpEnabled)			//If DMP was disabled then enable it
				nfDMPEnable(1, sys);
			imuReadFifo(sys);						//Read IMU's FIFO buffer
			nfGetEulerAngles(sys);					//Convert IMU quaternions to Euler angles
		} else {
			if(sys->pos.IMU.dmpEnabled)				//If polling IMU disabled, then Disable DMP
				nfDMPEnable(0, sys);
		}
		sys->flags.imuCheckFifo = 0;				//Reset interrupt flag

		if(sys->pos.Optical.pollEnabled)
		{
			getMouseXY(sys);						//Update mouse sensor data
			sys->pos.Optical.x += sys->pos.Optical.dx;
			sys->pos.Optical.y += sys->pos.Optical.dy;
			nfProcessOpticalData(sys);
		}
		return 0;
	} else
		return 1;
}

/*
* Function: void nfGetEulerAngles(RobotGlobalStructure *sys)
*
* Convert Quaternion numbers from the IMU to Euler rotational angles
*
* Inputs:
* RobotGlobalStructure *sys
*   Holds the address to the global sys->pos. structure that holds all positional data
*
* Returns:
* Loads Yaw, Pitch and Roll data back into sys->pos..
*
* Implementation:
* After the quaternions have been converted to Euler angles, the Yaw offset is applied which is a
* heading correction obtained from the PC. Once this has been applied, the Yaw value is checked to
* ensure it is still in range (-180<Yaw<180) and corrected if necessary.
*
*/
void nfGetEulerAngles(RobotGlobalStructure *sys)
{
	float w = sys->pos.IMU.qw;				//Pull quaternions from IMU
	float x = sys->pos.IMU.qx;
	float y = sys->pos.IMU.qy;
	float z = sys->pos.IMU.qz;
	float sqw = w*w;						//Pre-calculate squares
	float sqx = x*x;
	float sqy = y*y;
	float sqz = z*z;
	float unit = sqx + sqy + sqz + sqw;	//Should equal 1, otherwise is correction factor
	float test = x*y + z*w;
	if (test > 0.499*unit)					// singularity at north pole
	{
		sys->pos.IMU.roll = 2 * atan2(x,w);
		sys->pos.IMU.pitch = M_PI/2;
		sys->pos.IMU.yaw = 0;
		return;
	}
	if (test < -0.499*unit)					// singularity at south pole
	{
		sys->pos.IMU.roll = -2 * atan2(x,w);
		sys->pos.IMU.pitch = M_PI/2;
		sys->pos.IMU.yaw = 0;
		return;
	}
	sys->pos.IMU.roll = (atan2(2*y*w-2*x*z , sqx - sqy - sqz + sqw))*180/M_PI;
	sys->pos.IMU.pitch = (asin(2*test/unit))*180/M_PI;
	sys->pos.IMU.yaw = (atan2(2*x*w-2*y*z , -sqx + sqy - sqz + sqw))*180/M_PI;
	//Factor in the Yaw offset (Heading correction from the PC) and store in pos.facing
	sys->pos.facing = sys->pos.IMU.yaw + sys->pos.facingOffset;
	//Wrap facing so its always between -180 and 180 degrees
	sys->pos.facing = nfWrapAngle(sys->pos.facing);
}

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
* Implementation:
* TODO:implementation
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void nfProcessOpticalData(RobotGlobalStructure *sys)
{
	//Calculate headings (deg) if there is fresh data from the optical sensor
	if(abs(sys->pos.Optical.dx) > 0 || abs(sys->pos.Optical.dy) > 0)
	{
		//Relative heading
		sys->pos.relHeading = atan2(sys->pos.Optical.dx, sys->pos.Optical.dy)*180/M_PI;
		//Absolute heading
		sys->pos.heading = sys->pos.facing + sys->pos.relHeading;
		//Wrap to within -180 to 180 deg
		sys->pos.heading = nfWrapAngle(sys->pos.heading);
	}
	
	//Calculate dx and dy in mm
	sys->pos.dx = round((sys->pos.Optical.dx)*sys->pos.Optical.convFactorX);
	sys->pos.dy = round((sys->pos.Optical.dy)*sys->pos.Optical.convFactorY);
	
	//Integrate absolute x and y in mm
	sys->pos.x += sys->pos.dx;
	sys->pos.y += sys->pos.dy;
	
	//Calculate speed from optical sensor (mm/s)
	sys->pos.speed = sqrt((sys->pos.dx*sys->pos.dx + sys->pos.dy*sys->pos.dy))
						/sys->pos.deltaTime*1000;
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

/*
* Function:
* void nfDMPEnable(RobotGlobalStructure *sys)
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
* Implementation:
* Calls the DMP enable function in the IMU driver, and sets dmpEnabled high
*
*/
void nfDMPEnable(char enable, RobotGlobalStructure *sys)
{
	if(enable)
		imuDmpStart();
	else
		imuDmpStop();
	sys->pos.IMU.dmpEnabled = enable;
}

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
* Implementation:
* Three 16bit unsigned integers representing the position of the robot in the arena in mm are sent
* The first is x, the second is y and the third is a facing in degrees. The coordinate system of 
* the web cam has the origin in the top left corner. On the robot however the origin would be the
* bottom left corner. To bandage this, the y value is converted to negative (and the robot thinks
* its working in the lower right quadrant)
*
* Improvements:
* Find a better way to handle coordinated between the robot and PC
*
*/
void nfApplyPositionUpdateFromPC(uint8_t *rawData, RobotGlobalStructure *sys)
{
	float distMouseX = 0;
	float distMouseY = 0;
	float distPCX = 0;
	float distPCY = 0;

	led1Tog;
	//Update position (x and y swapped to convert coord systems. Temporary fix)
	sys->pos.y = (uint16_t)((rawData[0]<<8)|rawData[1]);
	sys->pos.x = (uint16_t)((rawData[2]<<8)|rawData[3]);
	//Update facing (only if robot isn't rotating too fast)
	if(abs(sys->pos.IMU.gyroZ) < 10)
		imuApplyYawCorrection((int16_t)((rawData[4]<<8)|rawData[5]), sys);
	
	if(sys->pos.oldPCX != 0 && sys->pos.oldPCY != 0)
	{
		//Distance travelled between updates in mouse counts
		distMouseX = sys->pos.Optical.x - sys->pos.Optical.xOld;
		distMouseY = sys->pos.Optical.y - sys->pos.Optical.yOld;
	
		//Distance travelled between updates in mm
		distPCX = sys->pos.x - sys->pos.oldPCX;
		distPCY = sys->pos.y - sys->pos.oldPCY;
	}
	
	if((abs(distPCX) > 80 || abs(distPCY) > 80) && (distMouseX || distMouseY))	
	{
		sys->pos.Optical.convFactorX += (distPCX/distMouseX);
		sys->pos.Optical.convFactorY += (distPCY/distMouseY);
		sys->pos.Optical.convFactorX /= 2;
		sys->pos.Optical.convFactorY /= 2;
	}
	
	sys->pos.oldPCX = sys->pos.x;
	sys->pos.oldPCY = sys->pos.y;
	
	sys->pos.Optical.xOld = sys->pos.Optical.x;
	sys->pos.Optical.yOld = sys->pos.Optical.y;
}

uint8_t nfOpticalTesting(uint8_t speed, uint8_t distance, RobotGlobalStructure *sys)
{
	static uint8_t state = 0;
		
	switch(state)
	{
		case 1:
			if(!mfMoveToHeadingByDistance(0, speed, distance, sys))
			{
				state = 2;
			}
			break;
			
		case 2:
			if(!fdelay_ms(2000))
				state = 3;
			break;
			
		case 3:
			if(!mfMoveToHeadingByDistance(180, speed, distance, sys))
			{
				state = 4;	
			}
		
			break;
			
		case 4:
				if(!fdelay_ms(2000))
					state = 0;
			break;
			
		case 0:
			state = 1;
			break;
	}
	return state;
}