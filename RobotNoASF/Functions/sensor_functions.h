/*
* sensor_functions.h
*
* Author : Matthew Witt (pxf5695@autuni.ac.nz)
* Created: 21/09/2017 7:05:43 PM
*
* Project Repository: https://github.com/AdamParlane/aut-swarm-robotics
*
* Header file for sensor functions
*
* More Info:
* Atmel SAM 4N Processor Datasheet:http://www.atmel.com/Images/Atmel-11158-32-bit%20Cortex-M4-Microcontroller-SAM4N16-SAM4N8_Datasheet.pdf
* Relevant reference materials or datasheets if applicable
*
* Functions:
* void funcName(void)
*
*/

#ifndef SENSOR_FUNCTIONS_H_
#define SENSOR_FUNCTIONS_H_

//////////////[Defines]/////////////////////////////////////////////////////////////////////////////
//Parameter definitions for lcfRetrieveLightData()
#define SF_RGB_ONLY		0
#define SF_RGB_AND_HSV		1

////Hue angle constants (Should help speed up maths)
#define SF_MAX_HUE_ANGLE	360
#define SF_HUE_ANGLE_DIV3	120
#define SF_HUE_ANGLE_DIV6	60

////Proximity sensor array elements
#define SF_PROX_FRONT		0
#define SF_PROX_FRONTL		1
#define SF_PROX_REARL		2
#define SF_PROX_REAR		3
#define SF_PROX_REARR		4
#define SF_PROX_FRONTR		5

//////////////[Functions]///////////////////////////////////////////////////////////////////////////
/*
* Function:
* void sfPollSensors(RobotGlobalStructure *sys)
*
* Polls sensors at the desired intervals
*
* Inputs:
* [input arguments and any relevant explanation]
*
* Returns:
* [return values and any relevant explanation]
*
* Implementation:
* [explain key steps of function]
* [use heavy detail for anything complicated]
* Template c file function header. H file function header will be the same without the
* implementation/improvement section
*
* Improvements:
* [Ideas for improvements that are yet to be made](optional)
*
*/
void sfPollSensors(RobotGlobalStructure *sys);

/*
* Function:
* void sfGetProxSensorData(RobotGlobalStructure *sys)
*
* Retrieves data from the proximity sensors and stores it in the global data structure
*
* Inputs:
* RobotGlobalStructure *sys
* Pointer to the robot global data structure
*
* Returns:
* none
*
*/
void sfGetProxSensorData(RobotGlobalStructure *sys);

/*
* Function:
* uint8_t sfUpdateLineSensorStates(RobotGlobalStructure *sys)
*
* Sees if any sensors have made a definite state change and loads the states into the line sensor
* state structure for use by other functions in this module.
*
* Inputs:
* none
*
* Returns:
* 1 if line state change detected, otherwise 0
*
*/
uint8_t sfUpdateLineSensorStates(RobotGlobalStructure *sys);

/*
* Function:
* void sfGetLineDirection(RobotGlobalStructure *sys)
*
* This function examines the states of the line follower sensors and determines the direction and
* urgency factor by which the robot should move to find its way to the centre of the line.
*
* Inputs:
* none
*
* Returns:
* returns a signed integer between -3 and 3 that determines the direction and speed magnitude that
* the robot should move to find the centre of the line.
* A negative output means that the robot should move left to find the line and a positive output
* means that the robot should move right. 0 means keep going straight because no direction data is
* able to be derived from sensor array.
*
*/
void sfGetLineDirection(RobotGlobalStructure *sys);

/*
* Function:
* uint8_t sfLightCapture(uint8_t channel, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *w)
*
* Retrieves the (16-bit) light data of all colours from the selected Light Sensor
*
* Inputs:
* uint8_t channel:
*   The I2C mulitplexer channel of the light sensor to read from.
*   MUX_LIGHTSENS_R for the right sensor or MUX_LIGHTSENS_L for the left
* uint16_t *r
*   Pointer to a 16bit integer to store red channel data
* uint16_t *g
*   Pointer to a 16bit integer to store green channel data
* uint16_t *b
*   Pointer to a 16bit integer to store blue channel data
* uint16_t *w
*   Pointer to a 16bit integer to store white channel data
*
* Returns:
* 0 on success, or non-zero when TWI error occurred.
*
*/
uint8_t sfLightCapture(uint8_t channel, ColourSensorData *colours);

/*
* Function:
* void sfRGB2HSV(struct ColourSensorData *colours)
*
* Converts RGB to HSV and stores them in a ColourSensorData structure
*
* Inputs:
* struct ColourSensorData *colours
*   Pointer to a ColourSensorData structure to store the calculated HSV values
*
* Returns:
* none
*
*/
void sfRGB2HSV(ColourSensorData *colours);

#endif /* SENSOR_FUNCTIONS_H_ */