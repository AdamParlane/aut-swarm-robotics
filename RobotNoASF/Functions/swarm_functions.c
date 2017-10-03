/*
 * swarm_functions.c
 *
 * Created: 3/10/2017 10:08:38 AM
 *  Author: adams
 */ 

#include "../robot_setup.h"

#include "../Interfaces/twimux_interface.h"
#include "../Interfaces/motor_driver.h"
#include "../Interfaces/prox_sens_interface.h"
#include "../Interfaces/timer_interface.h"
#include "../Functions/motion_functions.h"
#include "obstacle_avoidance.h"
#include "swarm.functions.h"

//basic follow the leader function essentially doing the opposite of obstacle avoidance
//note: CAN NOT tell difference between a leader and any other object so will hit walls etc
void followTheLeader(RobotGlobalStructure *sys)
{
	scanProximity(sys);// updates proximity sensors
	static float facing = 0;
	static uint8_t count = 0;
	float left = (float)proximity[5];
	float front = (float)proximity[0];
	float right = (float)proximity[1];
	if(sys->flags.obaEnabled)
	{
		facing = sys->pos.facing;
		sys->flags.obaEnabled = 0;
	}
	if(abs(mfMoveToHeading(facing, sys->pos.targetSpeed, sys)) < 10)
	{
		if(front > OBSTACLE_THRESHOLD)
		{
			if(left > OBSTACLE_THRESHOLD && right < OBSTACLE_THRESHOLD)//left slightly
			{
				facing += ((front/(front + left))-1) * 45;
			}
			else if (right > OBSTACLE_THRESHOLD && left < OBSTACLE_THRESHOLD)//right slightly
			{
				facing += (1-(front/(front + right))) * 45;
			}
		}
		else if((left > OBSTACLE_THRESHOLD) && (front < OBSTACLE_THRESHOLD))
			facing += (45 * left/2047.0) + 22;
		else if((right > OBSTACLE_THRESHOLD) && (front < OBSTACLE_THRESHOLD))
			facing -=  (45 * right/2047.0) + 22;
		else if((front < 50) && (right < 50) && (left < 50))
		{
			count++;
			if(count > 5)
			{
				facing += 90; // search for target
				count = 0;
			}
		}
	}
}