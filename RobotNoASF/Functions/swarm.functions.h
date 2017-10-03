/*
 * swarm.h
 *
 * Created: 3/10/2017 10:08:53 AM
 *  Author: adams
 */ 


#ifndef SWARM_FUNCTIONS_H_
#define SWARM_FUNCTIONS_H_

//basic follow the leader function essentially doing the opposite of obstacle avoidance
//note: CAN NOT tell difference between a leader and any other object so will hit walls etc
void followTheLeader(RobotGlobalStructure *sys);


#endif /* SWARM.FUNCTIONS_H_ */