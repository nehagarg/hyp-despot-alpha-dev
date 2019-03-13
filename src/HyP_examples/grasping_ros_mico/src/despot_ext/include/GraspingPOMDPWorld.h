/*
 * GraspingPOMDPWorld.h
 *
 *  Created on: Mar 13, 2019
 *      Author: neha
 */

#ifndef GRASPINGPOMDPWORLD_H_
#define GRASPINGPOMDPWORLD_H_

#include <despot/core/pomdp_world.h>

namespace despot {

class GraspingPOMDPWorld: public POMDPWorld {
public:
	GraspingPOMDPWorld(DSPOMDP* model, unsigned seed);
	virtual ~GraspingPOMDPWorld();
	bool ExecuteAction(ACT_TYPE action, OBS_TYPE& obs);
};

} /* namespace despot */

#endif /* GRASPINGPOMDPWORLD_H_ */
