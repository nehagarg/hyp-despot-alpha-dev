/*
 * GraspingPOMDPWorld.cpp
 *
 *  Created on: Mar 13, 2019
 *      Author: neha
 */

#include "GraspingPOMDPWorld.h"
#include "LearningModel.h"

namespace despot {

GraspingPOMDPWorld::GraspingPOMDPWorld(DSPOMDP* model, unsigned seed) : POMDPWorld(model,seed) {
	// TODO Auto-generated constructor stub

}

GraspingPOMDPWorld::~GraspingPOMDPWorld() {
	// TODO Auto-generated destructor stub
}

bool GraspingPOMDPWorld::ExecuteAction(ACT_TYPE action, OBS_TYPE& obs) {
		bool terminal = ((LearningModel*)model_)->StepActual(*state_, random_.NextDouble(), action,
				step_reward_, obs);
		return terminal;
	}

} /* namespace despot */

