#ifndef CUSTOM_PARTICLE_BELIEF_H
#define CUSTOM_PARTICLE_BELIEF_H

#include "state.h"
#include <despot/interface/belief.h>
#include <despot/core/particle_belief.h>

#include <limits>

class MaxLikelihoodScenario: public ParticleBelief{
public:
	MaxLikelihoodScenario(vector<State*> particles, const DSPOMDP* model,
		Belief* prior = NULL, bool split = true);

	vector<State*> SampleCustomScenarios(int num, vector<State*> particles,
	const DSPOMDP* model) const;

	virtual void Debug() const{}
};

class WorldModel;
class PedPomdp;
class PedPomdpBelief: public ParticleBelief{
public:

	PedPomdpBelief(vector<State*> particles,const DSPOMDP* model);

	virtual Belief* MakeCopy() const;
	/*Update function to be used in SOlver::BeliefUpdate*/
	virtual bool DeepUpdate(const std::vector<const State*>& state_history,
			std::vector<State*>& state_history_for_search,
			const State* cur_state,
			State* cur_state_for_search,
			ACT_TYPE action);
	virtual bool DeepUpdate(const State* cur_state);

	void ResampleParticles(const PedPomdp* model);

	virtual void Update(despot::ACT_TYPE, despot::OBS_TYPE);

	//long double TransProb(const State* state1, const State* state2, ACT_TYPE action) const;
	WorldModel& world_model_;


public:     

	//inline int GetThreadID() const{return 0/*MapThread(this_thread::get_id())*/;}
};

const double dummy_pos_value=std::numeric_limits<float>::max();

#endif
