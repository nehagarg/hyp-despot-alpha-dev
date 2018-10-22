#include "custom_particle_belief.h"
#include <despot/core/particle_belief.h>
#include "WorldModel.h"
#include "simulator.h"
#include "ped_pomdp.h"
#include <iostream>

static WorldStateTracker stateTracker (Simulator::worldModel);
static WorldBeliefTracker beliefTracker(Simulator::worldModel, stateTracker);
static PedStruct sorted_peds_list[ModelParams::N_PED_WORLD];
static PedStruct reordered_peds_list[ModelParams::N_PED_WORLD];


MaxLikelihoodScenario::MaxLikelihoodScenario(vector<State*> particles, const DSPOMDP* model,
		Belief* prior, bool split): ParticleBelief(particles, model,
		prior, split){

}

vector<State*> MaxLikelihoodScenario::SampleCustomScenarios(int num, vector<State*> particles,
	const DSPOMDP* model) const{
	//return particles;
	vector<State*> sample;

	cout<<"++++++++=========++++++++"<<endl;

	double goal_count[10][10]={{0}};
	if(particles.size() == 0)
		return sample;
	

	for(int i = 0; i < particles.size(); i ++) {
		const PomdpState* pomdp_state=static_cast<const PomdpState*>(particles.at(i));
		for(int j = 0; j < pomdp_state->num; j ++) {
			goal_count[j][pomdp_state->peds[j].goal] += particles[i]->weight;
		}
	}

	const PomdpState* pomdp_state=static_cast<const PomdpState*>(particles.at(0));
	for(int j = 0; j < pomdp_state->num; j ++) {
		cout << "Ped " << pomdp_state->peds[j].id << " Belief is ";
		for(int i = 0; i < 7; i ++) {
			cout << (goal_count[j][i] + 0.0) <<" ";
		}
		cout << endl;
	}
	
	PomdpState* particle = static_cast<PomdpState*>(model->Copy(pomdp_state));

	for(int j = 0; j < pomdp_state->num; j ++) {
		int max_likelihood_goal = -1;
		double max_likelihood = -1;
		for(int i = 0; i < 7; i ++) {
			if(goal_count[j][i] > max_likelihood) {
				max_likelihood = goal_count[j][i];
				max_likelihood_goal = i;
			}
		}
		cout<<"** "<<particle->peds[j].id<<"   goal: "<<max_likelihood_goal;
		particle->peds[j].goal = max_likelihood_goal;
		cout << endl;
	}

	particle -> weight = 1.0;
	sample.push_back(particle);
	
	return sample;
}

PedStruct NewDummyPed(int pid){
	return PedStruct(COORD(dummy_pos_value, dummy_pos_value), -1, pid);
}

PedPomdpBelief::PedPomdpBelief(vector<State*> particles, const DSPOMDP* model):
	ParticleBelief( particles, model),
	world_model_(Simulator::worldModel){
}

Belief* PedPomdpBelief::MakeCopy() const{
	std::vector<State*> particles = particles_;
	return new PedPomdpBelief(particles, static_cast<const DSPOMDP*>(model_));
}


void UpdateState(const PomdpStateWorld* src_world_state, WorldModel& world_model){

	stateTracker.removePeds();
	stateTracker.updateCar(world_model.path[src_world_state->car.pos], src_world_state->car.dist_travelled);
	stateTracker.updateVel(src_world_state->car.vel);

	cout << "[UpdateState] \n";
	//update the peds in stateTracker
	for(int i=0; i<src_world_state->num; i++) {
		Pedestrian p(src_world_state->peds[i].pos.x, src_world_state->peds[i].pos.y, src_world_state->peds[i].id);
		stateTracker.updatePed(p, false); // true: print debug info
	}
}

void SortPeds(PomdpState* sorted_search_state, const PomdpStateWorld* src_world_state){
	std::vector<WorldStateTracker::PedDistPair> sorted_peds = stateTracker.getSortedPeds();

	//update s.peds to the nearest n_peds peds
	sorted_search_state->num=min(src_world_state->num, ModelParams::N_PED_IN);
	for(int pid=0; pid<sorted_search_state->num; pid++) {
		if(pid<sorted_peds.size())
		{
			int pedid=sorted_peds[pid].second.id;
			// Search for the ped in src_world_state
			int i=0;
			for(;i<src_world_state->num;i++) {
				if (pedid==src_world_state->peds[i].id) {
					//found the corresponding ped, record the order
					sorted_search_state->peds[pid]=src_world_state->peds[i];
					break;
				}
			}
			if(i==src_world_state->num)//ped not found
			{
				sorted_search_state->peds[pid]=NewDummyPed(-1);
			}
		}
	}
}

void ReorderPeds(PomdpState* target_search_state,
		const PomdpState* ref_search_state,
		const PomdpStateWorld* src_history_state){
	//update s.peds to the nearest n_peds peds
	target_search_state->num=ref_search_state->num;
	for(int pid=0; pid<ref_search_state->num; pid++) {
		int pedid=ref_search_state->peds[pid].id;
		//Search for the ped in world_state
		int i=0;
		for(;i<src_history_state->num;i++) {
			if (pedid==src_history_state->peds[i].id) {
				//found the corresponding ped, record the order
				target_search_state->peds[pid]=src_history_state->peds[i];
				break;
			}
		}
		if (i >= src_history_state->num) {
			//not found, new ped
			target_search_state->peds[pid]=NewDummyPed(-1);
		}
	}
}


bool PedPomdpBelief::DeepUpdate(const std::vector<const State*>& state_history,
		std::vector<State*>& state_history_for_search,
		const State* cur_state,
		State* cur_state_for_search,
		ACT_TYPE action){
	//Update current belief
	const PomdpStateWorld* cur_world_state = static_cast<const PomdpStateWorld*>(cur_state);
	PomdpState* cur_state_search = static_cast<PomdpState*>(cur_state_for_search);

	//Sort pedestrians in the current state and update the current search state
	UpdateState(cur_world_state, world_model_);
	//SortPeds(cur_state_search,cur_world_state);

	for (int hi=0;hi< state_history.size(); hi++){
	   const PomdpStateWorld* hist_state =
			   static_cast<const PomdpStateWorld*>(state_history[hi]);
	   PomdpState* hist_state_search =
			   static_cast<PomdpState*>(state_history_for_search[hi]);

	   ReorderPeds(hist_state_search, cur_state_search, hist_state);
	}


	//Update and reorder the belief distributions for peds
	beliefTracker.update();

	if (Globals::config.silence == false && DESPOT::Debug_mode)
		beliefTracker.printBelief();
}

bool PedPomdpBelief::DeepUpdate(const State* cur_state){

	if (cur_state == NULL)
		return false;

	//Update current belief
	const PomdpStateWorld* cur_world_state = static_cast<const PomdpStateWorld*>(cur_state);

	//Sort pedestrians in the current state and update the current search state
	UpdateState(cur_world_state, world_model_);

	//Update and reorder the belief distributions for peds
	beliefTracker.update();

	return true;
}

void PedPomdpBelief::ResampleParticles(const PedPomdp* model){
	vector<PomdpState> samples = beliefTracker.sample(max(2000,5*Globals::config.num_scenarios));

	vector<State*> particles = model->ConstructParticles(samples);

	if(DESPOT::Debug_mode)
		std::srand(0);


	for (int i = 0; i < particles_.size(); i++)
					model_->Free(particles_[i]);

	particles_ = particles;

	if (fabs(State::Weight(particles) - 1.0) > 1e-6) {
			loge << "[PedPomdpBelief::PedPomdpBelief] Particle weights sum to " << State::Weight(particles) << " instead of 1" << endl;
			exit(1);
		}

	bool split = true;
	if (split) {
		// Maintain more particles to avoid degeneracy
		while (2 * num_particles_ < 5000)
			num_particles_ *= 2;
		if (particles_.size() < num_particles_) {
			logi << "[PedPomdpBelief::PedPomdpBelief] Splitting " << particles_.size()
				<< " particles into " << num_particles_ << " particles." << endl;
			vector<State*> new_particles;
			int n = num_particles_ / particles_.size();
			for (int i = 0; i < n; i++) {
				for (int j = 0; j < particles_.size(); j++) {
					State* particle = particles_[j];
					State* copy = model_->Copy(particle);
					copy->weight /= n;
					new_particles.push_back(copy);
				}
			}

			for (int i = 0; i < particles_.size(); i++)
				model_->Free(particles_[i]);

			particles_ = new_particles;
		}
	}

	if (fabs(State::Weight(particles) - 1.0) > 1e-6) {
		loge << "[PedPomdpBelief::PedPomdpBelief] Particle weights sum to " << State::Weight(particles) << " instead of 1" << endl;
		exit(1);
	}

	random_shuffle(particles_.begin(), particles_.end());
	// cerr << "Number of particles in initial belief: " << particles_.size() << endl;

	if (prior_ == NULL) {
		for (int i = 0; i < particles.size(); i++)
			initial_particles_.push_back(model_->Copy(particles[i]));
	}
}


void PedPomdpBelief::Update(ACT_TYPE action, OBS_TYPE obs) {
	logi << "[PedPomdpBelief::Update] Doing nothing " << endl;

	return ;
}
