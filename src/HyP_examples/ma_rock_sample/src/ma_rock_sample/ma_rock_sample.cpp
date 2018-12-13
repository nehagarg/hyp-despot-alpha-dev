#include "ma_rock_sample.h"
#include <string>
#include <bitset>
#include "GPU_base_ma_rock_sample.h"
using namespace std;

namespace despot {

/* =============================================================================
 * MultiAgentRockSample class
 * =============================================================================*/

MultiAgentRockSample::MultiAgentRockSample(string map) :
	BaseMultiAgentRockSample(map) {
	half_efficiency_distance_ = 20;
}

MultiAgentRockSample::MultiAgentRockSample(int size, int rocks) :
	BaseMultiAgentRockSample(size, rocks) {
	half_efficiency_distance_ = 20;

}


bool MultiAgentRockSample::Step(State& state, double rand_num, int action, double& reward,
	OBS_TYPE& obs) const {
	MARockSampleState& rockstate = static_cast<MARockSampleState&>(state);
	reward = 0;
	//obs = E_NONE;
	bool isterminal=true;

	obs=0;
	if (Globals::config.use_multi_thread_){
		QuickRandom::SetSeed(INIT_QUICKRANDSEED, Globals::MapThread(this_thread::get_id()));
	}
	else{
		QuickRandom::SetSeed(INIT_QUICKRANDSEED, 0);
	}
	//Update each of the robot
	for(int i=0;i<num_agents_;i++)
	{
		SetRobObs(obs, E_NONE,i);

		if(GetRobPosIndex(&rockstate, i)!=ROB_TERMINAL_ID){
			int agent_action=GetRobAction(action, i);
			if (agent_action < E_SAMPLE) { // Move
				switch (agent_action) {
				case Compass::EAST:
					if (GetX(&rockstate, i) + 1 < size_) {
						IncX(&rockstate, i);
					} else {
						reward += +10;
						SetRobPosIndex(rockstate.joint_pos, i, ROB_TERMINAL_ID);
					}
					break;

				case Compass::NORTH:
					if (GetY(&rockstate, i) + 1 < size_)
						IncY(&rockstate, i);
					else
						reward += -100;
					break;

				case Compass::SOUTH:
					if (GetY(&rockstate, i) - 1 >= 0)
						DecY(&rockstate, i);
					else
						reward += -100;
					break;

				case Compass::WEST:
					if (GetX(&rockstate, i) - 1 >= 0)
						DecX(&rockstate, i);
					else
						reward += -100;
					break;
				}
			}

			if (agent_action == E_SAMPLE) { // Sample
				int rock = grid_(GetRobPosIndex(&rockstate, i));
				if (rock >= 0) {
					if (GetRock(&rockstate, rock))
						reward += +10;
					else
						reward += -10;
					SampleRock(&rockstate, rock);
				} else {
					reward += -100;
				}
			}

			if (agent_action > E_SAMPLE) {//debugging
				int rob_obs=0;// (int)(rand_num*3);
				int rock = agent_action - E_SAMPLE - 1;
				double distance = Coord::EuclideanDistance(GetRobPos(&rockstate, i),
					rock_pos_[rock]);
				double efficiency = (1 + pow(2, -distance / half_efficiency_distance_))
					* 0.5;
				//double efficiency=0.5;

				for(int j = 0; j < num_obs_bits; j++)
				{int temp_rob_obs;

					if (rand_num < efficiency)
						temp_rob_obs= GetRock(&rockstate, rock) & E_GOOD;
					else
						temp_rob_obs= !(GetRock(&rockstate, rock) & E_GOOD);
					rob_obs = (2*rob_obs + temp_rob_obs);
					rand_num=QuickRandom::RandGeneration(rand_num);
				}
				rob_obs = 4*rob_obs;
				SetRobObs(obs, rob_obs, i);
			}

			if (GetRobPosIndex(&rockstate, i)!=ROB_TERMINAL_ID) {
				isterminal=false;
			}
		}
	}

	return isterminal;
}

int MultiAgentRockSample::NumActions() const {
	return //num_agents_*(num_rocks_ + 5);
			//1 << num_agents_*MAX_ACTION_BIT;
			std::pow((float)RobNumAction(), num_agents_);
}

double MultiAgentRockSample::ObsProb(OBS_TYPE obs, const State& state, int action) const {
	float prob=1;
	//calculate prob for each robot, multiply them together
	for(int i=0;i<num_agents_;i++){
		int agent_action=GetRobAction(action, i);
		int rob_obs=GetRobObs(obs, i);
		const MARockSampleState& rockstate =
			static_cast<const MARockSampleState&>(state);
		if(GetRobPosIndex(&rockstate, i)!=ROB_TERMINAL_ID){
			if (agent_action <= E_SAMPLE)
				prob *= (rob_obs == E_NONE);
			else if (rob_obs < 4) //Last 2 bits for E_NONE
				prob *=0;
			else{
				int rock = agent_action - E_SAMPLE - 1;
				double distance = Coord::EuclideanDistance(GetRobPos(&rockstate, i),
					rock_pos_[rock]);
				double efficiency = (1 + pow(2, -distance / half_efficiency_distance_))
					* 0.5;
				int true_state = (GetRock(&rockstate, rock) & 1)
				for(int j = 0; j < num_obs_bits; j++)
				{
					int my_rob_obs = (rob_obs >> (2+j)) & 1;
					prob*= ( true_state== my_rob_obs) ? efficiency : (1 - efficiency);
					if(j % 8 == 0)
					{
						prob = prob*1000; //Multiply by a constant to avoid prob becoming 0
					}
				}
			}
		}
	}
	return prob;
}

void MultiAgentRockSample::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	for(int i=0;i<num_agents_;i++){
		int agent_observation=GetRobObs(observation, i);
		out << " Agent "<<i << ": ";

		if(agent_observation == E_NONE)
		{
			out << "None";
		}
		else
		{
			for(int j = 0; j < num_obs_bits; j++)
			{
				int my_rob_obs = (agent_observation >> (2+j)) & 1;
				if(my_rob_obs == E_GOOD)
				{
					out << "G ";
				}
				if(my_rob_obs == E_BAD)
				{
					out << "B ";
				}
			}
		}

	}
	out<<endl;
}

Dvc_State* MultiAgentRockSample::GetPointerToParticleList(int offset,  Dvc_State* full_list) const
{
	return static_cast<Dvc_MARockSampleState*>(full_list)+ offset;
}

} // namespace despot
