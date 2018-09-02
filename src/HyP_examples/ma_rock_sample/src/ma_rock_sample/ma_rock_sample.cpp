#include "ma_rock_sample.h"
#include <string>
#include <bitset>

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

	/*for(int i=0;i<10;i++){

		int test_act=Random::RANDOM.NextInt(NumActions());
		cout <<"test act:"<<test_act<<endl;

		int ACT_0=GetRobAction(test_act, 0);
		int ACT_1=GetRobAction(test_act, 1);

		assert(ACT_0<RobNumAction());
		assert(ACT_1<RobNumAction());
		cout <<"0 after decoding:"<<ACT_0<<endl;
		cout <<"1 after decoding:"<<ACT_1<<endl;
		cout<<"===================="<<endl;
	}*/

	for(int i=0;i</*1000*/0;i++){

		MARockSampleState particle;
		particle.SetAllocated();
		particle.state_id=0;
		int test_coord_0;
		int test_coord_1;

		if(particle.state_id>=NumStates() || particle.allocated_>1){
			cout<<"!!!!!!!!!!Wrong init particle!!!!!!!";
			cout <<"Initial particle :"<<std::bitset<8>(particle.allocated_)<<" "
				<<std::bitset<32>(particle.state_id)<<std::endl;
		}

		//test_coord_1=Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
		test_coord_1=(Random::RANDOM.NextDouble()>0.5)?ROB_TERMINAL_ID:Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
		SetRobPosIndex(particle.joint_pos, 1, test_coord_1);
		if(particle.joint_pos>=NumPosStates() || particle.allocated_>1){
			cout<<"!!!!!!!!!!Wrong init particle!!!!!!!";
			cout <<"Initial particle r0:"<<std::bitset<8>(particle.allocated_)<<" "
				<<std::bitset<32>(particle.joint_pos)<<std::endl;
		}

		test_coord_0=(Random::RANDOM.NextDouble()>0.5)?ROB_TERMINAL_ID:Random::RANDOM.NextInt(grid_.xsize()*grid_.ysize()-1);
		//test_coord_0=ROB_TERMINAL_ID;

		SetRobPosIndex(particle.joint_pos, 0, test_coord_0);
		cout <<"Initial particle r0+r1:"<<std::bitset<8>(particle.allocated_)<<" "
				<<std::bitset<32>(particle.joint_pos)<<std::endl;
		if(particle.joint_pos>=NumPosStates() || particle.allocated_>1){
			cout<<"!!!!!!!!!!Wrong init particle!!!!!!!";

		}

		vector<State*> states;states.resize(100001);

		states[0]=Copy(&particle);

		int last_step=0;
		for(int step=1;step<100001;step++){
			states[step]=Copy(states[step-1]);

			if(states[step]->state_id>=NumStates() || states[step]->allocated_>1){
				cout<<"!!!!!!!!!!Wrong copied result!!!!!!!";
				cout <<"Before:"<<std::bitset<8>(states[step]->allocated_)<<" "<<std::bitset<32>(states[step]->state_id)<<std::endl;
			}
			OBS_TYPE obs;
			double reward;
			int action=Random::RANDOM.NextInt(NumActions());
			bool terminal=Step(*states[step],Random::RANDOM.NextDouble(), action, reward, obs);
			if(states[step]->state_id>=NumStates() || states[step]->allocated_>1){
				cout<<"!!!!!!!!!!Wrong encoding result!!!!!!!";
				cout <<"After:"<<std::bitset<8>(states[step]->allocated_)<<" "<<std::bitset<32>(states[step]->state_id)<<std::endl;
			}
			last_step=step;
			if(terminal)
				Free(states[step]);
				states[step]=Copy(&particle);
				continue;

		}
		for(int step=0;step<last_step+1;step++){
			Free(states[step]);
		}
	}
}


bool MultiAgentRockSample::Step(State& state, double rand_num, int action, double& reward,
	OBS_TYPE& obs) const {
	MARockSampleState& rockstate = static_cast<MARockSampleState&>(state);
	reward = 0;
	//obs = E_NONE;
	bool isterminal=true;


	obs=0;
	//Update each of the robot
	for(int i=0;i<num_agents_;i++)
	{
		SetRobObs(obs, E_NONE,i);
		//int x=0; int y=0;
		//x= (int)((rand_num)*(grid_.xsize()-1));
		//y= (int)((1-rand_num)*(grid_.ysize()-1));
		//std::cout<<bitset<32>(y)<<std::endl;
		//SetRobPosIndex(rockstate.joint_pos, i, y*grid_.ysize()+x);//debugging
		//reward=/*rockstate.joint_pos*1.0f*/1;
		//isterminal=false;

		//int agent_action=GetRobAction(action, i);

		if(GetRobPosIndex(&rockstate, i)!=ROB_TERMINAL_ID){
			int agent_action=GetRobAction(action, i);
			//agent_action=Compass::EAST;//debugging
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

				if (rand_num < efficiency)
					rob_obs= GetRock(&rockstate, rock) & E_GOOD;
				else
					rob_obs= !(GetRock(&rockstate, rock) & E_GOOD);
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
			else if (rob_obs != E_GOOD && rob_obs != E_BAD)
				prob *=0;
			else{
				int rock = agent_action - E_SAMPLE - 1;
				double distance = Coord::EuclideanDistance(GetRobPos(&rockstate, i),
					rock_pos_[rock]);
				double efficiency = (1 + pow(2, -distance / half_efficiency_distance_))
					* 0.5;

				prob*=
					((GetRock(&rockstate, rock) & 1) == rob_obs) ? efficiency : (1 - efficiency);
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

		switch (agent_observation) {
		case E_NONE:
			out << "None" /*<< endl*/;
			break;
		case E_GOOD:
			out << "Good" /*<< endl*/;
			break;
		case E_BAD:
			out << "Bad" /*<< endl*/;
			break;
		}
	}
	out<<endl;
}

} // namespace despot
