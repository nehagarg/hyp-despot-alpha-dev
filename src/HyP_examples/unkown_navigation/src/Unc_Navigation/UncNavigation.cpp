#include "UncNavigation.h"
#include <despot/GPUcore/thread_globals.h>
#include <despot/solver/despot.h>
#include "GPU_base_unc_navigation.h"
using namespace std;

namespace despot {

/* =============================================================================
 * UncNavigation class
 * =============================================================================*/

/*UncNavigation::UncNavigation(string map) :
	BaseUncNavigation(map) {
	half_efficiency_distance_ = 20;
}*/



UncNavigation::UncNavigation(int size, int obstacles) :
	BaseUncNavigation(size, obstacles) {
	cout<<__FUNCTION__<<endl;

	half_efficiency_distance_ = 20;
}

bool UncNavigation::Step(State& state, double rand_num, int action, double& reward,
	OBS_TYPE& obs) const {
	UncNavigationState& nav_state = static_cast<UncNavigationState&>(state);
	reward = 0;

	if (nav_state.rob==nav_state.goal)
	{
		reward=0;
		return true;// terminal
	}
	reward=-0.1;// small cost for one step
	Coord rob_pos=nav_state.rob;

	/*if(CPUDoPrint && nav_state.scenario_id==CPUPrintPID)
	{
		printf("Before step scenario %d\n", nav_state.scenario_id);
		printf("rob at %d %d\n", rob_pos.x, rob_pos.y);
		printf("rand %f action %d\n",rand_num, action);
	}*/

	float prob=1.0f-STEP_NOISE;

	if (action < E_STAY) { // Move
		if(rand_num<prob)// only succeed with 80% chance
		{
			rob_pos += NavCompass::DIRECTIONS[action];
			/*if(CPUDoPrint && nav_state.scenario_id==CPUPrintPID)
			{
				printf("rob move\n");
				printf("new rob at %d %d\n", rob_pos.x, rob_pos.y);
			}*/
		}

		if (nav_state.Inside(rob_pos) && nav_state.CollisionCheck(rob_pos)==false) {
			nav_state.rob=rob_pos;//execute move
			/*if(CPUDoPrint && nav_state.scenario_id==CPUPrintPID)
			{
				printf("new pos inside map and collision free\n");
			}*/
			if (nav_state.rob==nav_state.goal)
			{
				reward=/*10*/GOAL_REWARD; // arrive goal
				/*if(CPUDoPrint && nav_state.scenario_id==CPUPrintPID)
				{
					printf("goal\n");
				}*/
			}
		} else {
			reward=-1;// cannot move
			/*if(CPUDoPrint && nav_state.scenario_id==CPUPrintPID)
			{
				printf("move fail\n");
			}*/
		}
	}

	if (action == E_STAY) { // Stay
		reward=-0.2;
	}
	//obs = GetObservation(rand_num,nav_state);//blocked for debugging
	obs=0;//Initialize obs

	OBS_TYPE obs_i=0;
	if (Globals::config.use_multi_thread_){
		QuickRandom::SetSeed(INIT_QUICKRANDSEED, Globals::MapThread(this_thread::get_id()));
	}
	else{
		QuickRandom::SetSeed(INIT_QUICKRANDSEED, 0);
	}

	for(int dir=0;dir<UncNavigation::num_obs_bits;dir++)
	{
		switch(dir)
		{
		case 3:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1);
			break;
		case 2:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y);
			break;
		case 1:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1);
			break;
		case 0:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y);
			break;
		case 4:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y+1);
			break;
		case 5:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y-1);
			break;
		case 6:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y-1);
			break;
		case 7:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y+1);
			break;
		case 11:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y+2):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y+2);
			break;
		case 10:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y);
			break;
		case 9:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y-2):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y-2);
			break;
		case 8:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y);
			break;
		case 12:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y+2):!nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y+2);
			break;
		case 13:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y-2):!nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y-2);
			break;
		case 14:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y-2):!nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y-2);
			break;
		case 15:
			rand_num=QuickRandom::RandGeneration(rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y+2):!nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y+2);
			break;
		}
		obs=(obs|(obs_i<<dir));

		//printf("thread.x=%d, rand_num=%f, obs=%d\n",threadIdx.x,rand_num, obs);
	}
	/*if(CPUDoPrint && nav_state.scenario_id==CPUPrintPID)
	{
		printf("obs=%d\n", obs);
	}*/
	return false;
}

int UncNavigation::NumActions() const {
	return /*5*/9;
}
void UncNavigation::TestObsProb(const State& state) const
{
	double totalprob=0;
	for (int obs=0;obs<NumObservations();obs++)
	{
		double prob=ObsProb(obs, state,E_STAY);
		logi<<"ObsProb "<<obs<<"="<<prob<<endl;
		totalprob+=prob;
	}
	logi<<"TotalProb="<<totalprob<<endl;
}

double UncNavigation::ObsProb(OBS_TYPE obs, const State& state, int action) const {

	double prob=1;
	UncNavigationState nav_state(static_cast<const UncNavigationState&>(state));

	//int obs_North=(obs%16)/8;
	//int obs_East=((obs%16)-obs_North*8)/4;
	//int obs_South=((obs%16)-obs_North*8-obs_East*4)/2;
	//int obs_West=((obs%16)-obs_North*8-obs_East*4-obs_South*2);
	//int obs_North_East=obs/(int)std::pow(2.0,7);
	//int obs_South_East=(obs-obs_North_East*(int)std::pow(2.0,7))/(int)std::pow(2.0,6);
	//int obs_South_West=(obs-obs_North_East*(int)std::pow(2.0,7)-obs_South_East*(int)std::pow(2.0,6))/(int)std::pow(2.0,5);
	//int obs_North_West=(obs-obs_North_East*(int)std::pow(2.0,7)-obs_South_East*(int)std::pow(2.0,6)-obs_South_West*(int)std::pow(2.0,5))/(int)std::pow(2.0,4);
	OBS_TYPE my_obs = obs;
	int obs_North=(my_obs%16)/8;
	int obs_East=((my_obs%16)-obs_North*8)/4;
	int obs_South=((my_obs%16)-obs_North*8-obs_East*4)/2;
	int obs_West=((my_obs%16)-obs_North*8-obs_East*4-obs_South*2);
	my_obs = my_obs/16;
	int obs_North_East= (my_obs%16)/8;
	int obs_South_East=((my_obs % 16)-obs_North_East*8)/4;
	int obs_South_West=((my_obs % 16)-obs_North_East*8-obs_South_East*4)/2;
	int obs_North_West=((my_obs % 16)-obs_North_East*8-obs_South_East*4-obs_South_West*2);
	int obs_North2, obs_East2, obs_South2, obs_West2, obs_North_East2, obs_South_East2, obs_South_West2, obs_North_West2;

	if(UncNavigation::num_obs_bits > 8)
	{
		my_obs = my_obs/16;
		obs_North2=(my_obs%16)/8;
		obs_East2=((my_obs%16)-obs_North2*8)/4;
		obs_South2=((my_obs%16)-obs_North2*8-obs_East2*4)/2;
		obs_West2=((my_obs%16)-obs_North2*8-obs_East2*4-obs_South2*2);
		my_obs = my_obs/16;
		obs_North_East2= (my_obs%16)/8;
		obs_South_East2=((my_obs % 16)-obs_North_East2*8)/4;
		obs_South_West2=((my_obs % 16)-obs_North_East2*8-obs_South_East2*4)/2;
		obs_North_West2=((my_obs % 16)-obs_North_East2*8-obs_South_East2*4-obs_South_West2*2);
	}
	//PrintObs(state, obs,cout);
	//logi<<"Refracted as:"<< obs_North << obs_East <<obs_South<<obs_West<<endl;

	int truth_North,truth_East,truth_South,truth_West;
	int truth_NE, truth_SE, truth_SW, truth_NW;
	int truth_North2,truth_East2,truth_South2,truth_West2;
	int truth_NE2, truth_SE2, truth_SW2, truth_NW2;
	truth_North=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_NORTH]);
	truth_East=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_EAST]);
	truth_South=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_SOUTH]);
	truth_West=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_WEST]);
	truth_NE=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_NORTH_EAST]);
	truth_SE=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_SOUTH_EAST]);
	truth_SW=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_SOUTH_WEST]);
	truth_NW=nav_state.Grid(nav_state.rob+NavCompass::DIRECTIONS[E_NORTH_WEST]);
	if(UncNavigation::num_obs_bits > 8)
	{
		truth_North2=nav_state.Grid(nav_state.rob.x, nav_state.rob.y + 2);
		truth_East2=nav_state.Grid(nav_state.rob.x + 2, nav_state.rob.y );
		truth_South2=nav_state.Grid(nav_state.rob.x,  nav_state.rob.y -2);
		truth_West2=nav_state.Grid(nav_state.rob.x - 2, nav_state.rob.y );
		truth_NE2=nav_state.Grid(nav_state.rob.x + 2,nav_state.rob.y +2);
		truth_SE2=nav_state.Grid(nav_state.rob.x + 2,nav_state.rob.y -2);
		truth_SW2=nav_state.Grid(nav_state.rob.x - 2,nav_state.rob.y -2);
		truth_NW2=nav_state.Grid(nav_state.rob.x - 2,nav_state.rob.y +2);
	}
	float Noise=OBS_NOISE;
	if(obs_North==truth_North)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: N"<<endl;
	}
	if(obs_East==truth_East)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: E"<<endl;
	}
	if(obs_South==truth_South)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: S"<<endl;
	}
	if(obs_West==truth_West)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: W"<<endl;
	}
	if(obs_North_East==truth_NE)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: NE"<<endl;
	}
	if(obs_South_East==truth_SE)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise:SE"<<endl;
	}
	if(obs_South_West==truth_SW)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: SW"<<endl;
	}
	if(obs_North_West==truth_NW)
		prob*=1-Noise;
	else{
		prob*=Noise;
		if(DESPOT::Debug_mode)
			cout<<"Obs noise: NW"<<endl;
	}


	if(UncNavigation::num_obs_bits > 8)
		{

			prob = prob*100000; //As we are calculating likelihood, multiplying probability with a constant so that it does not become very small
			if(obs_North2==truth_North2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: N2"<<endl;
			}
			if(obs_East2==truth_East2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: E2"<<endl;
			}
			if(obs_South2==truth_South2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: S2"<<endl;
			}
			if(obs_West2==truth_West2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: W2"<<endl;
			}
			if(obs_North_East2==truth_NE2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: NE2"<<endl;
			}
			if(obs_South_East2==truth_SE2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise:SE2"<<endl;
			}
			if(obs_South_West2==truth_SW2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: SW2"<<endl;
			}
			if(obs_North_West2==truth_NW2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				if(DESPOT::Debug_mode)
					cout<<"Obs noise: NW2"<<endl;
			}
		}
	return prob;
}

void UncNavigation::PrintObs(const State& state, OBS_TYPE observation,
	ostream& out) const {
	OBS_TYPE my_obs = observation;
	switch (my_obs%16) {
	case E_FN_FE_FS_FW:
		out << "N0 E0 S0 W0" << " ";
		break;
	case E_FN_FE_FS_OW:
		out << "N0 E0 S0 W1" << " ";
		break;
	case E_FN_FE_OS_FW:
		out << "N0 E0 S1 W0" << " ";
		break;
	case E_FN_FE_OS_OW:
		out << "N0 E0 S1 W1" << " ";
		break;
	case E_FN_OE_FS_FW:
		out << "N0 E1 S0 W0" << " ";
		break;
	case E_FN_OE_FS_OW:
		out << "N0 E1 S0 W1" << " ";
		break;
	case E_FN_OE_OS_FW:
		out << "N0 E1 S1 W0" << " ";
		break;
	case E_FN_OE_OS_OW:
		out << "N0 E1 S1 W1" << " ";
		break;
	case E_ON_FE_FS_FW:
		out << "N1 E0 S0 W0" << " ";
		break;
	case E_ON_FE_FS_OW:
		out << "N1 E0 S0 W1" << " ";
		break;
	case E_ON_FE_OS_FW:
		out << "N1 E0 S1 W0" << " ";
		break;
	case E_ON_FE_OS_OW:
		out << "N1 E0 S1 W1" << " ";
		break;
	case E_ON_OE_FS_FW:
		out << "N1 E1 S0 W0" << " ";
		break;
	case E_ON_OE_FS_OW:
		out << "N1 E1 S0 W1" << " ";
		break;
	case E_ON_OE_OS_FW:
		out << "N1 E1 S1 W0" << " ";
		break;
	case E_ON_OE_OS_OW:
		out << "N1 E1 S1 W1" << " ";
		break;
	}
	my_obs = my_obs/16;
	switch (my_obs % 16) {
	case E_FN_FE_FS_FW:
		out << "NE0 SE0 SW0 NW0" << " ";
		break;
	case E_FN_FE_FS_OW:
		out << "NE0 SE0 SW0 NW1" << " ";
		break;
	case E_FN_FE_OS_FW:
		out << "NE0 SE0 SW1 NW0" << " ";
		break;
	case E_FN_FE_OS_OW:
		out << "NE0 SE0 SW1 NW1" << " ";
		break;
	case E_FN_OE_FS_FW:
		out << "NE0 SE1 SW0 NW0" << " ";
		break;
	case E_FN_OE_FS_OW:
		out << "NE0 SE1 SW0 NW1" << " ";
		break;
	case E_FN_OE_OS_FW:
		out << "NE0 SE1 SW1 NW0" << " ";
		break;
	case E_FN_OE_OS_OW:
		out << "NE0 SE1 SW1 NW1" << " ";
		break;
	case E_ON_FE_FS_FW:
		out << "NE1 SE0 SW0 NW0" << " ";
		break;
	case E_ON_FE_FS_OW:
		out << "NE1 SE0 SW0 NW1" << " ";
		break;
	case E_ON_FE_OS_FW:
		out << "NE1 SE0 SW1 NW0" << " ";
		break;
	case E_ON_FE_OS_OW:
		out << "NE1 SE0 SW1 NW1" << " ";
		break;
	case E_ON_OE_FS_FW:
		out << "NE1 SE1 SW0 NW0" << " ";
		break;
	case E_ON_OE_FS_OW:
		out << "NE1 SE1 SW0 NW1" << " ";
		break;
	case E_ON_OE_OS_FW:
		out << "NE1 SE1 SW1 NW0" << " ";
		break;
	case E_ON_OE_OS_OW:
		out << "NE1 SE1 SW1 NW1" << " ";
		break;
	}
	if(UncNavigation::num_obs_bits > 8)
	{
		out << "|";
		my_obs = my_obs/16;
		switch (my_obs%16) {
		case E_FN_FE_FS_FW:
			out << "N0 E0 S0 W0" << " ";
			break;
		case E_FN_FE_FS_OW:
			out << "N0 E0 S0 W1" << " ";
			break;
		case E_FN_FE_OS_FW:
			out << "N0 E0 S1 W0" << " ";
			break;
		case E_FN_FE_OS_OW:
			out << "N0 E0 S1 W1" << " ";
			break;
		case E_FN_OE_FS_FW:
			out << "N0 E1 S0 W0" << " ";
			break;
		case E_FN_OE_FS_OW:
			out << "N0 E1 S0 W1" << " ";
			break;
		case E_FN_OE_OS_FW:
			out << "N0 E1 S1 W0" << " ";
			break;
		case E_FN_OE_OS_OW:
			out << "N0 E1 S1 W1" << " ";
			break;
		case E_ON_FE_FS_FW:
			out << "N1 E0 S0 W0" << " ";
			break;
		case E_ON_FE_FS_OW:
			out << "N1 E0 S0 W1" << " ";
			break;
		case E_ON_FE_OS_FW:
			out << "N1 E0 S1 W0" << " ";
			break;
		case E_ON_FE_OS_OW:
			out << "N1 E0 S1 W1" << " ";
			break;
		case E_ON_OE_FS_FW:
			out << "N1 E1 S0 W0" << " ";
			break;
		case E_ON_OE_FS_OW:
			out << "N1 E1 S0 W1" << " ";
			break;
		case E_ON_OE_OS_FW:
			out << "N1 E1 S1 W0" << " ";
			break;
		case E_ON_OE_OS_OW:
			out << "N1 E1 S1 W1" << " ";
			break;
		}
		my_obs = my_obs/16;
		switch (my_obs % 16) {
		case E_FN_FE_FS_FW:
			out << "NE0 SE0 SW0 NW0" << " ";
			break;
		case E_FN_FE_FS_OW:
			out << "NE0 SE0 SW0 NW1" << " ";
			break;
		case E_FN_FE_OS_FW:
			out << "NE0 SE0 SW1 NW0" << " ";
			break;
		case E_FN_FE_OS_OW:
			out << "NE0 SE0 SW1 NW1" << " ";
			break;
		case E_FN_OE_FS_FW:
			out << "NE0 SE1 SW0 NW0" << " ";
			break;
		case E_FN_OE_FS_OW:
			out << "NE0 SE1 SW0 NW1" << " ";
			break;
		case E_FN_OE_OS_FW:
			out << "NE0 SE1 SW1 NW0" << " ";
			break;
		case E_FN_OE_OS_OW:
			out << "NE0 SE1 SW1 NW1" << " ";
			break;
		case E_ON_FE_FS_FW:
			out << "NE1 SE0 SW0 NW0" << " ";
			break;
		case E_ON_FE_FS_OW:
			out << "NE1 SE0 SW0 NW1" << " ";
			break;
		case E_ON_FE_OS_FW:
			out << "NE1 SE0 SW1 NW0" << " ";
			break;
		case E_ON_FE_OS_OW:
			out << "NE1 SE0 SW1 NW1" << " ";
			break;
		case E_ON_OE_FS_FW:
			out << "NE1 SE1 SW0 NW0" << " ";
			break;
		case E_ON_OE_FS_OW:
			out << "NE1 SE1 SW0 NW1" << " ";
			break;
		case E_ON_OE_OS_FW:
			out << "NE1 SE1 SW1 NW0" << " ";
			break;
		case E_ON_OE_OS_OW:
			out << "NE1 SE1 SW1 NW1" << " ";
			break;
		}

	}
	out << endl;
}

Dvc_State* UncNavigation::GetPointerToParticleList(int offset,  Dvc_State* full_list) const
{
	return static_cast<Dvc_UncNavigationState*>(full_list)+ offset;
}

} // namespace despot
