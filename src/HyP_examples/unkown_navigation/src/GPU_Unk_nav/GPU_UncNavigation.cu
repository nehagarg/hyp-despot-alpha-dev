#include "GPU_UncNavigation.h"

#include "GPU_base_unc_navigation.h"
#include <base_unc_navigation.h>
#include <despot/GPUutil/GPUrandom.h>


using namespace std;

namespace despot {

DEVICE int num_obs_bits = 8;
DEVICE float OBS_NOISE = 0.03f;
DEVICE Dvc_UncNavigation::Dvc_UncNavigation()

{

}


DEVICE Dvc_UncNavigation::~Dvc_UncNavigation()
{
}

DEVICE bool Dvc_UncNavigation::Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
	OBS_TYPE& obs) {

	Dvc_UncNavigationState& nav_state = static_cast<Dvc_UncNavigationState&>(state);//copy contents, link cells to existing ones
	bool terminal=false;
	reward = 0;

	int dir=threadIdx.y;

	if(dir==0)
	{
		terminal=(nav_state.rob==nav_state.goal);

		reward=-0.1;// small cost for one step
		DvcCoord rob_pos=nav_state.rob;

		float prob=1.0f-STEP_NOISE;

		if (action < E_STAY && terminal!=true) { // Move
			// only succeed with 80% chance
			rob_pos +=(rand_num<prob)? Dvc_Compass::GetDirections(action):DvcCoord(0,0);
			bool validmove=(nav_state.Inside(rob_pos) && nav_state.CollisionCheck(rob_pos)==false);

			nav_state.rob=validmove?rob_pos:nav_state.rob;
			reward=validmove?-0.1:-1;
			reward=(nav_state.rob==nav_state.goal)?/*10*/GOAL_REWARD:reward;
		}

		if (action == E_STAY) { // Sample
			reward=-0.2;
		}

		obs=0;//Initialize obs
	}

	OBS_TYPE obs_i=0;

	unsigned long long int Temp=INIT_QUICKRANDSEED;
	for(dir=0;dir<num_obs_bits;dir++)
	{
		switch(dir)
		{
		case 3:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1);
			break;
		case 2:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y);
			break;
		case 1:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1);
			break;
		case 0:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y);
			break;
		case 4:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y+1);
			break;
		case 5:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y-1);
			break;
		case 6:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y-1):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y-1);
			break;
		case 7:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y+1):!nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y+1);
			break;
		case 11:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y+2):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y+2);
			break;
		case 10:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y);
			break;
		case 9:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x,nav_state.rob.y-2):!nav_state.Grid(nav_state.rob.x,nav_state.rob.y-2);
			break;
		case 8:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y):!nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y);
			break;
		case 12:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y+2):!nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y+2);
			break;
		case 13:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y-2):!nav_state.Grid(nav_state.rob.x-2,nav_state.rob.y-2);
			break;
		case 14:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y-2):!nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y-2);
			break;
		case 15:
			rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
			obs_i=(rand_num<1-OBS_NOISE)?nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y+2):!nav_state.Grid(nav_state.rob.x+2,nav_state.rob.y+2);
			break;
		}
		obs=(obs|(obs_i<<dir));
	}

	if(obs>=Dvc_NumObservations())
		printf("Wrong obs %d", obs);

	if(threadIdx.y==0)
	{
		if(terminal){reward=0;obs=Dvc_NumObservations()-1;}
	}
	return terminal;
}

DEVICE float Dvc_UncNavigation::Dvc_ObsProb(OBS_TYPE& obs, Dvc_State& state, int action)
{
		float prob=1;
		Dvc_UncNavigationState nav_state(static_cast<const Dvc_UncNavigationState&>(state));

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

		if(num_obs_bits > 8)
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
		truth_North=nav_state.Grid(nav_state.rob.x, nav_state.rob.y + 1);
		truth_East=nav_state.Grid(nav_state.rob.x + 1, nav_state.rob.y );
		truth_South=nav_state.Grid(nav_state.rob.x,  nav_state.rob.y -1);
		truth_West=nav_state.Grid(nav_state.rob.x - 1, nav_state.rob.y );
		truth_NE=nav_state.Grid(nav_state.rob.x + 1,nav_state.rob.y +1);
		truth_SE=nav_state.Grid(nav_state.rob.x + 1,nav_state.rob.y -1);
		truth_SW=nav_state.Grid(nav_state.rob.x - 1,nav_state.rob.y -1);
		truth_NW=nav_state.Grid(nav_state.rob.x - 1,nav_state.rob.y +1);
		if(num_obs_bits > 8)
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
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: N"<<endl;
		}
		if(obs_East==truth_East)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: E"<<endl;
		}
		if(obs_South==truth_South)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: S"<<endl;
		}
		if(obs_West==truth_West)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: W"<<endl;
		}
		if(obs_North_East==truth_NE)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: NE"<<endl;
		}
		if(obs_South_East==truth_SE)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise:SE"<<endl;
		}
		if(obs_South_West==truth_SW)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: SW"<<endl;
		}
		if(obs_North_West==truth_NW)
			prob*=1-Noise;
		else{
			prob*=Noise;
			//if(DESPOT::Debug_mode)
			//	cout<<"Obs noise: NW"<<endl;
		}
		if(num_obs_bits > 8)
		{

			prob = prob*100000; //As we are calculating likelihood, multiplying probability with a constant so that it does not become very small
			if(obs_North2==truth_North2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: N"<<endl;
			}
			if(obs_East2==truth_East2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: E"<<endl;
			}
			if(obs_South2==truth_South2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: S"<<endl;
			}
			if(obs_West2==truth_West2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: W"<<endl;
			}
			if(obs_North_East2==truth_NE2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: NE"<<endl;
			}
			if(obs_South_East2==truth_SE2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise:SE"<<endl;
			}
			if(obs_South_West2==truth_SW2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: SW"<<endl;
			}
			if(obs_North_West2==truth_NW2)
				prob*=1-Noise;
			else{
				prob*=Noise;
				//if(DESPOT::Debug_mode)
				//	cout<<"Obs noise: NW"<<endl;
			}
		}
		return prob;
}

DEVICE Dvc_State* Dvc_UncNavigation::Allocate(int state_id, double weight) const {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	Dvc_UncNavigationState* state = new Dvc_UncNavigationState();
	state->state_id = state_id;
	state->weight = weight;

	return state;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Get(Dvc_State* particles, int pos) {
	Dvc_UncNavigationState* particle_i= static_cast<Dvc_UncNavigationState*>(particles)+pos;

	return particle_i;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Alloc( int num) {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	Dvc_UncNavigationState* state = (Dvc_UncNavigationState*)malloc(num*sizeof(Dvc_UncNavigationState));

	for(int i=0;i<num;i++)
		state[i].SetAllocated();
	return state;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Copy(const Dvc_State* particles, int pos) {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	const Dvc_UncNavigationState* particle_i= static_cast<const Dvc_UncNavigationState*>(particles)+pos;
	Dvc_UncNavigationState* state = new Dvc_UncNavigationState();

	*state = *particle_i;
	state->SetAllocated();
	return state;
}
DEVICE void Dvc_UncNavigation::Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_UncNavigationState* src_i= static_cast<const Dvc_UncNavigationState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_UncNavigationState* des_i= static_cast<const Dvc_UncNavigationState*>(des)+pos;

	*des_i = *src_i;
	des_i->SetAllocated();
}

DEVICE void Dvc_UncNavigation::Dvc_Free(Dvc_State* particle) {
	delete static_cast<Dvc_UncNavigationState*>(particle);
}

DEVICE int Dvc_UncNavigation::Dvc_NumObservations() { // one dummy terminal state
	return (int)pow(2.0, 1.0*num_obs_bits);
}


} // namespace despot
