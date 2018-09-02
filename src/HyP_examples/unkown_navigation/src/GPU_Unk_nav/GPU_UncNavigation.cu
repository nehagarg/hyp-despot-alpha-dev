#include "GPU_UncNavigation.h"

#include "GPU_base_unc_navigation.h"
#include <base/base_unc_navigation.h>

using namespace std;

namespace despot {
static int NumObstacles;
static bool NewRound=true;
static DvcCoord* obstacle_pos;
//const DvcCoord Dvc_NavCompass::DIRECTIONS[] = {  DvcCoord(0, 1), DvcCoord(1, 0),DvcCoord(0, -1),
//	DvcCoord(-1, 0), DvcCoord(1, 1), DvcCoord(1, -1), DvcCoord(-1, -1), DvcCoord(-1, 1) };
//const string Dvc_NavCompass::CompassString[] = { "North", "East","South", "West",
//	"NE", "SE", "SW", "NW" };


/* =============================================================================
 * Dvc_UncNavigation class
 * =============================================================================*/

/*Dvc_UncNavigation::Dvc_UncNavigation(string map) :
	Dvc_UncNavigation(map) {
	half_efficiency_distance_ = 20;
}*/


DEVICE Dvc_UncNavigation::Dvc_UncNavigation(/*int size, int obstacles*/)// :
	//grid_(size, size),
	//size_(size),
	//num_obstacles_(obstacles)
{
	/*if (size == 4 && obstacles == 4) {
		Init_4_4();
	} else if (size == 5 && obstacles == 5) {
		Init_5_5();
	} else if (size == 5 && obstacles == 7) {
		Init_5_7();
	} else if (size == 7 && obstacles == 8) {
		Init_7_8();
	} else if (size == 11 && obstacles == 11) {
		Init_11_11();
	} else {
		InitGeneral();
	}*/
	// put obstacles in random positions
	//InitGeneral();
	// InitStates();

}


DEVICE Dvc_UncNavigation::~Dvc_UncNavigation()
{
	//half_efficiency_distance_ = 20;

}

/*__device__ float Obs_Prob_[5][500][16];


DEVICE bool Dvc_UncNavigation::Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
	OBS_TYPE& obs) {

	Dvc_UncNavigationState& nav_state = static_cast<Dvc_UncNavigationState&>(state);//copy contents, link cells to existing ones
	bool terminal=false;
	reward = 0;

	int obs_i=threadIdx.x;

	if(obs_i==0)
	{
		terminal=(nav_state.rob==nav_state.goal);

		reward=-0.1;// small cost for one step
		DvcCoord rob_pos=nav_state.rob;

		float prob=1.0f-STEP_NOISE;

		if (action < E_STAY && terminal!=true) { // Move
			rob_pos +=(rand_num<prob)? Dvc_Compass::GetDirections(action):DvcCoord(0,0);

			bool validmove=(nav_state.Inside(rob_pos) && nav_state.CollisionCheck(rob_pos)==false);

			nav_state.rob=validmove?rob_pos:nav_state.rob;
			reward=validmove?-0.1:-1;
			reward=(nav_state.rob==nav_state.goal)?GOAL_REWARD:reward;
		}

		if (action == E_STAY) { // Sample
			reward=-0.1;
		}
	}
		obs=Dvc_NumObservations()-1;
		__shared__ float Obs_Prob_1[8][16];
		__syncthreads();

			float prob=1;

			prob*=((obs_i&8)>>3==nav_state.Grid(nav_state.rob.x,nav_state.rob.y+1))?1-OBS_NOISE:OBS_NOISE;
			prob*=((obs_i&4)>>2==nav_state.Grid(nav_state.rob.x+1,nav_state.rob.y))?1-OBS_NOISE:OBS_NOISE;
			prob*=((obs_i&2)>>1==nav_state.Grid(nav_state.rob.x,nav_state.rob.y-1))?1-OBS_NOISE:OBS_NOISE;
			prob*=((obs_i&1)==nav_state.Grid(nav_state.rob.x-1,nav_state.rob.y))?1-OBS_NOISE:OBS_NOISE;

			Obs_Prob_1[threadIdx.y][obs_i]=prob;//Debug, test alignment, no enough cache space to use

		__syncthreads();

		prob=0;
		if(obs_i==0)
		{
			for(int i=0;i<Dvc_NumObservations();i++)//pick an obs according to the prob of each one
			{
				prob+=Obs_Prob_1[threadIdx.y][i];
				if(rand_num<=prob)
				{	obs=i;	break;	}
			}
		}

	if(obs_i==0)
	{
		if(terminal){reward=0;}
	}

	return terminal;//Debug,test time
}*/


#define PrintID 142
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
		for(dir=0;dir<8;dir++)
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
			}
			obs=(obs|(obs_i<<dir));

		}


		if(obs>=Dvc_NumObservations())
			printf("Wrong obs %d", obs);
	if(threadIdx.y==0)
	{
		if(terminal){reward=0;obs=Dvc_NumObservations()-1;}
	}
	return terminal/*temp*/;//Debug,test time
}

DEVICE int Dvc_UncNavigation::NumActions() const {
	return /*5*/9;
}


DEVICE float Dvc_UncNavigation::ObsProb(OBS_TYPE obs, const Dvc_State& state, int action) {


	float prob=1;
	const Dvc_UncNavigationState* nav_state=static_cast<const Dvc_UncNavigationState*>(&state);

	int obs_North=obs/8;
	int obs_East=(obs-obs_North*8)/4;
	int obs_South=(obs-obs_North*8-obs_East*4)/2;
	int obs_West=(obs-obs_North*8-obs_East*4-obs_South*2);

	int truth_North,truth_East,truth_South,truth_West;
	truth_North=nav_state->Grid(nav_state->rob+Dvc_Compass::GetDirections(E_NORTH));
	truth_East=nav_state->Grid(nav_state->rob+Dvc_Compass::GetDirections(E_EAST));
	truth_South=nav_state->Grid(nav_state->rob+Dvc_Compass::GetDirections(E_SOUTH));
	truth_West=nav_state->Grid(nav_state->rob+Dvc_Compass::GetDirections(E_WEST));

	float Noise=OBS_NOISE;
	prob*=(obs_North==truth_North)?1-Noise:Noise;
	prob*=(obs_East==truth_East)?1-Noise:Noise;
	prob*=(obs_South==truth_South)?1-Noise:Noise;
	prob*=(obs_West==truth_West)?1-Noise:Noise;


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

	/*for(int i=0;i<num;i++)
		state[i].SetAllocated();*/
	return state;
}

DEVICE Dvc_State* Dvc_UncNavigation::Dvc_Copy(const Dvc_State* particles, int pos) {
	//Dvc_UncNavigationState* state = Dvc_memory_pool_.Allocate();
	const Dvc_UncNavigationState* particle_i= static_cast<const Dvc_UncNavigationState*>(particles)+pos;
	Dvc_UncNavigationState* state = new Dvc_UncNavigationState();

	*state = *particle_i;
	//state->SetAllocated();
	return state;
}
DEVICE void Dvc_UncNavigation::Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_UncNavigationState* src_i= static_cast<const Dvc_UncNavigationState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_UncNavigationState* des_i= static_cast<const Dvc_UncNavigationState*>(des)+pos;

	*des_i = *src_i;
	//des_i->SetAllocated();
}
DEVICE void Dvc_UncNavigation::Dvc_Free(Dvc_State* particle) {
	//Dvc_memory_pool_.Free(static_cast<Dvc_UncNavigationState*>(particle));
	delete static_cast<Dvc_UncNavigationState*>(particle);
}
/*DEVICE void Dvc_UncNavigation::Dvc_FreeList(Dvc_State* particle) {
	//Dvc_memory_pool_.Free(static_cast<Dvc_UncNavigationState*>(particle));
	free(static_cast<Dvc_UncNavigationState*>(particle));
}*/
/*
DEVICE int Dvc_UncNavigation::NumActiveParticles() const {
	return Dvc_memory_pool_.num_allocated();
}
*/
DEVICE int Dvc_UncNavigation::Dvc_NumObservations() { // one dummy terminal state
	return 256;
}

/*
DEVICE DvcCoord Dvc_UncNavigation::GetRobPos(const Dvc_State* state) const {
	return static_cast<const Dvc_UncNavigationState*>(state)->rob;
}
*/

DEVICE OBS_TYPE Dvc_UncNavigation::Dvc_GetObservation(double rand_num,
	const Dvc_UncNavigationState& nav_state) {
	OBS_TYPE obs=Dvc_NumObservations()+10;
	double TotalProb=0;
	bool found=false;

	for(int i=0;i<Dvc_NumObservations();i++)//pick an obs according to the prob of each one
	{
		/*TotalProb+=ObsProb(i, nav_state, E_STAY);
		obs=(rand_num<=TotalProb && !found)?i:obs;
		found=(rand_num<=TotalProb)?true:found;*/
		TotalProb+=ObsProb(i, nav_state, E_STAY);
		if(rand_num<=TotalProb)
		{	obs=i;	break;	}
	}
	return obs;
}
/*DEVICE OBS_TYPE Dvc_UncNavigation::Dvc_GetObservation_parallel(double rand_num,
	const Dvc_UncNavigationState& nav_state) {
	float temp=0;

	clock_t start_time = clock();

	OBS_TYPE obs=Dvc_NumObservations()+10;
	double TotalProb=0;
	//bool found=false;

	int action=blockIdx.x;
	int PID=blockIdx.y*blockDim.y+threadIdx.y;
	int obs_i=threadIdx.x;
	clock_t start_time1 = clock();

	__syncthreads();

	Obs_Prob_[action][PID][obs_i]=ObsProb(obs_i, nav_state, E_STAY);

	__syncthreads();
	clock_t stop_time1 = clock();

	float runtime1= (float)(stop_time1 - start_time1)/1733500.0;

	for(int i=0;i<Dvc_NumObservations();i++)//pick an obs according to the prob of each one
	{
		TotalProb+=Obs_Prob_[action][PID][i];
		if(rand_num<=TotalProb)
		{	obs=i;	break;	}
	}
	clock_t stop_time = clock();

	float runtime= (float)(stop_time - start_time)/1733.500;
	//runtime++;
	temp=runtime;
	//__syncthreads();

	return obs;//Debug, test time
}*/
/*
DEVICE int Dvc_UncNavigation::GetX(const Dvc_UncNavigationState* state) const {
	return state->rob.x;
}

DEVICE void Dvc_UncNavigation::IncX(Dvc_UncNavigationState* state) const {
	state->rob.x+=1;
}

DEVICE void Dvc_UncNavigation::DecX(Dvc_UncNavigationState* state) const {
	state->rob.x-=1;
}

DEVICE int Dvc_UncNavigation::GetY(const Dvc_UncNavigationState* state) const {
	return state->rob.y;
}

DEVICE void Dvc_UncNavigation::IncY(Dvc_UncNavigationState* state) const {
	state->rob.y+=1;
}

DEVICE void Dvc_UncNavigation::DecY(Dvc_UncNavigationState* state) const {
	state->rob.y-=1;
}


DEVICE Dvc_UncNavigationState Dvc_UncNavigation::NextState(Dvc_UncNavigationState& s, int a) const {
	if (s.rob==s.goal)// terminal state is an absorbing state
		return s;

    double Rand=Random::RANDOM.NextDouble();
	DvcCoord rob_pos = s.rob;
	Dvc_UncNavigationState newState(s);
	if (a < E_STAY) {//movement actions
	    if(Rand<0.8)// only succeed with 80% chance
	    	rob_pos += Dvc_NavCompass::DIRECTIONS[a];
		if (s.Inside(rob_pos) && s.CollisionCheck(rob_pos)==false) {
			newState.rob=rob_pos;//move the robot
		} else {
			;// don't move the robot
		}
		return newState;
	} else if (a == E_STAY) {//stay action
		return newState;
	} else //unsupported action
		return s;
}

DEVICE double Dvc_UncNavigation::Reward(Dvc_UncNavigationState& s, int a) const {
	if (s.rob==s.goal)// at terminal state, no reward
		return 0;
	DvcCoord rob_pos = s.rob;
	if (a < E_STAY) {
	    double Rand=Random::RANDOM.NextDouble();
	    if(Rand<0.8)// only succeed with 80% chance
	    	rob_pos += Dvc_NavCompass::DIRECTIONS[a];
	    if(rob_pos==s.goal)// arrive goal
	    	return 10;
		if (s.Inside(rob_pos) && s.CollisionCheck(rob_pos)==false) {
			return -0.1;// small cost for each move
		} else
			return -1;//run into obstacles or walls
	} else if (a == E_STAY) {
		return -0.1;// small cost for each step
	} else //unsupported action
		return 0;
}
*/


} // namespace despot
