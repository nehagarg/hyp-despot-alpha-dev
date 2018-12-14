#include "base_unc_navigation.h"

#include "../GPU_Unk_nav/GPU_base_unc_navigation.h"
#include <despot/core/particle_belief.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_policygraph.h>

using namespace std;

namespace despot {

static int NumObstacles;
static int NumFixObstacles;
static bool NewRound=true;
static Coord* obstacle_pos=NULL;
static Coord* fix_obstacle_pos=NULL;
static Coord* fix_line_obstacle_pos=NULL;
const Coord NavCompass::DIRECTIONS[] = {  Coord(0, 1), Coord(1, 0),Coord(0, -1),
	Coord(-1, 0), Coord(1, 1), Coord(1, -1), Coord(-1, -1), Coord(-1, 1) };
const string NavCompass::CompassString[] = { "North", "East","South", "West",
	"NE", "SE", "SW", "NW" };

PolicyGraph* policy_graph = NULL;

int BaseUncNavigation::num_obs_bits = 8;
float BaseUncNavigation::OBS_NOISE = 0.03f;
/* ==============================================================================
 * UncNavigationState class
 * ==============================================================================*/

UncNavigationState::UncNavigationState()
{
	sizeX_=0;
	sizeY_=0;
	rob.x=-1;rob.y=-1;
	cells=NULL;
}
UncNavigationState::UncNavigationState(int _state_id)
{
	sizeX_=0;sizeY_=0;
	rob.x=-1;rob.y=-1;
	goal.x=-1;goal.y=-1;
	cells=NULL;
	state_id=_state_id;
}
UncNavigationState::UncNavigationState(int sizeX, int sizeY)
{
	cells=NULL;
	InitCells(sizeX,sizeY);
	rob.x=-1;rob.y=-1;
	goal.x=-1;goal.y=-1;
}

UncNavigationState::UncNavigationState(const UncNavigationState& src)
{
	cells=NULL;
	Assign(src);
	//rob.x=src.rob.x; rob.y=src.rob.y;
	//goal=src.goal;
	//InitCells(src.sizeX_,src.sizeY_);
	//sizeX_=src.sizeX_;sizeY_=src.sizeY_;
	//cells=new bool[sizeX_*sizeY_];

	//memcpy((void*)cells,(const void*)src.cells, sizeX_*sizeY_*sizeof(bool));
}

string UncNavigationState::text() const {
	return "id = " + to_string(state_id);
}

UncNavigationBelief::UncNavigationBelief(vector<State*> particles, const DSPOMDP* model, Belief* prior , bool split ) :
		ParticleBelief(particles, model, prior, split)
{

}
void UncNavigationBelief::Update(ACT_TYPE action, OBS_TYPE obs)
{
	ParticleBelief::Update(action, obs); //Update before so that particles are stepped
	//Then add new synced particles
	int num_particles = particles_.size();
	int N = num_particles/100;
	if(N < 1)
	{
		N = 1;
	}
	vector<State*> new_particles = Sample(N);
	std::vector<int> obss;
	obss.resize(BaseUncNavigation::num_obs_bits);
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
	obss[NavCompass::NORTH] = obs_North;
	obss[NavCompass::EAST] = obs_East;
	obss[NavCompass::SOUTH] = obs_South;
	obss[NavCompass::WEST] = obs_West;
	obss[NavCompass::NORTHEAST] = obs_North_East;
	obss[NavCompass::SOUTHEAST] = obs_South_East;
	obss[NavCompass::SOUTHWEST] = obs_South_West;
	obss[NavCompass::NORTHWEST] = obs_North_West;
	if(BaseUncNavigation::num_obs_bits > 8)
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
		obss[8+ NavCompass::NORTH] = obs_North2;
		obss[8 + NavCompass::EAST] = obs_East2;
		obss[8 + NavCompass::SOUTH] = obs_South2;
		obss[8+ NavCompass::WEST] = obs_West2;
		obss[8+ NavCompass::NORTHEAST] = obs_North_East2;
		obss[8+ NavCompass::SOUTHEAST] = obs_South_East2;
		obss[8+ NavCompass::SOUTHWEST] = obs_South_West2;
		obss[8+ NavCompass::NORTHWEST] = obs_North_West2;
	}





	//Push some particles consistent with obs into belief
	for (int i = 0; i <new_particles.size(); i++) {
		new_particles[i]->weight = 1.0/(N + num_particles);
		UncNavigationState* nav_state = static_cast<UncNavigationState*>(new_particles[i]);
		particles_.push_back(nav_state);

		for(int j = 0; j < BaseUncNavigation::num_obs_bits; j++) //iterate over directions
		{


				Coord  pos = nav_state->rob+NavCompass::DIRECTIONS[j%8];
				if(j >=8)
				  {
				    pos = pos + NavCompass::DIRECTIONS[j%8];
				  }
				if(nav_state->Inside(pos))
				{
					double change_prob = Random::RANDOM.NextDouble();
					if(change_prob < BaseUncNavigation::OBS_NOISE)
					{
						nav_state->GridOpen(pos)= 1-obss[j];
					}
					else
					{
						nav_state->GridOpen(pos)= obss[j];
					}
				}

		}


	}

	//Make weight sum to 1
	double total_weight = 1.0 + (1.0*num_particles/(N + num_particles));
	for (int i = 0; i < particles_.size(); i++) {
			State* particle = particles_[i];
			particle->weight /= total_weight;

		}

}

BaseUncNavigation::BaseUncNavigation(int size, int obstacles) :
	size_(size),
	num_obstacles_(obstacles) 
{
	cout<<__FUNCTION__<<endl;
	//num_obs_bits = 8;
}


void BaseUncNavigation::RandGate(UncNavigationState* nav_state) const
{
   Coord pos;
   pos=nav_state->GateNorth();
   nav_state->GridOpen(pos) = (bool)Random::RANDOM.NextInt(2); // randomly put obstacles there
   pos=nav_state->GateEast();
   nav_state->GridOpen(pos) = (bool)Random::RANDOM.NextInt(2); // randomly put obstacles there
   pos=nav_state->GateWest();
   nav_state->GridOpen(pos) = (bool)Random::RANDOM.NextInt(2); // randomly put obstacles there
}

void BaseUncNavigation::RandMap(UncNavigationState* nav_state, float ObstacleProb, int skip) const
{
	//assign obstacle with prob ObstacleProb at 1/skip of the map
	for (int x=0;x<nav_state->sizeX_;x+=skip)
		for (int y=0;y<nav_state->sizeY_;y+=skip)
		{
			Coord pos(x,y);
			if(nav_state->Grid(pos)==false
					&& pos!=nav_state->goal )//ignore existing obstacles
				nav_state->GridOpen(pos)=Random::RANDOM.NextDouble()<ObstacleProb? true:false;
		}
}
void BaseUncNavigation::RandMap(UncNavigationState* nav_state, float prob, bool draw_mid_line) const
{
	for(int i=0;i<NumObstacles;i++)
	{
		Coord pos=obstacle_pos[i];
		double rand=Random::RANDOM.NextDouble();
		if(rand<prob)
			nav_state->GridOpen(pos)=true;//put obstacle there
		else
			nav_state->GridOpen(pos)=false;//put no obstacle there
	}
	for(int i=0;i<NumFixObstacles;i++)
	{
		Coord pos=fix_obstacle_pos[i];
		nav_state->GridOpen(pos)=true;//put obstacle there
	}
	//CalFixLineObstacles(size_/4,2,nav_state);
	if(draw_mid_line)CalGateObstacles(size_/2+1,2,1,nav_state);
	//CalGateObstacles(3*size_/4,2,nav_state);
	//CalFixLineObstacles(size_/2,3,nav_state);
}


void BaseUncNavigation::CalObstacles(float prob,UncNavigationState* nav_state, bool draw_mid_line) const
{
	if(prob>0)
	{
		//allocate a temporary state
		if(!nav_state)
		{
			nav_state = memory_pool_.Allocate();
			nav_state->InitCells(size_,size_);
			//put the goal first
			nav_state->FixedGoal();
		}
		CalFixObstacles(0.2,nav_state,draw_mid_line);
		if(draw_mid_line)
			NumObstacles=prob*(float)(size_*size_-1-NumFixObstacles-size_*1);//excluding goal positions and mideline gate
		else
			NumObstacles=prob*(float)(size_*size_-1-NumFixObstacles);//excluding goal positions

		if(obstacle_pos)delete [] obstacle_pos;
		obstacle_pos=new Coord[NumObstacles];

		int ExistingObs=0;


		//generate obstacles, excluding the center horizontal line
		Coord pos;
		do {
			do {
				pos = Coord(Random::RANDOM.NextInt(size_),
					Random::RANDOM.NextInt(size_));
			} while (nav_state->Grid(pos) == true || pos==nav_state->goal
					||(draw_mid_line && /*pos.y<=size_-size_/2+1 && pos.y>=size_-size_/2-1*/
							pos.y==size_-size_/2-1)  ) ;// check for random free map position
			nav_state->GridOpen(pos)=true;//put obstacle there
			obstacle_pos[ExistingObs]=pos;
			ExistingObs++;
		}while(ExistingObs<NumObstacles);

		cout<<"creating maps with "<<ExistingObs<<" random obstacles"<<endl;

	}
	else
		obstacle_pos=NULL;
}

void BaseUncNavigation::CalFixObstacles(float percentage,UncNavigationState* nav_state, bool draw_mid_line) const
{
	NumFixObstacles=percentage*(float)(size_*(size_-draw_mid_line*1));
	if(NumFixObstacles>0)
	{
		if(fix_obstacle_pos)delete [] fix_obstacle_pos;
		fix_obstacle_pos=new Coord[NumFixObstacles];

		int ExistingObs=0;
		//allocate a temporary state
		if(!nav_state)
		{
			nav_state = memory_pool_.Allocate();
			nav_state->InitCells(size_,size_);
			//put the goal first
			nav_state->FixedGoal();
		}
		//generate obstacles
		Coord pos;
		do {
			do {
				pos = Coord(Random::RANDOM.NextInt(size_),
					Random::RANDOM.NextInt(size_));
			} while (nav_state->Grid(pos) == true || pos==nav_state->goal
					||(draw_mid_line && /*pos.y<=size_-size_/2+1 && pos.y>=size_-size_/2-1*/
							pos.y==size_-size_/2-1)  );// check for random free map position
			nav_state->GridOpen(pos)=true;//put obstacle there
			fix_obstacle_pos[ExistingObs]=pos;
			ExistingObs++;
		}while(ExistingObs<NumFixObstacles);

		cout<<"creating maps with "<<ExistingObs<<" fixed obstacles"<<endl;
	}
	else
		fix_obstacle_pos=NULL;
}
void BaseUncNavigation::CalFixLineObstacles(int line_pos, int num_gates,UncNavigationState* nav_state) const
{
	int num_obstacles=size_-num_gates;
	if(num_obstacles>0)
	{
		//allocate a temporary state
		if(!nav_state)
		{
			nav_state = memory_pool_.Allocate();
			nav_state->InitCells(size_,size_);
			//put the goal first
			nav_state->FixedGoal();
		}
		//generate obstacles
		Coord pos(0, size_-line_pos);

		int factor=size_/(num_gates+1);
		for(int i=0;i<size_;i++)
		{
			pos.x=i;
			if(i==0 || i%factor!=0 || i==size_-1)
				nav_state->GridOpen(pos)=true;//put obstacle there
			else
				nav_state->GridOpen(pos)=false;//clear obstacle there
		}
		//cout<<"creating line obstacles with "<<num_gates<<" gates"<<endl;
	}
}
void BaseUncNavigation::CalGateObstacles(int line_pos, int num_gates,int num_opens,UncNavigationState* nav_state) const
{
	int num_obstacles=size_-num_gates;
	if(num_obstacles>0)
	{
		//allocate a temporary state
		if(!nav_state)
		{
			nav_state = memory_pool_.Allocate();
			nav_state->InitCells(size_,size_);
			//put the goal first
			nav_state->FixedGoal();

			Coord neighbor(nav_state->goal.x+Random::RANDOM.NextInt(2)-1, nav_state->goal.y+Random::RANDOM.NextInt(1));
			nav_state->GridOpen(neighbor)=false;//put obstacle there
			Coord neighbor1(nav_state->goal.x+Random::RANDOM.NextInt(2)-1, nav_state->goal.y+Random::RANDOM.NextInt(1));
			nav_state->GridOpen(neighbor1)=false;//put obstacle there
		}
		//generate obstacles
		Coord pos(0, size_-line_pos);

		int factor=size_/(num_gates/*+1*/);
		int* gate_pos=new int[num_opens];
		for(int i=0;i<num_opens;i++)
		{
			int ID;bool valid=true;
			do{
				ID=Random::RANDOM.NextInt(num_gates);
				valid=true;
				for(int j=0;j<i;j++)
				{if(ID==gate_pos[j]){valid=false;break;}}
			}
			while(!valid);
			gate_pos[i]=ID;
		}
		//Gose the wall
		for(int i=0;i<size_;i++)
		{
			pos.x=i;
			nav_state->GridOpen(pos)=true;//put obstacle there
		}
		//Open gates
		for(int i=0;i<num_opens;i++)
		{
			pos.x=(gate_pos[i]+1)*(factor)-factor/2;
			nav_state->GridOpen(pos)=false;//clear obstacle there

			Coord neighbor(pos.x, pos.y+1);
			nav_state->GridOpen(neighbor)=false;//put obstacle there
			Coord neighbor1(pos.x, pos.y-1);
			nav_state->GridOpen(neighbor1)=false;//put obstacle there
		}
		//cout<<"creating line obstacles with "<<num_gates<<" gates"<<endl;

		delete [] gate_pos;
	}
}
void BaseUncNavigation::FreeObstacles() const
{
	if(obstacle_pos)delete [] obstacle_pos;obstacle_pos=NULL;
	if(fix_obstacle_pos)delete [] fix_obstacle_pos;fix_obstacle_pos=NULL;
}

State* BaseUncNavigation::CreateStartState(string type) const {
	bool use_mid_line=true;
	if(NewRound)
	{
		CalObstacles(/*0.2*/1.0,NULL,use_mid_line);
		//NumObstacles=0;
		//NumFixObstacles=0;
	}
	//UncNavigationState state(size_, size_);
	UncNavigationState* startState = memory_pool_.Allocate();
	startState->InitCells(size_,size_);
	//if(NewRound)PrintState((State&)*startState,cout);

	//put the goal first
	startState->FixedGoal();
	// put obstacles in fixed positions
	Coord pos;
	if (num_obstacles_>0)
	{
		pos.x=size_/4; pos.y=3*size_/4;
		startState->GridOpen(pos) = true; // put the obstacle there
	}
	if (num_obstacles_>1)
	{
		pos.x=2*size_/4; pos.y=2*size_/4;
		startState->GridOpen(pos) = true; // put the obstacle there
	}
	if (num_obstacles_>2)
	{
		pos.x=size_/2-2; pos.y=0;
		startState->GridOpen(pos) = true; // put the obstacle there
	}
	if (num_obstacles_>3)
	{
		pos.x=0; pos.y=1*size_/4;
		startState->GridOpen(pos) = true; // put the obstacle there
	}
	if (num_obstacles_>4)
	{
		pos.x=size_-2; pos.y=1*size_/4+1;
		startState->GridOpen(pos) = true; // put the obstacle there
	}

	//RandGate(startState);
	RandMap(startState,/*0.8*/0.1, use_mid_line);//Generate map using the obstacle positions specified in obstacle_pos

	// pick a random position for the robot, always do this in the last step
	//AreaRobPos(startState,1);
	LineRobPos(startState,0,1);
	//UniformRobPos(startState);
	//UniUpperRobPos(startState);
	NewRound=false;


	return startState;
}
void BaseUncNavigation::UniformRobPos(UncNavigationState* startState) const
{
	Coord pos;
	do {
		pos = Coord(Random::RANDOM.NextInt(size_),
			Random::RANDOM.NextInt(size_));
	} while (/*startState->Grid(pos) == true || */pos==startState->goal);// check for random free map position
	startState->rob=pos;//put robot there
}

void BaseUncNavigation::UniUpperRobPos(UncNavigationState* startState) const
{
	Coord pos;
	do {
		pos = Coord(Random::RANDOM.NextInt(size_),
			size_-1-Random::RANDOM.NextInt(size_/2-1));
	} while (/*startState->Grid(pos) == true || */pos==startState->goal);// check for random free map position
	startState->rob=pos;//put robot there
}
void BaseUncNavigation::AreaRobPos(UncNavigationState* startState, int area_size) const
{
	// robot start from top middle block of area_size
	Coord Robpos(size_/2-area_size/2/*-1*/,size_-area_size);
	Coord pos;

/*	bool has_slot=false;
	for(int x=Robpos.x;x<Robpos.x+area_size;x++)
		for(int y=Robpos.y;y<Robpos.y+area_size;y++)
		{
			if(startState->Grid(x,y) == false)
			{
				has_slot=true;
				break;
			}
		}*/

	/*if(has_slot)
	{*/
		do {
			pos = Robpos+Coord(Random::RANDOM.NextInt(area_size),
				Random::RANDOM.NextInt(area_size));
		} while (/*startState->Grid(pos) == true || */pos==startState->goal);// check for random free map position
		startState->GridOpen(pos) = false ;
		startState->rob=pos;//put robot there
		//cout<<"("<<pos.x<<","<<pos.y<<")"<<" ";
/*	}
	else
	{
		AreaRobPos(startState,area_size+2);//enlarge the area and re-assign
	}*/
}

void BaseUncNavigation::LineRobPos(UncNavigationState* startState, int start_line, int line_width) const
{
	// robot start from top middle block of area_size
	Coord Robpos(0,size_-start_line-line_width);
	Coord pos;

	do {
		pos = Robpos+Coord(Random::RANDOM.NextInt(size_),
				Random::RANDOM.NextInt(line_width));
	} while (/*startState->Grid(pos) == true || */pos==startState->goal);// check for random free map position
	startState->GridOpen(pos) = false ;
	startState->rob=pos;//put robot there
}

Belief* BaseUncNavigation::InitialBelief(const State* start, string type) const {
	int N ; // = Globals::config.num_scenarios*10;
	//int N_min ;= 50000;
	if(use_special_belief)
	{
		N= 10000;
	}
	else
	{
		N = 50000;
	}
	//if (N < N_min)
	//{
	//	N = N_min;
	//}

	vector<State*> particles(N);
	if(FIX_SCENARIO==1)
	{
		ifstream fin;fin.open("InitialBelief.txt", ios::in);
		ImportStateList(particles,fin);
		fin.close();
		//ExportState(*particles[i],fout);
	}
	else
	{
		for (int i = 0; i < N; i++) {
			particles[i] = CreateStartState();
			particles[i]->weight = 1.0 / N;
		}
	}
	if(FIX_SCENARIO==2)
	{
		ofstream fout;fout.open("InitialBelief.txt", ios::trunc);

		fout<< particles.size()<<endl;
		for(int i=0;i<particles.size();i++)
			ExportState(*particles[i],fout);
		fout.close();
	}

	NewRound=true;
	if(use_special_belief)
	{
		return new UncNavigationBelief(particles, this);
	}
	else
	{
		return new ParticleBelief(particles, this);
	}
}

class UncNavigationParticleUpperBound1: public ParticleUpperBound {
protected:
	const BaseUncNavigation* rs_model_;
public:
	UncNavigationParticleUpperBound1(const BaseUncNavigation* model) :
		rs_model_(model) {
	}

	double Value(const State& state) const {
		const UncNavigationState& nav_state =
			static_cast<const UncNavigationState&>(state);
		int count_x=abs(nav_state.rob.x-nav_state.goal.x);
		int count_y=abs(nav_state.rob.y-nav_state.goal.y);
		double value1=0, value2=0;

		int min_xy=min(count_x,count_y);
		int diff_xy=abs(count_x-count_y);
		for (int i=0;i<min_xy;i++)
		{
			value1+=Globals::Discount(i)*(-0.1);
		}
		for (int j=0;j<diff_xy;j++)
		{
			value1+=Globals::Discount(min_xy+j)*(-0.1);
		}

		return value1+Globals::Discount(min_xy+diff_xy-1)*(/*10*/GOAL_REWARD);
	}
};
ScenarioUpperBound* BaseUncNavigation::CreateScenarioUpperBound(string name,
	string particle_bound_name) const {
	ScenarioUpperBound* bound = NULL;
	if (name == "DEFAULT" || "UB1") {
		bound = new UncNavigationParticleUpperBound1(this);
	} else if (name ==/* "DEFAULT" || */"TRIVIAL") {
		bound = new TrivialParticleUpperBound(this);
	} else {
		cerr << "Unsupported scenario upper bound: " << name << endl;
		exit(0);
	}

	if (Globals::config.useGPU)
		InitGPUUpperBound(name,	particle_bound_name);
	return bound;
}

ScenarioLowerBound* BaseUncNavigation::CreateScenarioLowerBound(string name, string
	particle_bound_name) const {

	ScenarioLowerBound* lb;

	if (name == "TRIVIAL") {
		lb = new TrivialParticleLowerBound(this);
	} else if (name == "RANDOM") {
		cout << "Policy tree rollout"<<endl;
		Globals::config.rollout_type="INDEPENDENT";
		lb = new RandomPolicy(this,
			CreateParticleLowerBound(particle_bound_name));
	} else if (name == "DEFAULT" || name == "RANDOM_GRAPH") {
		cout << "Policy graph rollout"<<endl;
		RandomPolicyGraph* tmp=new RandomPolicyGraph(this,
				CreateParticleLowerBound(particle_bound_name));
		tmp->ConstructGraph(POLICY_GRAPH_SIZE, NumObservations());
		tmp->SetEntry(0);
		Globals::config.rollout_type="GRAPH";
		policy_graph = tmp;
		lb = tmp;
	} else {
		cerr << "Unsupported lower bound algorithm: " << name << endl;
		exit(0);
		lb = NULL;
	}

	if (Globals::config.useGPU)
		InitGPULowerBound(name, particle_bound_name);
	return lb;
}

void BaseUncNavigation::PrintState(const State& state, ostream& out) const {
	UncNavigationState navstate=static_cast<const UncNavigationState&>(state);
	ios::fmtflags old_settings = out.flags();
	bool is_marker=false;

	int Width=7;
	int Prec=1;
	out << endl;
	for (int x = 0; x < size_ + 2; x++)
	{
		out.width(Width);out.precision(Prec);	out << "# ";
	}
	out << endl;
	for (int y = size_ - 1; y >= 0; y--) {
		out.width(Width);out.precision(Prec);	out << "# ";
		for (int x = 0; x < size_; x++) {
			Coord pos(x, y);
			int obstacle = navstate.GridStrict(pos);
			if (navstate.goal == Coord(x, y))
			{
				out.width(Width);out.precision(Prec);	out << "G ";
			}
			else if (GetRobPos(&state) == Coord(x, y))
			{
				out.width(Width);out.precision(Prec);	out << "R ";
			}
			else if (obstacle ==true)
			{
				is_marker=false;
				for(int i=0;i<NumFixObstacles;i++)
				{
					if(fix_obstacle_pos[i]==Coord(x,y))
						is_marker=true;
				}

				if(is_marker)
					{out.width(Width);out.precision(Prec);	out << "M ";}
				else
					{out.width(Width);out.precision(Prec);	out << "X ";}
			}
			else
			{
				out.width(Width);out.precision(Prec);	out << ". ";
			}
		}
		out.width(Width);out.precision(Prec);	out << "#" << endl;
	}
	for (int x = 0; x < size_ + 2; x++)
	{
		out.width(Width);out.precision(Prec);	out << "# ";
	}
	out << endl;

	out.flags(old_settings);

	//PrintBelief(*belief_);
}

void BaseUncNavigation::ExportState(const State& state, ostream& out) const {
	UncNavigationState navstate=static_cast<const UncNavigationState&>(state);
	ios::fmtflags old_settings = out.flags();

	int Width=7;
	int Prec=3;
	//out << "Head ";
	out << navstate.scenario_id <<" ";
	out << navstate.weight <<" ";
	out << navstate.rob.x <<" "<<navstate.rob.y<<" ";
	out << navstate.goal.x <<" "<<navstate.goal.y<<" ";

	for (int x = 0; x < size_; x++)
		for (int y = 0; y < size_; y++)
		{
			Coord pos(x, y);
			int obstacle = navstate.GridStrict(pos);
			out << obstacle<< " ";
		}
	out /*<< "End"*/<<endl;

	out.flags(old_settings);
}

State* BaseUncNavigation::ImportState(istream& in) const {
	UncNavigationState* navState = memory_pool_.Allocate();
	navState->InitCells(size_,size_);

	if (in.good())
	{
	    string str;
	    while(getline(in, str))
	    {
	        if(!str.empty())
	        {
		        istringstream ss(str);

		        int num; int x=0;int y=0;
				ss >> navState->scenario_id;
				ss >> navState->weight;
				ss >> navState->rob.x >>navState->rob.y;
				ss >> navState->goal.x >>navState->goal.y;
				while(ss >> num)
				{
					if(x>=size_)
						cout<<"Import state error: x >= size_!"<<endl;
					Coord pos(x, y);
					navState->GridOpen(pos)=num;
					y++;
					if(y==size_)
					{
						x++;
						y=0;
					}
				}
	        }
	    }
	}

	return navState;
}

void BaseUncNavigation::ImportStateList(std::vector<State*>& particles, std::istream& in) const {

	if (in.good())
	{
		int PID=0;
		string str;
		getline(in, str);
		//cout<<str<<endl;
		istringstream ss(str);
		int size;
		ss>>size;
		particles.resize(size);
		while(getline(in, str))
		{
			if(!str.empty())
			{
				if(PID>=particles.size())
					cout<<"Import particles error: PID>=particles.size()!"<<endl;

				UncNavigationState* navState = memory_pool_.Allocate();
				navState->InitCells(size_,size_);

			    istringstream ss(str);

				int num; int x=0;int y=0;
				ss >> navState->scenario_id;
				ss >> navState->weight;
				ss >> navState->rob.x >>navState->rob.y;
				ss >> navState->goal.x >>navState->goal.y;
				while(ss >> num)
				{
					if(x>=size_)
						cout<<"Import state error: x >= size_!"<<endl;
					Coord pos(x, y);
					navState->GridOpen(pos)=num;
					y++;
					if(y==size_)
					{
						x++;
						y=0;
					}
				}
				particles[PID]=navState;
				PID++;

			}
		}
	}


}

void BaseUncNavigation::PrintBeliefMap(float** Beliefmap, std::ostream& out) const
{
	ios::fmtflags old_settings = out.flags();
	//out.precision(5);
	//out.width(4);
	int Width=6;
	int Prec=1;
	out << endl;
	out.width(Width);out.precision(Prec);
	for (int x = 0; x < size_ + 2; x++)
	{
		out.width(Width);out.precision(Prec);	out << "# ";
	}
	out << endl;
	for (int y = size_ - 1; y >= 0; y--) {
		out.width(Width);out.precision(Prec); out << "# ";
		for (int x = 0; x < size_; x++) {
			out.width(Width);out.precision(Prec);
			/*if(Beliefmap[x][y]>0.2)
			{
				out.width(Width-3);
				out <<"<"<<Beliefmap[x][y]<<">"<<" ";
				out.width(Width);
			}
			else*/
				out <<Beliefmap[x][y]<<" ";
		}
		out.width(Width);out.precision(Prec); out << "#" << endl;
	}
	for (int x = 0; x < size_ + 2; x++)
		{out.width(Width);out.precision(Prec); out << "# ";}
	out << endl;
	out.flags(old_settings);
}
void BaseUncNavigation::AllocBeliefMap(float**& Beliefmap) const
{
	Beliefmap =new float*[size_];
	for(int i=0;i<size_;i++)
	{
		Beliefmap[i]=new float[size_];
		memset((void*)Beliefmap[i], 0 , size_*sizeof(float));
	}
}
void BaseUncNavigation::ClearBeliefMap(float**& Beliefmap) const
{
	for(int i=0;i<size_;i++)
	{
		delete [] Beliefmap[i];
	}
	delete [] Beliefmap;
}
void BaseUncNavigation::PrintBelief(const Belief& belief, ostream& out) const {
	const vector<State*>& particles =
		static_cast<const ParticleBelief&>(belief).particles();
	PrintParticles(particles, out);
}

void BaseUncNavigation::PrintAction(int action, ostream& out) const {
	if (action < E_STAY)
		out << NavCompass::CompassString[action] << endl;
	if (action == E_STAY)
		out << "Stay" << endl;
}

State* BaseUncNavigation::Allocate(int state_id, double weight) const {
	UncNavigationState* state = memory_pool_.Allocate();
	state->state_id = state_id;
	state->weight = weight;
	state->InitCells(size_,size_);
	return state;
}

State* BaseUncNavigation::Copy(const State* particle) const {
	UncNavigationState* state = memory_pool_.Allocate();
	*state = *static_cast<const UncNavigationState*>(particle);
	state->SetAllocated();
	return state;
}



void BaseUncNavigation::Free(State* particle) const {
	memory_pool_.Free(static_cast<UncNavigationState*>(particle));
}

int BaseUncNavigation::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

int BaseUncNavigation::NumObservations() const { // one dummy terminal state
	return (int)std::pow(2.0, num_obs_bits);
}


Coord BaseUncNavigation::GetRobPos(const State* state) const {
	return static_cast<const UncNavigationState*>(state)->rob;
}

OBS_TYPE BaseUncNavigation::GetObservation(double rand_num,
	const UncNavigationState& nav_state) const {
	OBS_TYPE obs=NumObservations()-1;
	double TotalProb=0;
	int i=0;
	for(i=0;i<NumObservations();i++)//pick an obs according to the prob of each one
	{
		TotalProb+=ObsProb(i, nav_state, E_STAY);
		if(rand_num<=TotalProb)
		{	obs=i;	break;	}
	}
	/*if(obs>=NumObservations() || obs<0 || i>=NumObservations())
		return 0;*/
	return obs;
}


int BaseUncNavigation::GetX(const UncNavigationState* state) const {
	return state->rob.x;
}

void BaseUncNavigation::IncX(UncNavigationState* state) const {
	state->rob.x+=1;
}

void BaseUncNavigation::DecX(UncNavigationState* state) const {
	state->rob.x-=1;
}

int BaseUncNavigation::GetY(const UncNavigationState* state) const {
	return state->rob.y;
}

void BaseUncNavigation::IncY(UncNavigationState* state) const {
	state->rob.y+=1;
}

void BaseUncNavigation::DecY(UncNavigationState* state) const {
	state->rob.y-=1;
}

void BaseUncNavigation::PrintParticles(const std::vector<State*> particles, std::ostream& out) const
{
	out << "Robot position belief:";
		float** Beliefmap;float** ObsBeliefmap;
		AllocBeliefMap(Beliefmap);AllocBeliefMap(ObsBeliefmap);
		float GateState[3];

		memset((void*)GateState, 0 , 3*sizeof(float));
		for (int i = 0; i < particles.size(); i++) {

			const UncNavigationState* navstate = static_cast<const UncNavigationState*>(particles[i]);

			GateState[0]+=((int)navstate->Grid(navstate->GateWest()))*navstate->weight;
			GateState[1]+=((int)navstate->Grid(navstate->GateNorth()))*navstate->weight;
			GateState[2]+=((int)navstate->Grid(navstate->GateEast()))*navstate->weight;

			for (int x=0; x<size_; x++)
				for (int y=0; y<size_; y++)
				{
					if(navstate->rob.x==x && navstate->rob.y==y)
						Beliefmap[x][y]+=navstate->weight;
					Coord pos(x,y);
					ObsBeliefmap[x][y]+=((int)navstate->Grid(pos))*navstate->weight;
						//out << "Weight=" << particles[i]->weight<<endl;
				}
		}

		PrintBeliefMap(Beliefmap,out);
		out << "Map belief:";
		PrintBeliefMap(ObsBeliefmap,out);
		out << "Gate obstacle belief:"<<endl;
		for (int i=0;i<3;i++)
			out<<GateState[i]<<" ";
		out<<endl;
		ClearBeliefMap(Beliefmap);ClearBeliefMap(ObsBeliefmap);
}

} // namespace despot
