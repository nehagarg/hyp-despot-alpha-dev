#ifndef BASEUncNavigation_H
#define BASEUncNavigation_H

#include <despot/interface/pomdp.h>
#include <despot/solver/pomcp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/util/grid.h>

namespace despot {
#define OBS_NOISE 0.03f
//#define OBS_NOISE 0.0f
#define STEP_NOISE 0.03f
#define POLICY_GRAPH_SIZE 1000
/* =============================================================================
 * UncNavigationState class
 * =============================================================================*/
struct NavCompass {
	enum {
		NORTH, EAST,SOUTH,WEST, NORTHEAST, SOUTHEAST, SOUTHWEST, NORTHWEST
	};

	static const Coord DIRECTIONS[];
	static const std::string CompassString[];
	//static int Opposite(int dir);
	//static bool Opposite(int dir1, int dir2);
};

#define GOAL_REWARD 20

class UncNavigationState: public State {
public:
	UncNavigationState();
	UncNavigationState(int _state_id);

	Coord goal;// goal position
	Coord rob;// robot position
	//int rob_x; int rob_y;
	int sizeX_,sizeY_;// map size
	//bool cells[16];// the map
	bool* cells;//the map

	std::string text() const;

	UncNavigationState(int sizeX, int sizeY);

	UncNavigationState(const UncNavigationState& src);

	void InitCells(int sizeX, int sizeY);

	void Assign(const UncNavigationState& src)
	{
		rob.x=src.rob.x; rob.y=src.rob.y;
		goal=src.goal;
		InitCells(src.sizeX_,src.sizeY_);
		state_id=src.state_id;
		scenario_id=src.scenario_id;
		weight=src.weight;
	    //sizeX_=src.sizeX_;sizeY_=src.sizeY_;
	   // if(cells==NULL)
	    //	cells=new bool[sizeX_*sizeY_];
		//for (int i=0;i<sizeX_;i++)
		//	for (int j=0;j<sizeY_;j++)
		//		cells[j*sizeX_+i]=src.Grid(i,j);
		memcpy((void*)cells, (const void*)src.cells, sizeX_*sizeY_*sizeof(bool));
	}

	UncNavigationState& operator=(const UncNavigationState& other) // copy assignment
	{
	    if (this != &other) { // self-assignment check expected
                         // storage can be reused
	    	Assign(other);
	    }
	    return *this;
	}

	bool Grid(int posX, int posY) const
	{
		bool result=false;
		if(Inside(posX,posY))
			result=cells[posY*sizeX_+posX];
		else
			result =true;//outside the map
		return result;
	}
	bool Grid(const Coord& pos) const
	{
		bool result=false;
		if(Inside(pos))
			result=cells[pos.y*sizeX_+pos.x];
		else
		{
			result=true;
			//exit(1);
		}
		return result;
	}
	bool GridStrict(const Coord& pos) const
	{
		bool result=false;
		if(Inside(pos))
			result=cells[pos.y*sizeX_+pos.x];
		else
		{
			std::cout<<__FUNCTION__<<":Trying to access invalid position of the map"<<std::endl;
			//exit(1);
		}
		return result;
	}
	bool& GridOpen(const Coord& pos)
	{
		if(Inside(pos))
			return cells[pos.y*sizeX_+pos.x];
		else
		{
			std::cerr<<__FUNCTION__<<":Trying to access invalid position of the map";
			exit(1);
		}
	}
	bool Inside(const Coord& coord) const {
		return coord.x >= 0 && coord.y >= 0 && coord.x < sizeX_
			&& coord.y < sizeY_;
	}
	bool Inside(int posX, int posY) const {
		return posX >= 0 && posY >= 0 && posX < sizeX_
			&& posY < sizeY_;
	}
	bool CollisionCheck(Coord &rob_pos) const
	{
		return Grid(rob_pos);
	}
	void RandomGoal()
	{
		goal=Coord(Random::RANDOM.NextInt(sizeX_),
				Random::RANDOM.NextInt(sizeY_));
	}
	void FixedGoal()
	{
		goal.x=sizeX_/2;
		goal.y=0;
	}
	Coord GateNorth() const
	{
		Coord pos(sizeX_/2,1);
		return pos;
	}
	Coord GateEast() const
	{
		Coord pos(sizeX_/2-1,0);
		return pos;
	}
	Coord GateWest() const
	{
		Coord pos(sizeX_/2+1,1);
		return pos;
	}

	~UncNavigationState();


};

/* =============================================================================
 * BaseUncNavigation class
 * =============================================================================*/

class BaseUncNavigation:public DSPOMDP {
	/*friend class UncNavigationENTScenarioLowerBound;
	friend class UncNavigationMMAPStateScenarioLowerBound;
	friend class UncNavigationEastScenarioLowerBound;
	friend class UncNavigationParticleUpperBound1;
	friend class UncNavigationParticleUpperBound2;
	friend class UncNavigationMDPParticleUpperBound;
	friend class UncNavigationApproxParticleUpperBound;
	friend class UncNavigationEastBeliefPolicy;
	friend class UncNavigationMDPBeliefUpperBound;
	friend class UncNavigationPOMCPPrior;*/

protected:
	//Grid<int> grid_; // the map
	//std::vector<Coord> obstacle_pos_;// obstacles' pos on the map
	int size_, num_obstacles_;// size of map, number of obstacles
	//Coord start_pos_,goal_pos_;// robot start pos, goal pos
	double half_efficiency_distance_;

	//UncNavigationState* obstacle_state_;//rock good or bad
	mutable MemoryPool<UncNavigationState> memory_pool_;

	std::vector<UncNavigationState*> states_;

protected:
	void InitGeneral();
	/*void Init_4_4();
	void Init_5_5();
	void Init_5_7();
	void Init_7_8();
	void Init_11_11();*/
	void InitStates();
	OBS_TYPE GetObservation(double rand_num, const UncNavigationState& navstate) const;

	std::vector<std::vector<std::vector<State> > > transition_probabilities_;
	std::vector<std::vector<double> > alpha_vectors_; // For blind policy
	mutable std::vector<ValuedAction> mdp_policy_;

public:
	enum OBS_enum{ //State flag of cells in the map. FRAGILE: Don't change!
		E_FN_FE_FS_FW = 0,
		E_FN_FE_FS_OW = 1,
		E_FN_FE_OS_FW = 2,
		E_FN_FE_OS_OW = 3,
		E_FN_OE_FS_FW = 4,
		E_FN_OE_FS_OW = 5,
		E_FN_OE_OS_FW = 6,
		E_FN_OE_OS_OW = 7,
		E_ON_FE_FS_FW = 8,
		E_ON_FE_FS_OW = 9,
		E_ON_FE_OS_FW = 10,
		E_ON_FE_OS_OW = 11,
		E_ON_OE_FS_FW = 12,
		E_ON_OE_FS_OW = 13,
		E_ON_OE_OS_FW = 14,
		E_ON_OE_OS_OW = 15,
	};

	/*enum { //Actions of the robot. FRAGILE: Don't change!
		E_STAY = 4,
		E_SOUTH = 2,
		E_EAST = 1,
		E_WEST = 3,
		E_NORTH = 0,
	};*/


	enum { //Actions of the robot. FRAGILE: Don't change!
		E_STAY = 8,
		E_SOUTH = 2,
		E_EAST = 1,
		E_WEST = 3,
		E_NORTH = 0,
		//"NE", "SE", "SW", "NW"
		E_SOUTH_EAST = 5,
		E_SOUTH_WEST = 6,
		E_NORTH_EAST = 4,
		E_NORTH_WEST = 7,
	};


public:
	BaseUncNavigation(std::string map);
	BaseUncNavigation(int size, int obstacles);
	~BaseUncNavigation()
	{
		DestroyMemoryPool(despot::MEMORY_MODE(DESTROY));
		FreeObstacles();
	}

	virtual bool Step(State& state, double rand_num, int action,
		double& reward, OBS_TYPE& obs) const = 0;
	virtual int NumActions() const = 0;
	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const = 0;

	//const std::vector<State>& TransitionProbability(int s, int a) const;
	UncNavigationState NextState(UncNavigationState& s, int a) const;
	double Reward(UncNavigationState& s, int a) const;

	//State* CreateStartState(std::string type = "DEFAULT") const;
	State* CreateStartState(std::string type = "DEFAULT") const;
	void RandGate(UncNavigationState* nav_state) const;
	void RandMap(UncNavigationState* nav_state, float ObstacleProb, int skip) const;
	void RandMap(UncNavigationState* nav_state, float prob, bool draw_mid_line) const;

	/*std::vector<State*> InitialParticleSet() const;
	std::vector<State*> NoisyInitialParticleSet() const;*/
	Belief* InitialBelief(const State* start, std::string type = "PARTICLE") const;

	inline double GetMaxReward() const {
		return /*10*/GOAL_REWARD;
	}
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	/*BeliefUpperBound* CreateBeliefUpperBound(std::string name = "DEFAULT") const;
*/
	inline ValuedAction GetBestAction() const {
		//return ValuedAction(E_NORTH, -1);
		return ValuedAction(E_STAY, -0.1);
	}
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;
	/*BeliefLowerBound* CreateBeliefLowerBound(std::string name = "DEFAULT") const;

	POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;*/

	void AllocBeliefMap(float**& map) const;
	void ClearBeliefMap(float**& map) const;
	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void ExportState(const State& state, std::ostream& out = std::cout) const;
	State* ImportState(std::istream& in) const;
	void ImportStateList(std::vector<State*>& particles, std::istream& in) const;

	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	void PrintBeliefMap(float** map,std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE observation, std::ostream& out = std::cout) const = 0;
	void PrintAction(int action, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	virtual Dvc_State* AllocGPUParticles(int numParticles,int alloc_mode) const;
	//virtual void CopyGPUParticles(Dvc_State* des,Dvc_State* src,int src_offset,int* IDs,int num_particles,bool interleave, cudaStream_t* CUDAstream=NULL) const;
/*
#ifndef __CUDACC__
	virtual void CopyGPUParticles(Dvc_State* des,Dvc_State* src,int src_offset,int* IDs,int num_particles,bool interleave) const
	{}
#else
*/
	virtual void CopyGPUParticles(Dvc_State* des,Dvc_State* src,int src_offset,
			int* IDs,int num_particles,bool interleave,
			Dvc_RandomStreams* streams, int stream_pos,
			void* CUDAstream=NULL, int shift=0) const;
//#endif
	virtual void CopyGPUWeight1(void* cudaStream, int shift=0) const;
	virtual float CopyGPUWeight2(void* cudaStream, int shift=0) const;
	virtual Dvc_State* CopyToGPU(const std::vector<State*>& particles , bool copy_cells) const;
	//virtual int* CopyToGPU( const std::vector<int>& particleIDs, cudaStream_t* CUDAstream=NULL) const;
	void ReadBackToCPU(std::vector<State*>& particles ,const Dvc_State* parent_particles,
			bool deepcopy) const;

	//#ifndef __CUDACC__
	virtual void CopyToGPU(const std::vector<int>& particleIDs,int* Dvc_ptr, void* CUDAstream=NULL) const;

	virtual void CreateMemoryPool(int chunk_size) const;
	virtual void DestroyMemoryPool(int mode) const;
/*
	{}
#else
	virtual int* CopyToGPU(const std::vector<State*>& particles, const std::vector<int>& particleIDs, cudaStream_t* CUDAstream=NULL) const;
#endif
*/
	virtual void DeleteGPUParticles( int num_particles) const;
	virtual void DeleteGPUParticles(Dvc_State* particles, int num_particles) const;
	virtual Dvc_State* GetGPUParticles() const;

	void Free(State* particle) const;
	int NumActiveParticles() const;

	Belief* Tau(const Belief* belief, int action, OBS_TYPE obs) const;
	//void Observe(const Belief* belief, int action, std::map<OBS_TYPE, double>& obss) const;
	//double StepReward(const Belief* belief, int action) const;

	virtual int NumObservations() const;
	virtual int ParallelismInStep() const{return 1;}
	//const State* GetState(int index) const;
	//int GetIndex(const State* state) const;

	//inline int GetAction(const State& navstate) const {
	//	return 0;
	//}

	//int GetRobPosIndex(const State* state) const;
	Coord GetRobPos(const State* state) const;
	//bool Getobstacle(const State* state, int obstacle) const;
	//void Sampleobstacle(State* state, int obstacle) const;
	int GetX(const UncNavigationState* state) const;
	void IncX(UncNavigationState* state) const;
	void DecX(UncNavigationState* state) const;
	int GetY(const UncNavigationState* state) const;
	void IncY(UncNavigationState* state) const;
	void DecY(UncNavigationState* state) const;

	void UniformRobPos(UncNavigationState* startState) const;
	void UniUpperRobPos(UncNavigationState* startState) const;
	void AreaRobPos(UncNavigationState* startState, int area_size) const;
	void LineRobPos(UncNavigationState* startState, int line, int width) const;
	void CalObstacles(float prob,UncNavigationState* nav_state=NULL, bool draw_mid_line=false) const;
	void CalFixObstacles(float perc,UncNavigationState* nav_state=NULL, bool draw_mid_line=false) const;
	void CalFixLineObstacles(int line_pos, int num_gates,UncNavigationState* nav_state=NULL) const;
	void CalGateObstacles(int line_pos, int num_gates,int num_opens,UncNavigationState* nav_state=NULL) const;
	void FreeObstacles() const;
protected:
	//void InitializeTransitions();
	//Coord IndexToCoord(int pos) const;
	//int CoordToIndex(Coord c) const;
	//std::vector<ValuedAction>& ComputeOptimalSamplingPolicy() const;
	//UncNavigationState* MajorityUncNavigationState(const std::vector<State*>& particles) const;
};

} // namespace despot

#endif
