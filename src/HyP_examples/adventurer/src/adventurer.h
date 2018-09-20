#ifndef ADVENTURER_H
#define ADVENTURER_H

#include <despot/interface/pomdp.h>
#include <despot/core/mdp.h>
#include <despot/util/coord.h>
#include <despot/solver/pomcp.h>

namespace despot {

/* ==============================================================================
 * AdventurerState class
 * ==============================================================================*/

class AdventurerState: public State {
public:
	AdventurerState();
	AdventurerState(int _state_id);

  std::string text() const;
};

/* ==============================================================================
 * Adventurer class
 * ==============================================================================*/

class Adventurer: public BeliefMDP,
	public MDP,
	public StateIndexer,
	public StatePolicy {
	friend class AdventurerSmartPolicy;
	friend class AdventurerPOMCPPrior;
	friend class AdventurerState;
        friend class GoRightUpperBound;

protected:
	int size_;
	int num_goals_;
  std::vector<double> goal_prob_;
	std::vector<double> goal_reward_;
	double max_goal_reward_;
  std::vector<double> trap_prob_;
	double obs_noise_;
  std::vector<AdventurerState*> states_;

	std::vector<std::vector<std::vector<State> > > transition_probabilities_; //state, action, [state, weight]

	mutable MemoryPool<AdventurerState> memory_pool_;

	std::vector<int> default_action_;

protected:
	void Init(std::istream& is);

	enum {
		A_STAY,
		A_LEFT,
		A_RIGHT
	};

public:
	static Adventurer* current_;

	Adventurer(int num_goals);
	Adventurer(std::string params_file);

	virtual bool Step(State& state, double random_num, int action,
		double& reward, OBS_TYPE& obs) const;
	inline int NumActions() const {
		return 3;
	}
	int NumStates() const;
	inline int GetIndex(const State* state) const {
		return state->state_id;
	}
	inline const State* GetState(int index) const {
		return states_[index];
	}

	virtual double ObsProb(OBS_TYPE obs, const State& state, int action) const;
	const std::vector<State>& TransitionProbability(int s, int a) const;
	double Reward(int s, int a) const;

	State* CreateStartState(std::string type) const;
	virtual Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	inline double GetMaxReward() const {
		return max_goal_reward_;
	}
	ParticleUpperBound* CreateParticleUpperBound(std::string name = "DEFAULT") const;
	ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	inline ValuedAction GetBestAction() const {
		return ValuedAction(0, 0.0);
	}
        int NumObservations() const {
            //Not being used so returning inf
            return std::numeric_limits<int>::max();
        }
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	void PrintAction(int action, std::ostream& out = std::cout) const;

	void PrintTransitions() const;
	void PrintMDPPolicy() const;
	void PrintPOMDPX() const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

	void ComputeDefaultActions(std::string type);
	int GetAction(const State& navistate) const;

	Belief* Tau(const Belief* belief, int action, OBS_TYPE obs) const;
	void Observe(const Belief* belief, int action, std::map<OBS_TYPE, double>& obss) const;
	double StepReward(const Belief* belief, int action) const;

	POMCPPrior* CreatePOMCPPrior(std::string name = "DEFAULT") const;
        
        //=================Hyp Despot Functions With Dummy Implementation=============================
        virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
            return NULL;
        };
        virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const
        {
           std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl; 
        };
        virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
                                                        int num_particles, bool interleave,
                                                        Dvc_RandomStreams* streams, int stream_pos,
                                                        void* CUDAstream = NULL, int shift = 0) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
                                                    bool deepcopy) const {
                  std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
                }
        virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
            return NULL;
        }
        virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void InitGPUModel()
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void InitGPUUpperBound(std::string name,std::string particle_bound_name) const 
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void InitGPULowerBound(std::string name,std::string particle_bound_name) const {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void DeleteGPUModel()
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name) {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void CreateMemoryPool() const 
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }
        virtual void DestroyMemoryPool(MEMORY_MODE mode) const 
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual int ParallelismInStep() const 
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
             return 0;
        }

};

} // namespace despot

#endif
