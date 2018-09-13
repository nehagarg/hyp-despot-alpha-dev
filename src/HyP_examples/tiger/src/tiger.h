#ifndef TIGER_H
#define TIGER_H

#include <despot/interface/pomdp.h>

namespace despot {

/* =============================================================================
 * TigerState class
 * =============================================================================*/

class TigerState: public State {
public:
	int tiger_position;

	TigerState();

	TigerState(int position);

	std::string text() const;
};

/* =============================================================================
 * Tiger class
 * =============================================================================*/

class Tiger: public DSPOMDP {
protected:
	mutable MemoryPool<TigerState> memory_pool_;

public:
	static const int LEFT, RIGHT, LISTEN;
	static const double NOISE;

	Tiger();
	Tiger(std::string params_file);

	virtual bool Step(State& s, double random_num, int action, double& reward,
		OBS_TYPE& obs) const;
	int NumStates() const;
	virtual int NumActions() const;
	int NumObservations() const;
	virtual double ObsProb(OBS_TYPE obs, const State& s, int a) const;

	State* CreateStartState(std::string type) const;
	Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

	inline double GetMaxReward() const {
		return 10;
	}

	inline ValuedAction GetBestAction() const {
		return ValuedAction(LISTEN, -1);
	}
	ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT",
		std::string particle_bound_name = "DEFAULT") const;

	void PrintState(const State& state, std::ostream& out = std::cout) const;
	void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
	virtual void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
	virtual void PrintAction(int action, std::ostream& out = std::cout) const;

	State* Allocate(int state_id, double weight) const;
	State* Copy(const State* particle) const;
	void Free(State* particle) const;
	int NumActiveParticles() const;

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
