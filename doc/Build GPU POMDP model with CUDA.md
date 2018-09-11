# Building GPU POMDP model for HyP-DESPOT

HyP-DESPOT requires a GPU counterpart of the POMDP model defined by the **DSPOMDP** class 
in order to perform parallel expansions and rollouts in the GPU.

## GPU POMDP model class
A template for such a GPU model is provided in [GPUinterface/GPUpomdp.h](src/HypDespot/include/despot/GPUinterface/GPUpomdp.h).

```
class Dvc_DSPOMDP {
public:
	/* ========================================================================
	 * Deterministic simulative model and related functions
	 * ========================================================================*/
	/**
	 * Determistic simulative model for POMDP.
	 * 
	 * The function in your custom POMDP model should be:
	 */
   
	 DEVICE static bool Dvc_Step(Dvc_State& state, float random_num, ACT_TYPE action,
	  	   double& reward, OBS_TYPE& obs);

	/**
	 * Determistic simulative model for POMDP.
	 * Used when the raw observation is an integer array (like the car driving problem)
	 *
	 * The function in your custom POMDP model should be:
   */

	 DEVICE static bool Dvc_Step_IntObs(Dvc_State& state, float random_num, ACT_TYPE action, 
	    	float& reward, int* obs);
	 
	/* ========================================================================
	 * Action
	 * ========================================================================*/
	/**
	 * Returns number of actions.
	 * 
	 * The function in your custom POMDP model should be:
	 */

   DEVICE static int NumActions();

	/* ========================================================================
	 * Memory management.
	 * ========================================================================*/

	/**
	 * Copy the state from a particle at entry pos in src list to a particle at entry pos (or 0 when offset_des is false) in des list.
	 * Used when both lists have already been allocated and reside in the global memory of GPU
	 *
	 * The function in your custom POMDP model should be:
	 */
   
   DEVICE static void Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des);
	
  /**
	 * Copy the state from a particle at entry pos in src list to a particle at entry pos (or 0 when offset_des is false) in des list.
	 * des list resides in the shared memory of GPU as contiguous list.
	 * !! Only contiguous memory is allowed in shared memory (pointer-linked members in class should be specially treated) !!
	 *
	 * The function in your custom POMDP model should be: 
	 */

   DEVICE static void Dvc_Copy_ToShared(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des);
  
	/**
	 * Returns the pointer to a particle at pos in the list.
	 *
	 * The function in your custom POMDP model should be:
	 */
   
   DEVICE static Dvc_State* Dvc_Get(const Dvc_State* particles, int pos);

	/* ========================================================================
	 * Bound-related functions.
	 * ========================================================================*/

	/**
	 * Returns the action that provides the best worst-case reward.
	 *
	 * The function in your custom POMDP model should be:
	 */
   
   DEVICE static Dvc_ValuedAction Dvc_GetBestAction();
   
	/**
	 * Returns the maximum reward.
	 *
	 * The function in your custom POMDP model should be: 
	 */
   
   DEVICE static float Dvc_GetMaxReward();
};

```c++
Custom GPU POMDP classes can be defined following this template. There is no need to inherit the *Dvc_DSPOMDP* class.

## Extentions in the DSPOMDP class

HyP-DESPOT requires additional functions in the DSPOMDP class to communicate with the GPU POMDP model.

```
class DSPOMDP {
public:
	/* ========================================================================
	 * Existing functions in DESPOT
	 * ========================================================================*/

  ...
  
	/* ========================================================================
	 * Extended functions in HyP-DESPOT
	 * ========================================================================*/

  /**
	 * Allocate amount=numParticles GPU particles according to mode. 
	 * mode can be:
   * INIT: used in the initialization stage of the program. Allocate common GPU memories used in all following searches (like particles_all_a). 
	 * ALLOC_ROOT: used when preparing the root of the HyP-DESPOT tree. Allocate the GPU particle list for the root.
   * ALLOC: used when expanding a non-root node in the HyP-DESPOT tree. Allocate GPU particles for the node using the memory pool.
	 */
   
 	 virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const = 0;
  
  /**
	 * Delete GPU particles according to mode. 
	 * mode can be:
   * DESTROY: used in the exiting stage of the program. Release common GPU memories used in all following searches (like particles_all_a). 
	 * RESET: used when resetting the HyP-DESPOT tree. Release the GPU particles memory pool.
	 */
   
	 virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const = 0;
   
  /**
	 * Copy GPU particles to the current node from its parent node.
   * @param des: destination particles (in the current node)
	 * @param src: source particles (in the parent node)
   * @param src_offset: offset of the starting position in the src list
   * @param IDs: ids of the destination particles with respect to the src list
   * @param num_particles: number of particles in the destination list
   * @param interleave: whether to interleave the copying process inside GPU with the CPU computation
   * @param streams: random streams attached to GPU scenarios.
   * @param stream_pos: current iterator position of the random streams (same as the depth of the current node) 
   * @param CUDAstream: (concurrent kernels) which CUDA stream in the GPU to use for computation in the function. 	 
   * @param shift: shift of start position in the destination particle list
   */
   
	 virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
	                                        int num_particles, bool interleave,
	                                        Dvc_RandomStreams* streams, int stream_pos,
	                                        void* CUDAstream = NULL, int shift = 0) const = 0;
  
  /**
	 * Copy CPU particles to a GPU particle list.
   * @param dvc_particles: destination particle list in GPU
	 * @param particles: source particles in CPU
   * @param deep_copy: if the particles contain pointers that link to other memory, whether to copy the linked memory.
   */
   
	virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const = 0;

	virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const = 0;

	virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
	                                    bool deepcopy) const {
		std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
	}

	virtual void InitGPUModel() = 0;

	virtual void InitGPUUpperBound(std::string name,	std::string particle_bound_name) const = 0;

	virtual void InitGPULowerBound(std::string name,	std::string particle_bound_name) const = 0;

	virtual void DeleteGPUModel() = 0;

	virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name) = 0;

	virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) = 0;

	virtual void CreateMemoryPool() const = 0;
	virtual void DestroyMemoryPool(MEMORY_MODE mode) const = 0;

	virtual int ParallelismInStep() const=0;

};
```c++

