#include "GPU_Car_Drive.h"
#include <despot/GPUcore/thread_globals.h>

#include <ped_pomdp.h>
#include <despot/util/coord.h>
#include <driver_types.h>
#include <stddef.h>
#include "despot/GPUutil/GPUmemorypool.h"
#include "despot/GPUutil/GPUrandom.h"

#include "GPU_CarUpperBound.h"

#define THREADDIM 128
using namespace std;

using namespace despot;
using namespace Globals;


static GPU_MemoryPool<Dvc_PomdpState>* gpu_mainstate_pool_=NULL;
static GPU_MemoryPool<Dvc_PedStruct>* gpu_ped_pool_=NULL;

static Dvc_PedStruct **Dvc_tempPeds=NULL;
static Dvc_PedStruct **Hst_tempPeds=NULL;

static float** Dvc_temp_weight=NULL;
static int** Hst_temp_IDs=NULL;
static float** Hst_temp_weight=NULL;
static Dvc_PomdpState** Hst_temp_mainstates=NULL;
static Dvc_PomdpState* Managed_rootnode_particles=NULL;


DEVICE Dvc_Path* path=NULL;
DEVICE Dvc_COORD* goals=NULL;
DEVICE double freq=0;
DEVICE double in_front_angle_cos=0;

using namespace despot;
/* ==============================================================================
 * Dvc_PomdpState class
 * ==============================================================================*/

DEVICE Dvc_PomdpState::Dvc_PomdpState():num(0), peds(NULL)
{
}

DEVICE Dvc_PomdpState::Dvc_PomdpState(const Dvc_PomdpState& src)
{
	*this=src;
}


/**
 * CopyPeds_to_Particles kernel:
 * Copy pedestrian states in a combined source list (in contigeous memory) to destination particles
 * This is for copying back to CPU
 */

__global__ void CopyPeds_to_Particles(Dvc_PomdpState* dvc_particles,  Dvc_PedStruct* src)
{
	int scenarioID=blockIdx.x;
	int ped_id=threadIdx.x;

	Dvc_PomdpState* Dvc_i=dvc_particles+scenarioID;
	Dvc_PedStruct* Src_i=src+scenarioID*Dvc_ModelParams::N_PED_IN;

	if(ped_id<Dvc_i->num)
		Dvc_i->peds[ped_id]=Src_i[ped_id];



}


/**
 * CopyPeds_to_list kernel:
 * Copy pedestrian states in particles into a combined contigoues memory list
 * This is for copying back to CPU
 */

__global__ void CopyPeds_to_list(const Dvc_PomdpState* particles,  Dvc_PedStruct* peds_list)
{
	int scenarioID=blockIdx.x;
	int ped_id=threadIdx.x;
	const Dvc_PomdpState* Dvc_i = particles + scenarioID;
	Dvc_PedStruct* Des_i = peds_list + scenarioID * Dvc_ModelParams::N_PED_IN;

	if(ped_id<Dvc_i->num)
		Des_i[ped_id]=Dvc_i->peds[ped_id];
}

HOST void Dvc_PomdpState::CopyMainStateToGPU(Dvc_PomdpState* dvc_particles, int scenarioID, const PomdpState* hst_particle)
{
	dvc_particles[scenarioID].car.dist_travelled=hst_particle->car.dist_travelled;
	dvc_particles[scenarioID].car.pos=hst_particle->car.pos;
	dvc_particles[scenarioID].car.vel=hst_particle->car.vel;
	dvc_particles[scenarioID].num=hst_particle->num;
	dvc_particles[scenarioID].weight=hst_particle->weight;
	dvc_particles[scenarioID].state_id=hst_particle->state_id;
	dvc_particles[scenarioID].scenario_id=hst_particle->scenario_id;


	int Data_block_size=ModelParams::N_PED_IN;

	if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
	{
		memcpy((void*)(Hst_tempPeds[GetCurrentStream()]+Data_block_size*scenarioID),
				(const void*)hst_particle->peds,
				Data_block_size*sizeof(Dvc_PedStruct));
	}
	else
	{
		memcpy((void*)(Hst_tempPeds[0]+Data_block_size*scenarioID),
				(const void*)hst_particle->peds,
				Data_block_size*sizeof(Dvc_PedStruct));
	}
}
HOST void Dvc_PomdpState::CopyPedsToGPU(Dvc_PomdpState* dvc_particles, int NumParticles, bool deep_copy)
{
	if(deep_copy)
	{
		int Data_size=NumParticles*ModelParams::N_PED_IN;
		dim3 grid1(NumParticles,1);dim3 threads1(ModelParams::N_PED_IN,1);
		if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
		{
			HANDLE_ERROR(cudaMemcpyAsync((void*)Dvc_tempPeds[GetCurrentStream()],
					(const void*)Hst_tempPeds[GetCurrentStream()],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyHostToDevice,((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[GetCurrentStream()]));
			logd << "dvc_particles=" << dvc_particles<< ",Dvc_tempPeds[i]=" << Dvc_tempPeds[GetCurrentStream()] <<", GetCurrentStream()="<< GetCurrentStream()<< endl;

			CopyPeds_to_Particles<<<grid1, threads1, 0, ((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[GetCurrentStream()]>>>
					(dvc_particles,Dvc_tempPeds[GetCurrentStream()]);
		}
		else
		{
			HANDLE_ERROR(cudaMemcpy((void*)Dvc_tempPeds[0],
					(const void*)Hst_tempPeds[0],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyHostToDevice));

			logd << "dvc_particles=" << dvc_particles<< ",Dvc_tempPeds[0]=" << Dvc_tempPeds[0]<< endl;
			CopyPeds_to_Particles<<<grid1, threads1>>>(dvc_particles,Dvc_tempPeds[0]);
		}

		//HANDLE_ERROR( cudaDeviceSynchronize());

	}
}

HOST void Dvc_PomdpState::ReadMainStateBackToCPU(const Dvc_PomdpState* dvc_particles, PomdpState* hst_particle)
{
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy((void*)Hst_temp_mainstates[ThreadID], (const void*)dvc_particles, sizeof(Dvc_PomdpState), cudaMemcpyDeviceToHost));
	hst_particle->car.dist_travelled=Hst_temp_mainstates[ThreadID]->car.dist_travelled;
	hst_particle->car.pos=Hst_temp_mainstates[ThreadID]->car.pos;
	hst_particle->car.vel=Hst_temp_mainstates[ThreadID]->car.vel;

	hst_particle->num=Hst_temp_mainstates[ThreadID]->num;
	hst_particle->weight=Hst_temp_mainstates[ThreadID]->weight;
	hst_particle->state_id=Hst_temp_mainstates[ThreadID]->state_id;
	hst_particle->scenario_id=Hst_temp_mainstates[ThreadID]->scenario_id;
}

HOST void Dvc_PomdpState::ReadPedsBackToCPU(const Dvc_PomdpState* dvc_particles,
		std::vector<State*> hst_particles, bool deep_copy)
{
	if(deep_copy)
	{
		int ThreadID=0;
		if(Globals::config.use_multi_thread_)
			ThreadID=Globals::MapThread(this_thread::get_id());

		int NumParticles=hst_particles.size();
		int Data_size=NumParticles*ModelParams::N_PED_IN;
		dim3 grid1(NumParticles,1);dim3 threads1(ModelParams::N_PED_IN,1);
		if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
		{
			CopyPeds_to_list<<<grid1, threads1, 0, ((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]>>>
					(dvc_particles,Dvc_tempPeds[ThreadID]);
			HANDLE_ERROR(cudaMemcpyAsync((void*)Hst_tempPeds[ThreadID],
					(const void*)Dvc_tempPeds[ThreadID],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyDeviceToHost,((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]));
			cudaStreamSynchronize(((cudaStream_t*)StreamManager::MANAGER.cuda_streams)[ThreadID]);
		}
		else
		{
			CopyPeds_to_list<<<grid1, threads1>>>(dvc_particles,Dvc_tempPeds[0]);
			HANDLE_ERROR(cudaMemcpy((void*)Hst_tempPeds[0],
					(const void*)Dvc_tempPeds[0],
					Data_size*sizeof(Dvc_PedStruct),
					cudaMemcpyDeviceToHost));
		}


		int Data_block_size=ModelParams::N_PED_IN;

		for(int i=0;i<NumParticles;i++)
		{
			PomdpState* car_state=static_cast<PomdpState*>(hst_particles[i]);

			if(Globals::config.use_multi_thread_ && StreamManager::MANAGER.cuda_streams)
			{
				memcpy((void*)car_state->peds,
						(const void*)(Hst_tempPeds[ThreadID]+Data_block_size*i),
						Data_block_size*sizeof(Dvc_PedStruct));
			}
			else
			{
				memcpy((void*)car_state->peds,
						(const void*)(Hst_tempPeds[0]+Data_block_size*i),
						Data_block_size*sizeof(Dvc_PedStruct));
			}
		}
	}
}


__global__ void CopyParticles(Dvc_PomdpState* des,Dvc_PomdpState* src,
		float* weight,int* particle_IDs,int num_particles,
		Dvc_RandomStreams* streams, int stream_pos
		)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;

	if(pos==0)
	{
		weight[0]=0;
		if(streams) streams->position_=stream_pos;
	}
	if(pos < num_particles)
	{

		int scenarioID=particle_IDs[pos];
		Dvc_PomdpState* src_i=src+scenarioID;//src is a full length array for all particles
		Dvc_PomdpState* des_i=des+pos;//des is short, only for the new partition

		des_i->car.dist_travelled=src_i->car.dist_travelled;
		des_i->car.pos=src_i->car.pos;
		des_i->car.vel=src_i->car.vel;
		des_i->num=src_i->num;
		des_i->weight=src_i->weight;
		des_i->state_id=src_i->state_id;
		des_i->scenario_id=src_i->scenario_id;

		for(int i=0;i<src_i->num;i++)
		{
			des_i->peds[i].goal=src_i->peds[i].goal;
			des_i->peds[i].id=src_i->peds[i].id;
			des_i->peds[i].pos.x=src_i->peds[i].pos.x;
			des_i->peds[i].pos.y=src_i->peds[i].pos.y;
			des_i->peds[i].vel=src_i->peds[i].vel;
		}

		if(!Dvc_config->track_alpha_vector)
		{
			//Accumulate weight of the particles
			atomicAdd(weight, des_i->weight);
		}
	}
}

void PedPomdp::CreateMemoryPool() const
{
	if(gpu_mainstate_pool_==NULL)
		gpu_mainstate_pool_=new GPU_MemoryPool<Dvc_PomdpState>;
	if(gpu_ped_pool_==NULL)
		gpu_ped_pool_=new GPU_MemoryPool<Dvc_PedStruct>;
}

void PedPomdp::DestroyMemoryPool(MEMORY_MODE mode) const
{
	switch(mode)
	{
		case DESTROY:
			if(gpu_mainstate_pool_){delete gpu_mainstate_pool_;gpu_mainstate_pool_=NULL;}
			if(gpu_ped_pool_){delete gpu_ped_pool_;gpu_ped_pool_=NULL;}
			break;
		case RESET:
			if(gpu_mainstate_pool_ ){ gpu_mainstate_pool_->ResetChuncks();};
			if(gpu_ped_pool_ ){ gpu_ped_pool_->ResetChuncks();};
			break;
	}
}
__global__ void LinkPeds(Dvc_PomdpState* state, Dvc_PedStruct* peds_memory, int numParticles)
{
	for(int i=0;i<numParticles;i++)
	{
		state[i].peds=peds_memory+i*Dvc_ModelParams::N_PED_IN;
	}
}

Dvc_State* PedPomdp::AllocGPUParticles(int numParticles, MEMORY_MODE mode, Dvc_State*** particles_for_all_actions) const
{
	clock_t start=clock();
	dim3 grid((numParticles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);
	int num_threads=1;

	if(Globals::config.use_multi_thread_)
	{
		num_threads = Globals::config.NUM_THREADS;
	}

	Dvc_PedStruct* node_particle_peds;
	switch(mode)
	{
	case INIT:

		CreateMemoryPool();

		/* Intermediate pedestrian container for copying pedestrians in host particles to device particles */
		if(Dvc_tempPeds == NULL && Hst_tempPeds == NULL){
			Dvc_tempPeds=new Dvc_PedStruct*[num_threads];
			Hst_tempPeds=new Dvc_PedStruct*[num_threads];
			for(int i=0;i<num_threads;i++)
			{
				HANDLE_ERROR(cudaMalloc((void**)&Dvc_tempPeds[i],numParticles*ModelParams::N_PED_IN*sizeof(Dvc_PedStruct) ));
				HANDLE_ERROR(cudaHostAlloc((void**)&Hst_tempPeds[i],numParticles*ModelParams::N_PED_IN*sizeof(Dvc_PedStruct),0 ));
			}
		}

		cout<<"numParticles="<<numParticles<<endl;

		if(particles_for_all_actions[0] == NULL){
			particles_for_all_actions[0]=new Dvc_State*[num_threads];
			//Allocate pedestrian memory separately
			Dvc_PedStruct*  peds_tmp=gpu_ped_pool_->Allocate((NumActions()*num_threads)*numParticles*ModelParams::N_PED_IN);

			for(int i=0;i<num_threads;i++)
			{
				HANDLE_ERROR(cudaMalloc((void**)&particles_for_all_actions[0][i],
						NumActions()*numParticles*sizeof(Dvc_PomdpState)));
				//Link pre-allocated pedestrian memory 
				LinkPeds<<<dim3(numParticles,1), dim3(ModelParams::N_PED_IN,1)>>>
						(static_cast<Dvc_PomdpState*>(particles_for_all_actions[0][i]),
						peds_tmp+(NumActions()*i)*numParticles*ModelParams::N_PED_IN,
						NumActions()*numParticles);
			}
			//Record the ped memory used by the pre-allocated lists
			//never reuse these memory for vnode particles
			gpu_ped_pool_->RecordHead();
		}

		/*Intermediate memory for copying particle IDs to device memory 
		cudaHostAlloc enables the copying to interleave with kernel executions*/
		Hst_temp_IDs=new int*[num_threads];
		for(int i=0;i<num_threads;i++)
		{
			if(Globals::config.track_alpha_vector)
			{
				cudaHostAlloc(&Hst_temp_IDs[i],(2+ Globals::config.num_scenarios + Globals::config.num_obs)*NumActions()*sizeof(int),0);
			}
			else
			{
				cudaHostAlloc(&Hst_temp_IDs[i],numParticles*sizeof(int),0);
			}
		}

		/*Intermediate memory for copying weights to device memory. 
		cudaHostAlloc enables the copying to interleave with kernel executions*/

		Hst_temp_weight=new float*[num_threads];
		for(int i=0;i<num_threads;i++)
			cudaHostAlloc(&Hst_temp_weight[i],1*sizeof(float),0);

		Dvc_temp_weight=new float*[num_threads];
		for(int i=0;i<num_threads;i++)
			HANDLE_ERROR(cudaMalloc(&Dvc_temp_weight[i], sizeof(float)));


		/*Intermediate memory for copying main memory of particle (everything except pedestrians) from device back to host
		cudaHostAlloc enables the copying to interleave with kernel executions*/
		Hst_temp_mainstates=new Dvc_PomdpState*[num_threads];

		for(int i=0;i<num_threads;i++)
			HANDLE_ERROR(cudaHostAlloc((void**)&Hst_temp_mainstates[i],1*sizeof(Dvc_PomdpState),0));

		/* No node particle allocated */
		return NULL;

	case ALLOC_ROOT:

		/*Intermediate managed memory for root node particles.
		 * Managed memory enables data copying between CPU and GPU without launching memcpy (which is expensive)
		 */
		HANDLE_ERROR(cudaMallocManaged((void**)&Managed_rootnode_particles, numParticles*sizeof(Dvc_PomdpState)));

		node_particle_peds = gpu_ped_pool_->Allocate(numParticles*ModelParams::N_PED_IN);

		/* Link pedestrian lists to the main memory of particles */
		LinkPeds<<<dim3(numParticles,1), dim3(ModelParams::N_PED_IN,1)>>>(Managed_rootnode_particles, node_particle_peds, numParticles);
		HANDLE_ERROR(cudaDeviceSynchronize());
		return Managed_rootnode_particles;

	case ALLOC:

		/* Allocate vnode particles: main memory and the pedestrian lists */
		Dvc_PomdpState* vnode_particles = gpu_mainstate_pool_->Allocate(numParticles);
		Dvc_PedStruct* vnode_particle_peds = gpu_ped_pool_->Allocate(numParticles*ModelParams::N_PED_IN);

		/* Link pedestrian lists to the main memory of particles */
		LinkPeds<<<dim3(numParticles,1), dim3(ModelParams::N_PED_IN,1)>>>(vnode_particles, vnode_particle_peds, numParticles);
		HANDLE_ERROR(cudaDeviceSynchronize());
		return vnode_particles;
	};


	return NULL;
}


void PedPomdp::CopyGPUParticlesFromParent(Dvc_State* des,Dvc_State* src,int src_offset,
		int* dvc_particle_IDs,int num_particles,bool interleave,
		Dvc_RandomStreams* streams, int stream_pos,
		void* cudaStream, int shift) const
{
	dim3 grid((num_particles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);
	if(num_particles<THREADDIM)
	{
		grid.x=1;grid.y=1;threads.x=num_particles;
	}

	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	if(cudaStream)
	{
		CopyParticles<<<grid, threads,0, *(cudaStream_t*)cudaStream>>>(static_cast<Dvc_PomdpState*>(des),
				static_cast<Dvc_PomdpState*>(src)+src_offset,Dvc_temp_weight[(ThreadID+shift)%Globals::config.NUM_THREADS],
				dvc_particle_IDs,num_particles, streams,stream_pos);
		if(!interleave)
			;
	}
	else
	{
		CopyParticles<<<grid, threads,0, 0>>>(static_cast<Dvc_PomdpState*>(des),
				static_cast<Dvc_PomdpState*>(src)+src_offset,Dvc_temp_weight[ThreadID],
				dvc_particle_IDs,num_particles, streams,stream_pos);
		if(!interleave)
			HANDLE_ERROR(cudaDeviceSynchronize());
	}
}


Dvc_State* PedPomdp::GetPointerToParticleList(int offset,  Dvc_State* full_list) const
{
	return static_cast<Dvc_PomdpState*>(full_list)+ offset;
}
Dvc_State* PedPomdp::CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles, bool deep_copy) const
	//dvc_particles: managed device memory storing particles
	// deep_copy: option on whether to copy list objects in side particles
{

	auto start = Time::now();


	for (int i=0;i<particles.size();i++)
	{
		const PomdpState* src=static_cast<const PomdpState*>(particles[i]);
		Dvc_PomdpState::CopyMainStateToGPU(static_cast<const Dvc_PomdpState*>(dvc_particles),src->scenario_id,src);
	}
	Dvc_PomdpState::CopyPedsToGPU(static_cast<const Dvc_PomdpState*>(dvc_particles),particles.size());

	return dvc_particles;
}

void PedPomdp::CopyParticleIDsToGPU( int* Dvc_ptr, const std::vector<int>& particleIDs, void *cudaStream) const
{
	if(cudaStream)
	{
		int ThreadID=Globals::MapThread(this_thread::get_id());
		memcpy(Hst_temp_IDs[ThreadID],particleIDs.data(),particleIDs.size()*sizeof(int));

		HANDLE_ERROR(cudaMemcpyAsync(Dvc_ptr,Hst_temp_IDs[ThreadID],particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice,*(cudaStream_t*)cudaStream));
	}
	else
	{
		logd << "Dvc_ptr = "<< Dvc_ptr << " particleIDs.size() = " << particleIDs.size()<< " cudaStream = "<< cudaStream<< endl;
		HANDLE_ERROR(cudaMemcpy(Dvc_ptr,particleIDs.data(),particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice));
	}
}


void PedPomdp::DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_for_all_actions ) const
{
	int num_threads=1;

	switch (mode){
	case DESTROY:

		if(Globals::config.use_multi_thread_)
		{
			num_threads=Globals::config.NUM_THREADS;
		}
		for(int i=0;i<num_threads;i++)
		{
			if(particles_for_all_actions[i]!=NULL)
				{HANDLE_ERROR(cudaFree(particles_for_all_actions[i]));particles_for_all_actions[i]=NULL;}
		}
		if(particles_for_all_actions)delete [] particles_for_all_actions;particles_for_all_actions=NULL;
		for(int i=0;i<num_threads;i++)
		{
			cudaFreeHost(Hst_temp_IDs[i]);
		}
		delete [] Hst_temp_IDs;
		for(int i=0;i<num_threads;i++)
		{
			cudaFreeHost(Hst_temp_weight[i]);
		}
		delete [] Hst_temp_weight;
		for(int i=0;i<num_threads;i++)
		{
			cudaFree(Dvc_temp_weight[i]);
		}
		delete [] Dvc_temp_weight;

		for(int i=0;i<num_threads;i++)
		{
			cudaFree(Dvc_tempPeds[i]);
			cudaFreeHost(Hst_tempPeds[i]);
			cudaFreeHost(Hst_temp_mainstates[i]);
		}

		delete [] Dvc_tempPeds;
		delete [] Hst_tempPeds;
		delete [] Hst_temp_mainstates;
		break;
	case RESET:
		HANDLE_ERROR(cudaFree(static_cast<Dvc_PomdpState*>(Managed_rootnode_particles)));

		break;
	};

	DestroyMemoryPool(mode);
}


DEVICE float Dvc_PedPomdpParticleUpperBound1::Value(
		const Dvc_State* particles, int scenarioID, Dvc_History& history) {

	return Dvc_ModelParams::GOAL_REWARD / (1 - Dvc_Globals::Dvc_Discount(Dvc_config));
}



DEVICE bool Dvc_PedPomdp::Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
	int* obs) {

	Dvc_PomdpState& pedpomdp_state = static_cast<Dvc_PomdpState&>(state);//copy contents, link cells to existing ones
	__shared__ int iscollision[32];

	if(FIX_SCENARIO==1 || GPUDoPrint)
		if(GPUDoPrint && pedpomdp_state.scenario_id==PRINT_ID && blockIdx.x==ACTION_ID && threadIdx.y==0){
			printf("(GPU) Before step: scenario=%d \n", pedpomdp_state.scenario_id);
			printf("action= %d\n ",action);
			printf("Before step:\n");
			int pos=pedpomdp_state.car.pos;
			printf("car_pox= %d ",pos);
			printf("trav_dist=%f\n",pedpomdp_state.car.dist_travelled);
			printf("car_vel= %f\n",pedpomdp_state.car.vel);

			for(int i=0;i<pedpomdp_state.num;i++)
			{
				printf("ped %d pox_x= %f pos_y=%f\n",i,
						pedpomdp_state.peds[i].pos.x,pedpomdp_state.peds[i].pos.y);
			}
		}

	bool terminal=false;
	reward = 0;

	unsigned long long int Temp=INIT_QUICKRANDSEED;

	/* Termination checking */
	if(threadIdx.y==0)
	{
		// Terminate upon reaching goal
		if (pedpomdp_state.car.dist_travelled > Dvc_ModelParams::GOAL_TRAVELLED-1e-4
				|| pedpomdp_state.car.pos >= path->size_-1) {
			reward = Dvc_ModelParams::GOAL_REWARD;
			terminal= true;
		}
	}

	/* Collision checking */
	iscollision[threadIdx.x]=false;
	__syncthreads();

	if(!terminal)
	{
		const int car = pedpomdp_state.car.pos;
		const Dvc_COORD& car_pos = path->way_points_[car];
		const Dvc_COORD& forward_pos = path->way_points_[path->forward(car, 1.0)];

		if(threadIdx.y<pedpomdp_state.num){
			const Dvc_COORD& pedpos = pedpomdp_state.peds[threadIdx.y].pos;
			bool collide_ped=false;
			float HNx = forward_pos.x - car_pos.x, // car direction
						 HNy = forward_pos.y - car_pos.y;
			float HMx = pedpos.x - car_pos.x,
						 HMy = pedpos.y - car_pos.y;


/// car geomery for golfcart
/*			double car_width = 0.87,
			car_length = 1.544;

			double safe_margin = 0.92, side_safe_margin = 0.4, back_safe_margin = 0.33,
				 side_margin = car_width / 2.0 + side_safe_margin,
				 front_margin = car_length/2.0 + safe_margin,
				 back_margin = car_length/2.0 + back_safe_margin;
*/
/// end golfcart

/// car geomery for audi r8
			/*double car_width = 2.0,
			 car_length = 4.4;

			 double safe_margin = 0.8, side_safe_margin = 0.35, back_safe_margin = 0.2,
			 side_margin = car_width / 2.0 + side_safe_margin,
			 front_margin = 3.6 + safe_margin,
			 back_margin = 0.8 + back_safe_margin;*/
/// end audi r8


/// car geometry for pomdp car
			double car_width = 1.2,
						 car_length = 2.2;
			double safe_margin = 0.3,
				 side_margin = car_width / 2.0 + safe_margin,
				 front_margin = safe_margin,
				 back_margin = car_length + safe_margin;
/// end pomdp car


			float HLx = - HNy, // direction after 90 degree anticlockwise rotation
						 HLy = HNx;

			float HM_HN = HMx * HNx + HMy * HNy, // HM . HN
						 HN_HN = HNx * HNx + HNy * HNy; // HN . HN
			if (HM_HN >= 0 && HM_HN * HM_HN > HN_HN * front_margin * front_margin)
				collide_ped = false;
			else if (HM_HN <= 0 && HM_HN * HM_HN > HN_HN * back_margin * back_margin)
				collide_ped = false;
			else
			{
			    float HM_HL = HMx * HLx + HMy * HLy, // HM . HL
						 HL_HL = HLx * HLx + HLy * HLy; // HL . HL
			    collide_ped= HM_HL * HM_HL <= HL_HL * side_margin * side_margin;
			}
			atomicOr(iscollision+threadIdx.x, collide_ped);
		}
	}
	__syncthreads(); // Synchronize the block to wait for collision checking with all peds (parallelized in the Y dimemsion) to finish.

	if(threadIdx.y==0 && !terminal)
	{

		/* Terminate if collision is detected */
		if(pedpomdp_state.car.vel > 0.001 && iscollision[threadIdx.x] ) { /// collision occurs only when car is moving
		    reward= Dvc_ModelParams::CRASH_PENALTY *
		    		(pedpomdp_state.car.vel * pedpomdp_state.car.vel +
		    				Dvc_ModelParams::REWARD_BASE_CRASH_VEL);

		    if(action == ACT_DEC) reward += 0.1;

			terminal= true;
		}

		/* Compute reward */
		if(!terminal)
		{
			// Smoothness penalty
			reward += (action == ACT_DEC || action == ACT_ACC) ? -0.1 : 0.0;

			reward += Dvc_ModelParams::REWARD_FACTOR_VEL *
					(pedpomdp_state.car.vel - Dvc_ModelParams::VEL_MAX) / Dvc_ModelParams::VEL_MAX;

			float acc = (action == ACT_ACC) ? Dvc_ModelParams::AccSpeed :
				((action == ACT_CUR) ?  0 : (-Dvc_ModelParams::AccSpeed));

			/* State transition: car */
			float dist = pedpomdp_state.car.vel / freq;
			int nxt = path->forward(pedpomdp_state.car.pos, dist);
			pedpomdp_state.car.pos = nxt;
			pedpomdp_state.car.dist_travelled += dist;

			const float N = Dvc_ModelParams::NOISE_ROBVEL;
			if (N>0) {
				if(FIX_SCENARIO!=1 && !GPUDoPrint)
					rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);

				float prob = rand_num;
				if (prob > N) {
					pedpomdp_state.car.vel += acc / freq;
				}
			} else {
				pedpomdp_state.car.vel += acc / freq;
			}
			pedpomdp_state.car.vel = max(min(pedpomdp_state.car.vel, Dvc_ModelParams::VEL_MAX), 0.0);
		}
	}
	__syncthreads();


	if(!terminal)
	{
		/* State transition: peds */
		if(threadIdx.y<pedpomdp_state.num)
		{
			int i=0;
			while(i<threadIdx.y)
			{
				if(FIX_SCENARIO!=1 && !GPUDoPrint)
					rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
				i++;
			}
			if(threadIdx.y!=0 && FIX_SCENARIO!=1 && !GPUDoPrint)
				rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);

			const Dvc_COORD& goal = goals[pedpomdp_state.peds[threadIdx.y].goal];
			if (abs(goal.x+1)<1e-5 && abs(goal.y+1)<1e-5) {  //stop intention, ped doesn't move
				;
			}
			else
			{
				// Straightline model with Gussian noise on directions
				Dvc_Vector goal_vec(goal.x - pedpomdp_state.peds[threadIdx.y].pos.x, goal.y - pedpomdp_state.peds[threadIdx.y].pos.y);
				float a = goal_vec.GetAngle();
				float noise = sqrt(-2 * log(rand_num));

				if(FIX_SCENARIO!=1 && !GPUDoPrint)
					rand_num=Dvc_QuickRandom::RandGeneration(&Temp, rand_num);
				noise *= cos(2 * M_PI * rand_num)* Dvc_ModelParams::NOISE_GOAL_ANGLE;
				a += noise;

				Dvc_Vector move(a, pedpomdp_state.peds[threadIdx.y].vel/freq, 0);
				pedpomdp_state.peds[threadIdx.y].pos.x += move.dw;
				pedpomdp_state.peds[threadIdx.y].pos.y += move.dh;
			}
		}
	}
	__syncthreads();


	if(threadIdx.y==0 && obs!=NULL)//for each particle in the thread block
	{
		/* generate observations by descretizing the observable part of the state */
		if(!terminal)
		{
			int i=0;
			obs[i++]=2+2*pedpomdp_state.num;
			obs[i++] = int(pedpomdp_state.car.pos);
			obs[i++] = int((pedpomdp_state.car.vel+1e-5) / Dvc_ModelParams::vel_rln);
			for(int j = 0; j < pedpomdp_state.num; j ++) {
				obs[i++] = int(pedpomdp_state.peds[j].pos.x / Dvc_ModelParams::pos_rln);
				obs[i++] = int(pedpomdp_state.peds[j].pos.y / Dvc_ModelParams::pos_rln);
			}
		}
		else
		{
			int i=0;
			obs[i++]=0;
			obs[i++] = 0;
			obs[i++] = 0;
			for(int j = 0; j < pedpomdp_state.num; j ++) {
				obs[i++] = 0;
				obs[i++] = 0;
			}
		}
	}

	if(!terminal && GPUDoPrint && pedpomdp_state.scenario_id==PRINT_ID && blockIdx.x==ACTION_ID && threadIdx.y==0){
		printf("(GPU) After step: scenario=%d \n", pedpomdp_state.scenario_id);
		printf("rand=%f, action=%d \n", rand_num, action);
		printf("After step:\n");
		printf("Reward=%f\n",reward);
		int pos=pedpomdp_state.car.pos;
		printf("car pox= %d ",pos);
		printf("dist=%f\n",pedpomdp_state.car.dist_travelled);
		printf("car vel= %f\n",pedpomdp_state.car.vel);
		for(int i=0;i<pedpomdp_state.num;i++)
		{
			printf("ped %d pox_x= %f pos_y=%f\n",i,
					pedpomdp_state.peds[i].pos.x,pedpomdp_state.peds[i].pos.y);
		}
	}
	return terminal;
}

DEVICE int Dvc_PedPomdp::NumActions() {
	return 3;
}


DEVICE float Dvc_PedPomdp::Dvc_ObsProbInt(int* obs, Dvc_State& state, int action)
{
	//const PomdpState& state = static_cast<const PomdpState&>(s);
	Dvc_PomdpState& pedpomdp_state = static_cast<Dvc_PomdpState&>(state);//copy contents, link cells to existing ones
		//PrintState(state);
		float prob = 1.0;
		float b = 0.0;
		for (int j = 0; j < pedpomdp_state.num; j ++) {
		  b = b + ((obs[2*j + 3]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.x )*((obs[2*j + 3]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.x );
		  b = b + ((obs[2*j + 4]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.y )*((obs[2*j + 4]*Dvc_ModelParams::pos_rln) - pedpomdp_state.peds[j].pos.y );
		  //std::cout << j << " obs vec " << obs[2*j + 2]<< "," << obs[2*j + 3] << ")b= " << b<< std::endl;
		}
		float stddev = 1.0;
		b = - b / (2.0* stddev*stddev);
		//std::cout << "b= " << b << std::endl;
		return expf(b);
}
DEVICE void Dvc_PedPomdp::Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_PomdpState* src_i= static_cast<const Dvc_PomdpState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_PomdpState* des_i= static_cast<const Dvc_PomdpState*>(des)+pos;
	des_i->weight=src_i->weight;
	des_i->scenario_id=src_i->scenario_id;
	des_i->num=src_i->num;
	des_i->car.dist_travelled=src_i->car.dist_travelled;
	des_i->car.pos=src_i->car.pos;
	des_i->car.vel=src_i->car.vel;
	for(int i=0;i< des_i->num;i++)
	{
		des_i->peds[i].vel=src_i->peds[i].vel;
		des_i->peds[i].pos.x=src_i->peds[i].pos.x;
		des_i->peds[i].pos.y=src_i->peds[i].pos.y;
		des_i->peds[i].goal=src_i->peds[i].goal;
		des_i->peds[i].id=src_i->peds[i].id;
	}
}

DEVICE void Dvc_PedPomdp::Dvc_Copy_ToShared(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des) {
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_PomdpState* src_i= static_cast<const Dvc_PomdpState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_PomdpState* des_i= static_cast<const Dvc_PomdpState*>(des)+pos;
	des_i->weight=src_i->weight;
	des_i->scenario_id=src_i->scenario_id;
	des_i->num=src_i->num;
	des_i->car.dist_travelled=src_i->car.dist_travelled;
	des_i->car.pos=src_i->car.pos;
	des_i->car.vel=src_i->car.vel;
	des_i->peds=(Dvc_PedStruct*)((void*)(des_i)+3*sizeof(Dvc_PedStruct));
	for(int i=0;i< des_i->num;i++)
	{
		des_i->peds[i].vel=src_i->peds[i].vel;
		des_i->peds[i].pos.x=src_i->peds[i].pos.x;
		des_i->peds[i].pos.y=src_i->peds[i].pos.y;
		des_i->peds[i].goal=src_i->peds[i].goal;
		des_i->peds[i].id=src_i->peds[i].id;
	}
}
DEVICE Dvc_State* Dvc_PedPomdp::Dvc_Get(Dvc_State* particles, int pos) {
	Dvc_PomdpState* particle_i= static_cast<Dvc_PomdpState*>(particles)+pos;

	return particle_i;
}

DEVICE Dvc_ValuedAction Dvc_PedPomdp::Dvc_GetBestAction() {
	return Dvc_ValuedAction(0,
			Dvc_ModelParams::CRASH_PENALTY * (Dvc_ModelParams::VEL_MAX*Dvc_ModelParams::VEL_MAX + Dvc_ModelParams::REWARD_BASE_CRASH_VEL));
}

void PedPomdp::ReadParticlesBackToCPU(std::vector<State*>& particles ,const Dvc_State* dvc_particles,
			bool deepcopy) const
{
	for (int i=0;i<particles.size();i++)
	{
		const Dvc_PomdpState* src=static_cast<const Dvc_PomdpState*>(dvc_particles)+i;
		PomdpState* des=static_cast<PomdpState*>(particles[i]);
		Dvc_PomdpState::ReadMainStateBackToCPU(src,des);
	}
	Dvc_PomdpState::ReadPedsBackToCPU(
			static_cast<const Dvc_PomdpState*>(dvc_particles),
			particles);
}
