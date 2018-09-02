#include "GPU_base_ma_rock_sample.h"

#include "base/base_ma_rock_sample.h"
#include "GPU_ma_rock_sample.h"
#include "despot/GPUutil/GPUutil.h"
#include "despot/solver/GPUdespot.h"

using namespace std;

namespace despot {

#define THREADDIM 128

DEVICE int ma_map_size_=NULL;
DEVICE int ma_num_rocks_=NULL;
DEVICE double ma_half_efficiency_distance_=NULL;
DEVICE int* ma_grid_=NULL;/*A flattened pointer of a 2D map*/
DEVICE DvcCoord* ma_rock_pos_=NULL;
DEVICE int num_agents_=NULL;
DEVICE Dvc_MultiAgentRockSample* ma_rs_model_=NULL;
DEVICE int ma_Dvc_policy_size_=0;
DEVICE Dvc_ValuedAction* ma_Dvc_policy_=NULL;

Dvc_State** Hst_stepped_particles_all_a=NULL;

static GPU_MemoryPool<Dvc_MARockSampleState>* gpu_memory_pool_=NULL;
static Dvc_MARockSampleState* Dvc_particles=NULL;
//static int* Dvc_particleIDs=NULL;
static Dvc_MARockSampleState** tmp=NULL;
static float** tmp_result=NULL;
static int** tempHostID;
static float** temp_weight;

/* ==============================================================================
 * MARockSampleState class
 * ==============================================================================*/

DEVICE Dvc_MARockSampleState::Dvc_MARockSampleState() {
	joint_pos = 0;
}

HOST void Dvc_MARockSampleState::CopyToGPU(Dvc_MARockSampleState* Dvc, int scenarioID, const MARockSampleState* Hst, bool copy_cells)
{
	Dvc[scenarioID].weight=Hst->weight;
	//Dvc[scenarioID].allocated_=Hst->allocated_;
	Dvc[scenarioID].state_id=Hst->state_id;
	Dvc[scenarioID].joint_pos=Hst->joint_pos;
	Dvc[scenarioID].scenario_id=Hst->scenario_id;
}
HOST void Dvc_MARockSampleState::ReadBackToCPU(const Dvc_MARockSampleState* Dvc, MARockSampleState* Hst, bool copy_cells)
{
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy((void*)tmp[ThreadID], (const void*)Dvc,
			sizeof(Dvc_MARockSampleState), cudaMemcpyDeviceToHost));

	Hst->weight=tmp[ThreadID]->weight;
	//Hst->allocated_=tmp[ThreadID]->allocated_;
	Hst->state_id=tmp[ThreadID]->state_id;
	Hst->joint_pos=tmp[ThreadID]->joint_pos;
	Hst->scenario_id=tmp[ThreadID]->scenario_id;
}


DEVICE DvcCoord Dvc_MultiAgentRockSample::GetCoord(int index)
{
	assert(index >= 0 && index < ma_map_size_ * ma_map_size_);
	return DvcCoord(index % ma_map_size_, index / ma_map_size_);
}
/*DEVICE DvcCoord Dvc_MultiAgentRockSample::GetRobPos(const Dvc_State* state, int rid) {
	return GetCoord(GetRobPosIndex(state, rid));
}*/

DEVICE bool Dvc_MultiAgentRockSample::GetRock(const Dvc_State* state, int rock) {
	return Dvc_CheckFlag(state->state_id, rock);
}

DEVICE int Dvc_MultiAgentRockSample::GetX(const Dvc_MARockSampleState* state, int rid) {
	return GetRobPosIndex(state,rid) % ma_map_size_;
}

DEVICE void Dvc_MultiAgentRockSample::IncX(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos+(1 << (num_agents_-1-rid)*MAX_COORD_BIT);
}

DEVICE void Dvc_MultiAgentRockSample::DecX(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos-(1 << (num_agents_-1-rid)*MAX_COORD_BIT);
}

DEVICE int Dvc_MultiAgentRockSample::GetY(const Dvc_MARockSampleState* state, int rid) {
	return GetRobPosIndex(state,rid) / ma_map_size_;
}

DEVICE void Dvc_MultiAgentRockSample::IncY(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos+(1 << (num_agents_-1-rid)*MAX_COORD_BIT) * ma_map_size_;
}

DEVICE void Dvc_MultiAgentRockSample::DecY(Dvc_MARockSampleState* state, int rid) {
	state->joint_pos=state->joint_pos -(1 << (num_agents_-1-rid)*MAX_COORD_BIT) * ma_map_size_;
}

DEVICE int Dvc_MultiAgentRockSample::GetRobPosIndex(const Dvc_MARockSampleState* state, int rid) {
	return (state->joint_pos >> (num_agents_-1-rid)*MAX_COORD_BIT) & COORD_BIT_MASK;
}

DEVICE int Dvc_MultiAgentRockSample::GetRobPosIndex(int joint_pos_id, int rid) {
	return (joint_pos_id >> (num_agents_-1-rid)*MAX_COORD_BIT) & COORD_BIT_MASK;
}

DEVICE int Dvc_MultiAgentRockSample::SetRobPosIndex(int& p, int rid, int new_pos) {
	assert(p<(1 << (num_agents_*MAX_COORD_BIT)));
	//atomicAdd(&s, (new_pos -GetRobPosIndex(s, rid)) << (ma_num_rocks_+(num_agents_-1-rid)*MAX_COORD_BIT));
	p=p+((new_pos -GetRobPosIndex(p, rid)) << ((num_agents_-1-rid)*MAX_COORD_BIT));

	return p;
}

DEVICE DvcCoord Dvc_MultiAgentRockSample::GetRobPos(const Dvc_MARockSampleState* state, int rid) {
	return GetCoord(GetRobPosIndex(state, rid));
}

DEVICE int Dvc_MultiAgentRockSample::GetRobAction(int action, int rid)
{
	//return (action >>rid*MAX_ACTION_BIT)& ACTION_BIT_MASK;
	if(rid==0)
		return action % (ma_num_rocks_+5);
	else if(rid>0)
		return (action % (int)(pow((float)(ma_num_rocks_+5), rid+1))) /(int)(pow((float)(ma_num_rocks_+5), rid));
}

DEVICE int Dvc_MultiAgentRockSample::GetRobObs(OBS_TYPE obs, int rid)
{
	return (obs >>rid*MAX_OBS_BIT)& OBS_BIT_MASK;
}

DEVICE void Dvc_MultiAgentRockSample::SetRobObs(OBS_TYPE& obs, int rob_obs, int rid)
{
	//atomicAdd(&obs, ((rob_obs -GetRobObs(obs, rid)) << (rid*MAX_OBS_BIT)));
	obs= obs + ((OBS_TYPE)(rob_obs -GetRobObs(obs, rid)) << (rid*MAX_OBS_BIT));
}

DEVICE void Dvc_MultiAgentRockSample::SampleRock(Dvc_State* state, int rock) {
	Dvc_UnsetFlag(state->state_id, rock);
}


DEVICE float Dvc_MARockSampleApproxParticleUpperBound::Value(const Dvc_State* particles, int scenarioID, Dvc_History& history)
{
	const Dvc_MARockSampleState* rs_state =
		static_cast<const Dvc_MARockSampleState*>(particles) + scenarioID;
	float value = 0;
	float discount = 1.0;
	for(int rid=0; rid< num_agents_;rid++)
	{
		if(ma_rs_model_->GetRobPosIndex(rs_state, rid)!=ROB_TERMINAL_ID){
			DvcCoord rob_pos = ma_rs_model_->GetRobPos(rs_state, rid);
			bool visited[NUM_ROCKS];
			for (int rock = 0; rock < ma_num_rocks_; rock++)
				visited[rock]=false;
			while (true) {
				// Move to the nearest valuable rock and sample
				int shortest = 2 * ma_map_size_;
				int id = -1;
				DvcCoord rock_pos(-1, -1);
				for (int rock = 0; rock < ma_num_rocks_; rock++) {
					int dist = DvcCoord::ManhattanDistance(rob_pos,
						ma_rock_pos_[rock]);
					if (Dvc_CheckFlag(rs_state->state_id, rock) && dist < shortest
						&& !visited[rock]) {
						shortest = dist;
						id = rock;
						rock_pos = ma_rock_pos_[rock];
					}
				}

				if (id == -1)
					break;

				discount *= Dvc_Globals::Dvc_Discount(Dvc_config, DvcCoord::ManhattanDistance(rock_pos, rob_pos));
				value += discount * 10.0;
				visited[id] = true;
				rob_pos = rock_pos;
			}

			value += 10.0 * discount
				* Dvc_Globals::Dvc_Discount(Dvc_config,ma_map_size_ - ma_rs_model_->GetX(rs_state, rid));
		}
	}
	return value;
}
DEVICE float Dvc_MARockSampleMDPParticleUpperBound::Value(const Dvc_State* particles, int scenarioID, Dvc_History& history) {
	const Dvc_MARockSampleState* rs_state =
		static_cast<const Dvc_MARockSampleState*>(particles) + scenarioID;
	return ma_Dvc_policy_[rs_state->state_id].value;
}

DEVICE float Dvc_MARockSampleTrivialParticleUpperBound::Value(const Dvc_State* particles, int scenarioID, Dvc_History& history) {
		return 10 / (1 - Dvc_Globals::Dvc_Discount(Dvc_config));
}


DEVICE Dvc_ValuedAction Dvc_MARockSampleEastScenarioLowerBound::Value(
		Dvc_State* particles,
		Dvc_RandomStreams& streams,
		Dvc_History& history, int dummy) {
	const Dvc_MARockSampleState* rs_state =
			static_cast<const Dvc_MARockSampleState*>(particles);
	float value=0;
	for(int rid=0; rid< num_agents_;rid++){
		if(ma_rs_model_->GetRobPosIndex(rs_state, rid)!=ROB_TERMINAL_ID){
			value +=10 * Dvc_Globals::Dvc_Discount(Dvc_config,
				ma_map_size_ - ma_rs_model_->GetX(rs_state, rid) - 1);
		}
	}
	return Dvc_ValuedAction(Dvc_Compass::EAST*(ma_num_rocks_+5)+Dvc_Compass::EAST,value);
}
/*
class RockSampleEastScenarioLowerBound : public ScenarioLowerBound {
private:
	const BaseMultiAgentRockSample* ma_rs_model_;
	const Grid<int>& ma_grid_;

public:
	RockSampleEastScenarioLowerBound(const DSPOMDP* model) :
		ScenarioLowerBound(model),
		ma_rs_model_(static_cast<const BaseMultiAgentRockSample*>(model)),
		ma_grid_(ma_rs_model_->ma_grid_) {
	}

	Dvc_ValuedAction Value(const vector<State*>& particles, Dvc_RandomStreams& streams,
		Dvc_History& history) const {
		return Dvc_ValuedAction(Compass::EAST,
			10 * State::Weight(particles)
				* Dvc_Globals::Dvc_Discount(Dvc_config,ma_grid_.xsize() - ma_rs_model_->GetX(particles[0]) - 1));
	}
};

class RockSampleMMAPStateScenarioLowerBound : public ScenarioLowerBound {
private:
	const BaseMultiAgentRockSample* ma_rs_model_;
	const Grid<int>& ma_grid_;
	vector<vector<int> > rock_order_;

public:
	RockSampleMMAPStateScenarioLowerBound(const DSPOMDP* model) :
		ScenarioLowerBound(model),
		ma_rs_model_(static_cast<const BaseMultiAgentRockSample*>(model)),
		ma_grid_(ma_rs_model_->ma_grid_) {
		const vector<Dvc_ValuedAction> policy =
			ma_rs_model_->ComputeOptimalSamplingPolicy();
		rock_order_ = vector<vector<int> >(policy.size());
		for (int s = 0; s < policy.size(); s++) {
			int cur = s;
			while (cur != policy.size() - 1) {
				int action = policy[cur].action;
				if (action == ma_rs_model_->E_SAMPLE) {
					int rock = ma_grid_(
						ma_rs_model_->IndexToCoord(cur >> ma_num_rocks_));
					if (rock < 0)
						exit(0);
					rock_order_[s].push_back(rock);
				}
				cur = ma_rs_model_->NextState(cur, action);
			}
		}
	}

	Dvc_ValuedAction Value(const vector<State*>& particles, Dvc_RandomStreams& streams,
		Dvc_History& history) const {
		vector<double> expected_sampling_value = vector<double>(
			ma_num_rocks_);
		int state = 0;
		DvcCoord rob_pos(-1, -1);
		double total_weight = 0;
		for (int i = 0; i < particles.size(); i++) {
			State* particle = particles[i];
			state = (1 << ma_num_rocks_)
				* ma_rs_model_->GetRobPosIndex(particle);
			rob_pos = ma_rs_model_->GetRobPos(particle);
			for (int r = 0; r < ma_num_rocks_; r++) {
				expected_sampling_value[r] += particle->weight
					* (Dvc_CheckFlag(particle->state_id, r) ? 10 : -10);
			}
			total_weight += particle->weight;
		}

		for (int rock = 0; rock < ma_num_rocks_; rock++)
			if (expected_sampling_value[rock] / total_weight > 0.0)
				SetFlag(state, rock);

		int action = -1;
		double value = 0;
		double discount = 1.0;
		for (int r = 0; r < rock_order_[state].size(); r++) {
			int rock = rock_order_[state][r];
			DvcCoord rock_pos = ma_rock_pos_[rock];

			if (action == -1) {
				if (rock_pos.x < rob_pos.x)
					action = Compass::WEST;
				else if (rock_pos.x > rob_pos.x)
					action = Compass::EAST;
				else if (rock_pos.y < rob_pos.y)
					action = Compass::SOUTH;
				else if (rock_pos.y > rob_pos.y)
					action = Compass::NORTH;
				else
					action = ma_rs_model_->E_SAMPLE;
			}

			discount *= Dvc_Globals::Dvc_Discount(Dvc_config,DvcCoord::ManhattanDistance(rock_pos, rob_pos));
			value += discount * expected_sampling_value[rock];
			rob_pos = rock_pos;
		}

		if (action == -1)
			action = Compass::EAST;

		value += 10 * total_weight * discount
			* Dvc_Globals::Dvc_Discount(Dvc_config,ma_grid_.xsize() - rob_pos.x);

		return Dvc_ValuedAction(action, value);
	}
};
*/

Dvc_State* BaseMultiAgentRockSample::AllocGPUParticles(int numParticles,int alloc_mode) const
{//numParticles==num_Scenarios
	clock_t start=clock();

	int threadCount=1;
	if(Globals::config.use_multi_thread_)
		threadCount=Globals::config.NUM_THREADS;
	switch(alloc_mode)
	{
	case 0:
		CreateMemoryPool(0);

		HANDLE_ERROR(cudaMallocManaged((void**)&Dvc_particles, numParticles*sizeof(Dvc_MARockSampleState)));

		//HANDLE_ERROR(cudaMallocManaged((void**)&tmp,sizeof(Dvc_MARockSampleState)));
		//HANDLE_ERROR(cudaMalloc((void**)&Dvc_particleIDs,numParticles*sizeof(int) ));
		tmp=new Dvc_MARockSampleState*[threadCount];

		for(int i=0;i<threadCount;i++)
		{
			HANDLE_ERROR(cudaHostAlloc((void**)&tmp[i],1*sizeof(Dvc_MARockSampleState),0));
		}
		Dvc_stepped_particles_all_a=new Dvc_State*[threadCount];
		Hst_stepped_particles_all_a=new Dvc_State*[threadCount];
		for(int i=0;i<threadCount;i++)
			HANDLE_ERROR(cudaMalloc((void**)&Dvc_stepped_particles_all_a[i],
					NumActions()*numParticles*sizeof(Dvc_MARockSampleState)));
		for(int i=0;i<threadCount;i++)
			HANDLE_ERROR(cudaHostAlloc((void**)&Hst_stepped_particles_all_a[i],
					NumActions()*numParticles*sizeof(Dvc_MARockSampleState),0));

		tempHostID=new int*[threadCount];
		for(int i=0;i<threadCount;i++)
		{
			cudaHostAlloc(&tempHostID[i],numParticles*sizeof(int),0);
		}

		temp_weight=new float*[threadCount];
		for(int i=0;i<threadCount;i++)
			cudaHostAlloc(&temp_weight[i],1*sizeof(float),0);

		tmp_result=new float*[threadCount];
		for(int i=0;i<threadCount;i++)
			HANDLE_ERROR(cudaMalloc(&tmp_result[i], sizeof(float)));

		return Dvc_particles;
	case 1:
		CreateMemoryPool(1);
		return Dvc_particles;
	case 2:
		//cout<<__FUNCTION__ <<endl;
		//cout<<"numParticles"<<numParticles<<endl;
		//cout<<"gpu_memory_pool_"<<gpu_memory_pool_<<endl;
		Dvc_MARockSampleState* tmp_list=gpu_memory_pool_->Allocate(numParticles);
		//Dvc_MARockSampleState* tmp;
		//HANDLE_ERROR(cudaMalloc((void**)&tmp, numParticles*sizeof(Dvc_MARockSampleState)));

		return tmp_list;
	};
	/*Dvc_particles_copy is an extern variable declared in GPUpolicy.h*/
	//HANDLE_ERROR(cudaMallocManaged((void**)&Dvc_particles_copy, numParticles*sizeof(Dvc_MARockSampleState)));
	/*AllocParticleCopy<<<grid, threads>>>(size_, size_,numParticles);
	HANDLE_ERROR(cudaDeviceSynchronize());*/

	cout<<"GPU particles alloc time:"<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl;
}
__global__ void CopyParticles(Dvc_MARockSampleState* des,Dvc_MARockSampleState* src,float* weight,
		int* IDs,int num_particles,Dvc_RandomStreams* streams, int stream_pos)
{
	int pos=blockIdx.x*blockDim.x+threadIdx.x;
	//int y=threadIdx.y;

	if(pos==0)
	{
		weight[0]=0;
		if(streams) streams->position_ = stream_pos;
	}
	__syncthreads();
	if(pos < num_particles)
	{
		bool error=false;
		int src_pos=IDs[pos];
		Dvc_MARockSampleState* src_i=src+src_pos;//src is a full length array for all particles
		Dvc_MARockSampleState* des_i=des+pos;//des is short, only for the new partition

		des_i->weight=src_i->weight;
		//des_i->allocated_=src_i->allocated_;
		des_i->state_id=src_i->state_id;
		des_i->joint_pos=src_i->joint_pos;
		des_i->scenario_id=src_i->scenario_id;
		//IDs[pos]=src_i->scenario_id;//Important!! Change the IDs to scenario IDs for later usage
		if(des_i->weight>=0.000201 || des_i->weight<=0.000199)
		{
			error=true;//error here
		}
		else
		{
			error=false;
		}

		atomicAdd(weight, des_i->weight);

		pos=error;
	}
}

void BaseMultiAgentRockSample::CopyGPUParticles(Dvc_State* des,Dvc_State* src,int src_offset,int* IDs,
		int num_particles,bool interleave,
		Dvc_RandomStreams* streams, int stream_pos,
		void* CUDAstream, int shift) const
{
	//dim3 grid(1,1); dim3 threads(num_particles,1);
	dim3 grid((num_particles+THREADDIM-1)/THREADDIM,1); dim3 threads(THREADDIM,1);


	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	if(CUDAstream)
	{
		CopyParticles<<<grid, threads,0, *(cudaStream_t*)CUDAstream>>>(static_cast<Dvc_MARockSampleState*>(des),
				static_cast<Dvc_MARockSampleState*>(src)+src_offset,tmp_result[ThreadID],
				IDs,num_particles, streams, stream_pos);
		if(!interleave)
			;//HANDLE_ERROR(cudaStreamSynchronize(*(cudaStream_t*)CUDAstream));
	}
	else
	{
		CopyParticles<<<grid, threads,0, 0>>>(static_cast<Dvc_MARockSampleState*>(des),
				static_cast<Dvc_MARockSampleState*>(src)+src_offset,tmp_result[ThreadID],
				IDs,num_particles, streams, stream_pos);
		if(!interleave)
			HANDLE_ERROR(cudaDeviceSynchronize());
	}


		//HANDLE_ERROR(cudaDeviceSynchronize());
}
void BaseMultiAgentRockSample::CopyGPUWeight1(void* cudaStream, int shift) const
{
	;
}
float BaseMultiAgentRockSample::CopyGPUWeight2(void* cudaStream, int shift) const
{
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	if(cudaStream==NULL)
	{
		float particle_weight=0;
		HANDLE_ERROR(cudaMemcpy(&particle_weight, tmp_result[ThreadID], sizeof(float),cudaMemcpyDeviceToHost));
		return particle_weight;
	}
	else
	{
		HANDLE_ERROR(cudaMemcpyAsync(temp_weight[ThreadID], tmp_result[ThreadID], sizeof(float),cudaMemcpyDeviceToHost, *(cudaStream_t*)cudaStream));
		if(*temp_weight[ThreadID]>1)
			Global_print_value(this_thread::get_id(),*temp_weight[ThreadID],"Wrong weight from CopyGPUWeight");
		HANDLE_ERROR(cudaStreamSynchronize(*(cudaStream_t*)cudaStream));
		return *temp_weight[ThreadID];
	}
}
void BaseMultiAgentRockSample::DeleteGPUParticles( int num_particles) const
{
	HANDLE_ERROR(cudaFree(static_cast<Dvc_MARockSampleState*>(Dvc_particles)));

	//HANDLE_ERROR(cudaFree(Dvc_particleIDs));

	int num_threads=1;

	if(Globals::config.use_multi_thread_)
	{
		num_threads=Globals::config.NUM_THREADS;
	}
	for(int i=0;i<num_threads;i++)
	{
		if(Dvc_stepped_particles_all_a[i]!=NULL)
			{HANDLE_ERROR(cudaFree(Dvc_stepped_particles_all_a[i]));Dvc_stepped_particles_all_a[i]=NULL;}
	}
	if(Dvc_stepped_particles_all_a)delete [] Dvc_stepped_particles_all_a;Dvc_stepped_particles_all_a=NULL;
	for(int i=0;i<num_threads;i++)
	{
		if(Hst_stepped_particles_all_a[i]!=NULL)
			{HANDLE_ERROR(cudaFreeHost(Hst_stepped_particles_all_a[i]));Hst_stepped_particles_all_a[i]=NULL;}
	}
	if(Hst_stepped_particles_all_a)delete [] Hst_stepped_particles_all_a;Hst_stepped_particles_all_a=NULL;
	//cudaFree(tmp);
	for(int i=0;i<num_threads;i++)
	{
		cudaFreeHost(tmp[i]);
	}
	delete [] tmp;

	for(int i=0;i<num_threads;i++)
	{
		cudaFreeHost(tempHostID[i]);
	}
	delete [] tempHostID;
	for(int i=0;i<num_threads;i++)
	{
		cudaFreeHost(temp_weight[i]);
	}
	delete [] temp_weight;
	for(int i=0;i<num_threads;i++)
	{
		cudaFree(tmp_result[i]);
	}
	delete [] tmp_result;
}

void BaseMultiAgentRockSample::DeleteGPUParticles(Dvc_State* particles, int num_particles) const
{
	HANDLE_ERROR(cudaFree(static_cast<Dvc_MARockSampleState*>(particles)));
}

Dvc_State* BaseMultiAgentRockSample::GetGPUParticles() const
{
	return Dvc_particles;
}
Dvc_State* BaseMultiAgentRockSample::CopyToGPU(const std::vector<State*>& particles, bool copy_cells) const
{
	//Dvc_MARockSampleState* Dvc_particles;
	//HANDLE_ERROR(cudaMalloc((void**)&Dvc_particles, particles.size()*sizeof(Dvc_MARockSampleState)));
	//dvc_particles should be managed device memory
	clock_t start=clock();

	for (int i=0;i<particles.size();i++)
	{
		const MARockSampleState* src=static_cast<const MARockSampleState*>(particles[i]);
		//Dvc_particles[i].Assign(src);
		Dvc_MARockSampleState::CopyToGPU(Dvc_particles,src->scenario_id,src);
		//Dvc_MARockSampleState::CopyToGPU(NULL,src->scenario_id,src, false);//copy to Dvc_particles_copy, do not copy cells, leave it a NULL pointer
	}
	//cout<<"GPU particles copy time:"<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl;

	return Dvc_particles;
}
void BaseMultiAgentRockSample::CopyToGPU(const std::vector<int>& particleIDs, int* Dvc_ptr, void* CUDAstream) const
{
	if(CUDAstream)
	{
		int ThreadID=Globals::MapThread(this_thread::get_id());
		memcpy(tempHostID[ThreadID],particleIDs.data(),particleIDs.size()*sizeof(int));

		//HANDLE_ERROR(cudaHostRegister((void*)particleIDs.data(),particleIDs.size()*sizeof(int),cudaHostRegisterPortable));
		HANDLE_ERROR(cudaMemcpyAsync(Dvc_ptr,tempHostID[ThreadID],particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice,*(cudaStream_t*)CUDAstream));
		//HANDLE_ERROR(cudaStreamSynchronize(*(cudaStream_t*)CUDAstream));
		//HANDLE_ERROR(cudaHostUnregister((void*)particleIDs.data()));
	}
	else
	{
		HANDLE_ERROR(cudaMemcpy(Dvc_ptr,particleIDs.data(),particleIDs.size()*sizeof(int), cudaMemcpyHostToDevice));
	}
	//return Dvc_particleIDs;
}

void BaseMultiAgentRockSample::ReadBackToCPU(std::vector<State*>& particles ,const Dvc_State* dvc_particles,
			bool deepcopy) const
{
	auto start = Time::now();

	for (int i=0;i<particles.size();i++)
	{
		const Dvc_MARockSampleState* src=static_cast<const Dvc_MARockSampleState*>(dvc_particles)+i;
		MARockSampleState* des=static_cast<MARockSampleState*>(particles[i]);
		Dvc_MARockSampleState::ReadBackToCPU(src,des);
	}
//	Dvc_MARockSampleState::ReadBackToCPU2(
//			static_cast<const Dvc_UncNavigationState*>(dvc_particles),
//			particles,true);
}

void BaseMultiAgentRockSample::CreateMemoryPool(int chunk_size) const
{
	//cout<<__FUNCTION__<<endl;
	if(gpu_memory_pool_==NULL)
		gpu_memory_pool_=new GPU_MemoryPool<Dvc_MARockSampleState>;
	//gpu_memory_pool_->SetChunkSize(chunk_size);
}
void BaseMultiAgentRockSample::DestroyMemoryPool(int mode) const
{
	//cout<<__FUNCTION__<<endl;
	if(gpu_memory_pool_){delete gpu_memory_pool_;gpu_memory_pool_=NULL;}

	/*switch(mode)
	{
		case 0:
			if(gpu_memory_pool_){delete gpu_memory_pool_;gpu_memory_pool_=NULL;}
			break;
		case 1:
			if(gpu_memory_pool_ ){ gpu_memory_pool_->ResetChuncks();};
			break;
	}*/
}

__global__ void RSPassPolicy(int policy_size, Dvc_ValuedAction* policy)
{
	ma_Dvc_policy_=policy;
	ma_Dvc_policy_size_=policy_size;
}

void BaseMultiAgentRockSample::DebugGPUParticles(Dvc_State* dvc_particles, const vector<int>& particleIDs) const {
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy(Hst_stepped_particles_all_a[ThreadID],dvc_particles,
		NumActions()*Globals::config.num_scenarios*sizeof(Dvc_MARockSampleState),
		cudaMemcpyDeviceToHost));

	cout<<"Copied back particles"<<endl;
	for(int action =0;action<NumActions();action++)
	{
		cout<<"Begin action "<<action<<" ==========================="<<endl;
		for(int i=0;i<particleIDs.size();i++)
		{
			int global_list_pos = action * Globals::config.num_scenarios + particleIDs[i];

			Dvc_MARockSampleState* particle=static_cast<Dvc_MARockSampleState*>(Hst_stepped_particles_all_a[ThreadID])+global_list_pos;
			cout<<"particle id "<<particle->scenario_id<<": ";

			cout<<particle->scenario_id<<" "<<particle->state_id<<" "<<particle->joint_pos<<endl;
		}
		cout<<"End action "<<action<<" ==========================="<<endl;
	}
}

void BaseMultiAgentRockSample::DebugGPUParticleList(Dvc_State* dvc_particles, int listlength, char* msg) const {
	int ThreadID=0;
	if(Globals::config.use_multi_thread_)
		ThreadID=Globals::MapThread(this_thread::get_id());
	HANDLE_ERROR(cudaMemcpy(Hst_stepped_particles_all_a[ThreadID],dvc_particles,
		listlength*sizeof(Dvc_MARockSampleState),
		cudaMemcpyDeviceToHost));

	cout<<"Copied back particle list"<<endl;

	cout<<"Begin list "<<" ==========================="<<endl;
	for(int i=0;i<listlength;i++)
	{
		Dvc_MARockSampleState* particle=static_cast<Dvc_MARockSampleState*>(Hst_stepped_particles_all_a[ThreadID])+i;

		cout<<msg<<" "<<particle->scenario_id<<": ";

		cout<<particle->scenario_id<<" "<<particle->state_id<<" "<<particle->joint_pos<<endl;
	}
	cout<<"End list "<<" ==========================="<<endl;

}


void BaseMultiAgentRockSample::PrintParticles(const std::vector<State*> particles, std::ostream& out) const
{
	/*for(int i=0;i<particles.size();i++)
	{
		MARockSampleState* particle=static_cast<MARockSampleState*>(particles[i]);
		cout<<"particle id "<<particle->scenario_id<<": ";
		cout<<particle->scenario_id<<" "<<particle->state_id<<" "<<particle->joint_pos<<endl;
	}*/
}
} // namespace despot
