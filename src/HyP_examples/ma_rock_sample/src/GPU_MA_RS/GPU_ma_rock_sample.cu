#include "GPU_ma_rock_sample.h"
#include "base/base_ma_rock_sample.h"
#include "ma_rock_sample/ma_rock_sample.h"
#include <bitset>

#include <despot/solver/GPUdespot.h>
using namespace std;

namespace despot {

/* =============================================================================
 * Dvc_MultiAgentRockSample class
 * =============================================================================*/
extern __shared__ int localParticles[];

__global__ void step_global( Dvc_State* vnode_particles,float* rand,
		float * reward, OBS_TYPE* obs, float* ub, Dvc_ValuedAction* lb,
		bool* term, int num_particles, int parent_action, Dvc_State* state)
{
	int action=blockIdx.x;
	int PID = (blockIdx.y * blockDim.x + threadIdx.x) ;
	if(PID<num_particles){
		int global_list_pos = action * num_particles + PID;
		float rand_num=rand[global_list_pos];

		if (threadIdx.y == 0) {
			DvcModelCopyToShared_(
				(Dvc_State*) ((int*) localParticles + 8 * threadIdx.x),
				vnode_particles, PID % num_particles, false);
		}
		Dvc_State* current_particle = (Dvc_State*) ((int*) localParticles + 8 * threadIdx.x);
		__syncthreads();


		term[global_list_pos]=DvcModelStep_(*current_particle, rand_num, parent_action, reward[global_list_pos], obs[global_list_pos]);

		if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
			/*Record stepped particles from parent as particles in this node*/
			if (threadIdx.y == 0 && action==0) {
				Dvc_State* temp = DvcModelGet_(vnode_particles, PID % num_particles);
				DvcModelCopyNoAlloc_(temp, current_particle,0, false);
			}
		}

		term[global_list_pos]=DvcModelStep_(*current_particle, rand_num, action, reward[global_list_pos], obs[global_list_pos]);

		Dvc_History history;
		Dvc_RandomStreams streams;
		ub[global_list_pos]=DvcUpperBoundValue_(current_particle, 0, history);
		lb[global_list_pos]=DvcLowerBoundValue_(current_particle,streams,history, 0) ;

		Dvc_State* temp = DvcModelGet_(state, global_list_pos);
		DvcModelCopyNoAlloc_(temp, current_particle, 0, false);
	}
}


__global__ void step_global_1( Dvc_State* vnode_particles,float* rand,
		float * reward, OBS_TYPE* obs, float* ub, Dvc_ValuedAction* lb,
		bool* term, int num_particles, int parent_action, Dvc_State* state)
{
	int action=blockIdx.x;
	int PID = (blockIdx.y * blockDim.x + threadIdx.x) ;
	if(PID<num_particles){
		int global_list_pos = action * num_particles + PID;
		float rand_num=rand[global_list_pos];

		if (threadIdx.y == 0) {
			DvcModelCopyToShared_(
				(Dvc_State*) ((int*) localParticles + 8 * threadIdx.x),
				vnode_particles, PID % num_particles, false);
		}
		Dvc_State* current_particle = (Dvc_State*) ((int*) localParticles + 8 * threadIdx.x);
		__syncthreads();

		term[global_list_pos]=DvcModelStep_(*current_particle, rand_num, parent_action, reward[global_list_pos], obs[global_list_pos]);


		Dvc_State* temp = DvcModelGet_(state, global_list_pos);
		DvcModelCopyNoAlloc_(temp, current_particle, 0, false);
	}
}
__global__ void step_global_2( Dvc_State* vnode_particles,float* rand,
		float * reward, OBS_TYPE* obs, float* ub, Dvc_ValuedAction* lb,
		bool* term, int num_particles, int parent_action, Dvc_State* state)
{
	int action=blockIdx.x;
	int PID = (blockIdx.y * blockDim.x + threadIdx.x) ;
	if(PID<num_particles){
		int global_list_pos = action * num_particles + PID;
		float rand_num=rand[global_list_pos];

		if (threadIdx.y == 0) {
			DvcModelCopyToShared_(
				(Dvc_State*) ((int*) localParticles + 8 * threadIdx.x),
				state, global_list_pos, false);
		}
		Dvc_State* current_particle = (Dvc_State*) ((int*) localParticles + 8 * threadIdx.x);


		if (blockIdx.y * blockDim.x + threadIdx.x < num_particles) {
			/*Record stepped particles from parent as particles in this node*/
			if (threadIdx.y == 0 && action==0) {
				Dvc_State* temp = DvcModelGet_(vnode_particles, PID % num_particles);
				DvcModelCopyNoAlloc_(temp, current_particle,0, false);
			}
		}
		__syncthreads();


		term[global_list_pos]=DvcModelStep_(*current_particle, rand_num, action, reward[global_list_pos], obs[global_list_pos]);

		Dvc_History history;
		Dvc_RandomStreams streams;
		ub[global_list_pos]=DvcUpperBoundValue_(current_particle, 0, history);
		lb[global_list_pos]=DvcLowerBoundValue_(current_particle,streams,history, 0) ;

		Dvc_State* temp = DvcModelGet_(state, global_list_pos);
		DvcModelCopyNoAlloc_(temp, current_particle, 0, false);
	}
}
void MultiAgentRockSample::Debug( ScenarioLowerBound* lower_bound,ScenarioUpperBound* upper_bound) const{
	//memory_pool_.CheckFreeList();

	if(Globals::config.useGPU){

		int num_tests=0;

		if(num_tests>0){
			cout<<"===================Unit test=================="<<endl;

			Dvc_MARockSampleState* dvc_node_particles;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_node_particles,Globals::config.num_scenarios*sizeof(Dvc_MARockSampleState)));
			Dvc_MARockSampleState* dvc_state;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_state,NumActions()*Globals::config.num_scenarios*sizeof(Dvc_MARockSampleState)));
			OBS_TYPE* dvc_obs;
			vector<OBS_TYPE> hst_obs;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_obs,NumActions()*Globals::config.num_scenarios*sizeof(OBS_TYPE)));
			hst_obs.resize(NumActions()*Globals::config.num_scenarios);

			float* dvc_reward;
			vector<double> hst_reward;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_reward,NumActions()*Globals::config.num_scenarios*sizeof(float)));
			hst_reward.resize(NumActions()*Globals::config.num_scenarios);

			Dvc_ValuedAction* dvc_lb;
			vector<ValuedAction> hst_lb;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_lb,NumActions()*Globals::config.num_scenarios*sizeof(Dvc_ValuedAction)));
			hst_lb.resize(NumActions()*Globals::config.num_scenarios);

			float* dvc_ub;
			vector<float> hst_ub;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_ub,NumActions()*Globals::config.num_scenarios*sizeof(float)));
			hst_ub.resize(NumActions()*Globals::config.num_scenarios);

			bool* dvc_term;
			vector<bool> hst_term;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_term,NumActions()*Globals::config.num_scenarios*sizeof(bool)));
			hst_term.resize(NumActions()*Globals::config.num_scenarios);

			float* dvc_rand;
			vector<double> hst_rand;
			HANDLE_ERROR(cudaMallocManaged((void**)&dvc_rand,NumActions()*Globals::config.num_scenarios*sizeof(float)));
			hst_rand.resize(NumActions()*Globals::config.num_scenarios);

			vector<MARockSampleState*> hst_states;
			hst_states.resize(NumActions()*Globals::config.num_scenarios);

			vector<MARockSampleState> original_states;
			original_states.resize(NumActions()*Globals::config.num_scenarios);
			vector<MARockSampleState> gpu_original_states;
			gpu_original_states.resize(NumActions()*Globals::config.num_scenarios);

			int parent_action=0;
			for(int action=0;action<NumActions();action++)
			{
				for(int PID=0;PID<Globals::config.num_scenarios;PID++)
				{
					int global_sid=action*Globals::config.num_scenarios+PID;

					hst_states[global_sid]=static_cast<MARockSampleState*>(CreateStartState());
					hst_states[global_sid]->weight=1;

					hst_states[global_sid]->state_id= 12535;
					hst_states[global_sid]->joint_pos= 123995;

					int pos_x=2;
					int pos_y=7;
					SetRobPosIndex(hst_states[global_sid]->joint_pos, 0, grid_.Index(pos_x, pos_y));
					pos_x=2;
					pos_y=5;
					SetRobPosIndex(hst_states[global_sid]->joint_pos, 1, grid_.Index(pos_x, pos_y));


					/*int pos_x=Random::RANDOM.NextInt(grid_.xsize()-1);
					int pos_y=Random::RANDOM.NextInt(grid_.ysize()-1);
					SetRobPosIndex(hst_states[global_sid]->joint_pos, 0, grid_.Index(pos_x, pos_y));
					if(Random::RANDOM.NextDouble()<0.05)
						SetRobPosIndex(hst_states[global_sid]->joint_pos, 0, ROB_TERMINAL_ID);

					pos_x=Random::RANDOM.NextInt(grid_.xsize());
					pos_y=Random::RANDOM.NextInt(grid_.ysize());
					SetRobPosIndex(hst_states[global_sid]->joint_pos, 1, grid_.Index(pos_x, pos_y));
					if(Random::RANDOM.NextDouble()<0.05)
						SetRobPosIndex(hst_states[global_sid]->joint_pos, 1, ROB_TERMINAL_ID);*/

					Dvc_MARockSampleState::CopyToGPU(dvc_node_particles+PID, 0, hst_states[global_sid], true);
					gpu_original_states[global_sid].joint_pos=dvc_node_particles[PID].joint_pos;

					pos_x=2;
					pos_y=8;
					SetRobPosIndex(hst_states[global_sid]->joint_pos, 0, grid_.Index(pos_x, pos_y));
					pos_x=2;
					pos_y=6;
					SetRobPosIndex(hst_states[global_sid]->joint_pos, 1, grid_.Index(pos_x, pos_y));
					//hst_states[global_sid]->joint_pos= 125020;
					original_states[global_sid]=*hst_states[global_sid];

					hst_rand[global_sid]=Random::RANDOM.NextDouble();
					dvc_rand[global_sid]=hst_rand[global_sid];

					/*if(dvc_node_particles[PID].joint_pos!=hst_states[global_sid]->joint_pos){
						cout<<"a-"<<action<<" p-"<<PID<<"Wrong init!! diff robpos (gpu cpu): "<< bitset<32>(dvc_node_particles[PID].joint_pos)<<
								" "<<bitset<32>(hst_states[global_sid]->joint_pos)<<endl;}

					if(dvc_node_particles[PID].state_id!=hst_states[global_sid]->state_id){
						cout<<"a-"<<action<<" p-"<<PID<<"Wrong init!! diff rock (gpu cpu): "<< bitset<32>(dvc_node_particles[PID].state_id)<<
								" "<<bitset<32>(hst_states[global_sid]->state_id)<<endl;}

					if(abs(dvc_rand[global_sid]-hst_rand[global_sid])>1e-4){
						cout<<"a-"<<action<<" p-"<<PID<<"Wrong init!! diff rand (gpu cpu): "<< dvc_rand[global_sid]<<" "<<hst_rand[global_sid]<<endl;}
*/
				}
			}

			for(int i=0;i<num_tests/*0*/;i++){
				cout<<"===================Step "<<i<<"=================="<<endl;

				RandomStreams streams;
				History history;
				for(int action=0;action<NumActions();action++)
				{
					for(int PID=0;PID<Globals::config.num_scenarios;PID++)
					{
						std::vector<State*> particles;
						int global_sid=action*Globals::config.num_scenarios+PID;
						particles.push_back(hst_states[global_sid]);


						hst_term[global_sid] =Step(*particles[0], hst_rand[global_sid],action, hst_reward[global_sid],hst_obs[global_sid]);
						//cout<<".............."<<endl;
						hst_ub[global_sid]=upper_bound->Value(particles, streams,history);
						hst_lb[global_sid]=lower_bound->Value(particles, streams,history);

					}
				}

				dim3 grid(NumActions(), (Globals::config.num_scenarios+32-1)/32);
				dim3 threads(32,1);
				//step_global<<<grid,threads, 32 * 8 * sizeof(int)>>>(dvc_node_particles, dvc_rand,dvc_reward, dvc_obs, dvc_ub, dvc_lb,
				//		dvc_term, Globals::config.num_scenarios, 0, dvc_state);
				step_global_1<<<grid,threads, 32 * 8 * sizeof(int)>>>(dvc_node_particles, dvc_rand,dvc_reward, dvc_obs, dvc_ub, dvc_lb,
						dvc_term, Globals::config.num_scenarios, parent_action, dvc_state);
				step_global_2<<<grid,threads, 32 * 8 * sizeof(int)>>>(dvc_node_particles, dvc_rand,dvc_reward, dvc_obs, dvc_ub, dvc_lb,
						dvc_term, Globals::config.num_scenarios, parent_action, dvc_state);
				HANDLE_ERROR(cudaDeviceSynchronize());
				//cout<<".............."<<endl;

				for(int action=0;action<NumActions();action++)
				{
					for(int PID=0;PID<Globals::config.num_scenarios;PID++)
					{
						int global_sid=action*Globals::config.num_scenarios+PID;

						bool print_origin=false;
						if(dvc_obs[global_sid]!=hst_obs[global_sid]){
							cout<<"diff obs (gpu cpu): "<< bitset<8>(dvc_obs[global_sid])<<" "<<bitset<8>(hst_obs[global_sid])<<endl;print_origin=true;

							cout<<"	rob 0: "<< bitset<MAX_OBS_BIT>(GetRobObs(dvc_obs[global_sid],0))<<" "<<bitset<MAX_OBS_BIT>(GetRobObs(hst_obs[global_sid], 0))<<endl;
							cout<<"	rob 1: "<< bitset<MAX_OBS_BIT>(GetRobObs(dvc_obs[global_sid],1))<<" "<<bitset<MAX_OBS_BIT>(GetRobObs(hst_obs[global_sid], 1))<<endl;

						}
						if(dvc_reward[global_sid]!=hst_reward[global_sid]){
							cout<<"diff reward (gpu cpu): "<< dvc_reward[global_sid]<<" "<<hst_reward[global_sid]<<endl;
							print_origin=true;}

						if(dvc_state[global_sid].joint_pos!=hst_states[global_sid]->joint_pos){
							cout<<"diff robpos (gpu cpu): "<< bitset<2*MAX_COORD_BIT>(dvc_state[global_sid].joint_pos)
									<<" "<<bitset<2*MAX_COORD_BIT>(hst_states[global_sid]->joint_pos)<<endl;

							Coord rob_0=grid_.GetCoord(GetRobPosIndex(dvc_node_particles[PID].joint_pos, 0));
							Coord rob_1=grid_.GetCoord(GetRobPosIndex(dvc_node_particles[PID].joint_pos, 1));
							cout<<"	gpu middle ouput rob 0 coord: "<<rob_0.x<<" "<<rob_0.y<<endl;
							cout<<"	gpu middle ouput rob 1 coord: "<<rob_1.x<<" "<<rob_1.y<<endl;

							rob_0=grid_.GetCoord(GetRobPosIndex(dvc_state[global_sid].joint_pos, 0));
							rob_1=grid_.GetCoord(GetRobPosIndex(dvc_state[global_sid].joint_pos, 1));
							cout<<"	gpu output rob 0 coord: "<<rob_0.x<<" "<<rob_0.y<<endl;
							cout<<"	gpu output rob 1 coord: "<<rob_1.x<<" "<<rob_1.y<<endl;

							rob_0=grid_.GetCoord(GetRobPosIndex(hst_states[global_sid]->joint_pos, 0));
							rob_1=grid_.GetCoord(GetRobPosIndex(hst_states[global_sid]->joint_pos, 1));
							cout<<"	cpu output rob 0 coord: "<<rob_0.x<<" "<<rob_0.y<<endl;
							cout<<"	cpu output rob 1 coord: "<<rob_1.x<<" "<<rob_1.y<<endl;
							print_origin=true;}

						if(dvc_state[global_sid].state_id!=hst_states[global_sid]->state_id){
							cout<<"diff rock (gpu cpu): "<< bitset<15>(dvc_state[global_sid].state_id)<<" "
									<<bitset<15>(hst_states[global_sid]->state_id)<<endl;
							print_origin=true;}

						if(abs(dvc_ub[global_sid]-hst_ub[global_sid])>1e-4){
							cout<<"diff ub (gpu cpu): "<< dvc_ub[global_sid]<<" "<<hst_ub[global_sid]<<endl;
							print_origin=true;}

						if(abs(dvc_lb[global_sid].value-hst_lb[global_sid].value)>1e-5){
							cout<<"diff lb (gpu cpu): "<< dvc_lb[global_sid].value<<" "<<hst_lb[global_sid].value<<endl;print_origin=true;}

						if(dvc_term[global_sid]!=hst_term[global_sid]){
							cout<<"diff term (gpu cpu): "<< dvc_term[global_sid]<<" "<<hst_term[global_sid]<<endl;print_origin=true;}

						if(print_origin)
						{
							cout<<"original state (a-"<<action<<" p-"<<PID<<"):"<<endl;
							cout<<"	state_id="<<bitset<2*MAX_COORD_BIT>(original_states[global_sid].state_id)<<endl;
							cout<<"	joint_pos="<<bitset<15>(original_states[global_sid].joint_pos)<<endl;
							Coord rob_0=grid_.GetCoord(GetRobPosIndex(original_states[global_sid].joint_pos, 0));
							Coord rob_1=grid_.GetCoord(GetRobPosIndex(original_states[global_sid].joint_pos, 1));
							cout<<"	cpu input rob 0 coord: "<<rob_0.x<<" "<<rob_0.y<<endl;
							cout<<"	cpu input rob 1 coord: "<<rob_1.x<<" "<<rob_1.y<<endl;
							rob_0=grid_.GetCoord(GetRobPosIndex(gpu_original_states[global_sid].joint_pos, 0));
							rob_1=grid_.GetCoord(GetRobPosIndex(gpu_original_states[global_sid].joint_pos, 1));
							cout<<"	gpu input rob 0 coord: "<<rob_0.x<<" "<<rob_0.y<<endl;
							cout<<"	gpu input rob 1 coord: "<<rob_1.x<<" "<<rob_1.y<<endl;

							cout<<"action="<<action<<endl;
							cout<<"rand="<<hst_rand[global_sid]<<endl;
							cout<<"robpos (gpu cpu): "<< bitset<2*MAX_COORD_BIT>(dvc_state[global_sid].joint_pos)
																<<" "<<bitset<2*MAX_COORD_BIT>(hst_states[global_sid]->joint_pos)<<endl;

							cout<<"----------------------"<<endl;
						}
					}
				}
			}

			HANDLE_ERROR(cudaFree(dvc_lb));
			HANDLE_ERROR(cudaFree(dvc_ub));
			HANDLE_ERROR(cudaFree(dvc_obs));
			HANDLE_ERROR(cudaFree(dvc_reward));
			HANDLE_ERROR(cudaFree(dvc_term));
			HANDLE_ERROR(cudaFree(dvc_rand));
			HANDLE_ERROR(cudaFree(dvc_state));
			HANDLE_ERROR(cudaFree(dvc_node_particles));

			cout<<"===================Unit test finished=================="<<endl;
			exit(0);
		}

	}

}
DEVICE bool Dvc_MultiAgentRockSample::Dvc_Step(Dvc_State& state, float rand_num, int action, float& reward,
			OBS_TYPE& obs)
{
	reward=0;
	obs=0;
	bool terminal=true;
	Dvc_MARockSampleState& rockstate = static_cast<Dvc_MARockSampleState&>(state);


	__syncthreads();

	for(int rid=0;rid<num_agents_;rid++)
	{
		SetRobObs(obs, E_NONE, rid);

		if(GetRobPosIndex(&rockstate, rid)!=ROB_TERMINAL_ID){

			int rob_act=GetRobAction(action, rid);
			//rob_act=Dvc_Compass::EAST;//debugging
			if (rob_act < E_SAMPLE) { // Move
				switch (rob_act) {
				case Dvc_Compass::EAST:
					if (GetX(&rockstate, rid) + 1 < ma_map_size_) {
						IncX(&rockstate, rid);
					} else {
						reward+= +10;
						SetRobPosIndex(rockstate.joint_pos, rid, ROB_TERMINAL_ID);
					}
					break;

				case Dvc_Compass::NORTH:
					if (GetY(&rockstate, rid) + 1 < ma_map_size_)
						IncY(&rockstate, rid);
					else{
						reward += -100;
					}
					break;

				case Dvc_Compass::SOUTH:
					if (GetY(&rockstate, rid) - 1 >= 0)
						DecY(&rockstate, rid);
					else
						reward += -100;
					break;

				case Dvc_Compass::WEST:
					if (GetX(&rockstate, rid) - 1 >= 0)
						DecX(&rockstate, rid);
					else
						reward += -100;
					break;
				}
			}
			if (rob_act == E_SAMPLE) { // Sample
				int rock = ma_grid_[GetRobPosIndex(&rockstate, rid)];
				if (rock >= 0) {
					if (GetRock(&rockstate, rock))
						reward += +10;
					else
						reward += -10;
					SampleRock(&rockstate, rock);
				} else {
					reward += -100;
				}
			}

			if (rob_act > E_SAMPLE) { // Sense
				int rock = rob_act - E_SAMPLE - 1;
				float distance = DvcCoord::EuclideanDistance(GetRobPos(&rockstate, rid),
					ma_rock_pos_[rock]);
				float efficiency = (1 + powf(2, -distance / ma_half_efficiency_distance_))
					* 0.5;

				if (rand_num < efficiency)
					SetRobObs(obs, GetRock(&rockstate, rock) & E_GOOD, rid);
				else
					SetRobObs(obs, !(GetRock(&rockstate, rock) & E_GOOD), rid);
			}


			if (GetRobPosIndex(&rockstate, rid)!=ROB_TERMINAL_ID) {
				terminal=false;
			}
		}
	}


	return terminal;
}

DEVICE int Dvc_MultiAgentRockSample::NumActions()
{
	return pow((float)(ma_num_rocks_ + 5), num_agents_);
}


DEVICE int Dvc_MultiAgentRockSample::Dvc_NumObservations()
{
	return /*3*/num_agents_*MAX_OBS_BIT;
}
DEVICE Dvc_State* Dvc_MultiAgentRockSample::Dvc_Get(Dvc_State* particles, int pos)
{
	Dvc_MARockSampleState* particle_i= static_cast<Dvc_MARockSampleState*>(particles)+pos;

	return particle_i;
}
DEVICE void Dvc_MultiAgentRockSample::Dvc_Copy_NoAlloc(Dvc_State* des, const Dvc_State* src, int pos, bool offset_des)
{
	/*Pass member values, assign member pointers to existing state pointer*/
	const Dvc_MARockSampleState* src_i= static_cast<const Dvc_MARockSampleState*>(src)+pos;
	if(!offset_des) pos=0;
	Dvc_MARockSampleState* des_i= static_cast<const Dvc_MARockSampleState*>(des)+pos;

	des_i->weight = src_i->weight;
	des_i->scenario_id = src_i->scenario_id;
	des_i->state_id = src_i->state_id;
	des_i->joint_pos = src_i->joint_pos;

	//des_i->allocated_=true;
}

} // namespace despot
