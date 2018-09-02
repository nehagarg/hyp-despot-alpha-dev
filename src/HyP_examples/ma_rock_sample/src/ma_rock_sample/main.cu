#include <despot/config.h>
#include <despot/core/globals.h>
#include <despot/interface/pomdp.h>
#include <despot/GPUconfig.h>
#include <despot/GPUcore/GPUglobals.h>
#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUinterface/GPUdefault_policy.h>
#include <despot/GPUcore/GPUpolicy_graph.h>
#include <despot/GPUinterface/GPUpomdp.h>
#include <despot/GPUutil/GPUcoord.h>
#include <despot/simple_tui.h>
#include <despot/solver/GPUdespot.h>
#include <despot/util/optionparser.h>
#include <stdlib.h>
#include <iostream>
#include <ostream>

#include "../../../../../src/GPUcore/GPU_MA_RS/GPU_ma_rock_sample.h"
#include "ma_rock_sample.h"


using namespace std;
using namespace despot;
static Dvc_MultiAgentRockSample* Dvc=NULL;
static Dvc_DefaultPolicy* inde_lowerbound=NULL;
static Dvc_PolicyGraph* graph_lowerbound=NULL;
static Dvc_MARockSampleEastScenarioLowerBound* east_lowerbound=NULL;
static Dvc_TrivialParticleLowerBound* b_lowerbound=NULL;
static Dvc_MARockSampleTrivialParticleUpperBound* upperbound=NULL;
static Dvc_MARockSampleApproxParticleUpperBound* approx_upperbound=NULL;
static int* tempGrid;
static DvcCoord* temp_rockpos;

__global__ void PassModelFuncs(Dvc_MultiAgentRockSample* model,int map_size,
		int num_rocks, double half_efficiency_distance, int* grid, DvcCoord* rockpos, int num_agents)
{
	DvcModelStep_=&(model->Dvc_Step);
	DvcModelCopyNoAlloc_=&(model->Dvc_Copy_NoAlloc);
	DvcModelCopyToShared_=&(model->Dvc_Copy_NoAlloc);

	DvcModelGet_=&(model->Dvc_Get);
	DvcModelGetBestAction_=&(model->Dvc_GetBestAction);
	DvcModelGetMaxReward_=&(model->Dvc_GetMaxReward);

	ma_rs_model_=model;
	num_agents_=num_agents;
	ma_map_size_=map_size;
	ma_num_rocks_=num_rocks;
	ma_half_efficiency_distance_=half_efficiency_distance;
	ma_grid_=grid;//A flattened pointer of a 2D map
	ma_rock_pos_=rockpos;
}


__global__ void PassActionValueFuncs(Dvc_MultiAgentRockSample* model, Dvc_RandomPolicy* lowerbound,Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleMDPParticleUpperBound* upperbound)
{
	lowerbound->Init(model->NumActions());
	DvcDefaultPolicyAction_=&(lowerbound->Action);

	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}

__global__ void PassActionValueFuncs(Dvc_MultiAgentRockSample* model, Dvc_RandomPolicy* lowerbound,Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleTrivialParticleUpperBound* upperbound)
{
	lowerbound->Init(model->NumActions());
	DvcDefaultPolicyAction_=&(lowerbound->Action);

	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}

__global__ void PassValueFuncs(Dvc_PolicyGraph* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleMDPParticleUpperBound* upperbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}
__global__ void PassValueFuncs(Dvc_PolicyGraph* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleTrivialParticleUpperBound* upperbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}

__global__ void PassValueFuncs(Dvc_MARockSampleEastScenarioLowerBound* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleMDPParticleUpperBound* upperbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}

__global__ void PassValueFuncs(Dvc_MARockSampleEastScenarioLowerBound* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleApproxParticleUpperBound* upperbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}
__global__ void PassValueFuncs(Dvc_MARockSampleEastScenarioLowerBound* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_MARockSampleTrivialParticleUpperBound* upperbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

}
__global__ void RSPassPolicyGraph(int graph_size, int num_edges_per_node, int* action_nodes, int* obs_edges)
{
	graph_size_=graph_size;
	num_edges_per_node_=num_edges_per_node;
	action_nodes_=action_nodes;
	obs_edges_=obs_edges;
}

__global__ void ClearPolicyGraph()
{
	free(action_nodes_);
	free(obs_edges_);
}


class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {

    DSPOMDP* model = NULL;
		if (options[E_PARAMS_FILE]) {
			cerr << "Map file is not supported" << endl;
			exit(0);
		} else {
			int size = 20, number = 20;
			if (options[E_SIZE])
				size = atoi(options[E_SIZE].arg);
			else {
				if(FIX_SCENARIO==1 || FIX_SCENARIO==2)
					size=15;
				else
					size=15;
			}
			if (options[E_NUMBER]) {
				number = atoi(options[E_NUMBER].arg);
			} else {
				if(FIX_SCENARIO==1 || FIX_SCENARIO==2)
					number=15;
				else
					number =15;
			}

			model = new MultiAgentRockSample(size, number);


		}
    return model;
  }
  void InitializedGPUModel(std::string rollout_type)
  {

  }

  void InitializedGPUModel(std::string rollout_type, DSPOMDP* Hst_model)
  {
	  MultiAgentRockSample* Hst =static_cast<MultiAgentRockSample*>(Hst_model);
	  HANDLE_ERROR(cudaMalloc((void**)&Dvc, sizeof(Dvc_MultiAgentRockSample)));
	  if(rollout_type=="INDEPENDENT")
		  HANDLE_ERROR(cudaMalloc((void**)&inde_lowerbound, sizeof(Dvc_RandomPolicy)));
	  if(rollout_type=="GRAPH")
		  HANDLE_ERROR(cudaMalloc((void**)&graph_lowerbound, sizeof(Dvc_PolicyGraph)));
	  if(rollout_type=="BLIND")
	  	  HANDLE_ERROR(cudaMalloc((void**)&east_lowerbound, sizeof(Dvc_MARockSampleEastScenarioLowerBound)));

	  HANDLE_ERROR(cudaMalloc((void**)&b_lowerbound, sizeof(Dvc_TrivialParticleLowerBound)));
	  HANDLE_ERROR(cudaMalloc((void**)&upperbound, sizeof(Dvc_MARockSampleTrivialParticleUpperBound)));
	  HANDLE_ERROR(cudaMalloc((void**)&approx_upperbound, sizeof(Dvc_MARockSampleApproxParticleUpperBound)));


	  HANDLE_ERROR(cudaMalloc((void**)&tempGrid,  Hst->size_* Hst->size_*sizeof(int)));
	  HANDLE_ERROR(cudaMalloc((void**)&temp_rockpos, Hst->num_rocks_*sizeof(DvcCoord)));

	  HANDLE_ERROR(cudaMemcpy(tempGrid, Hst->grid_.Data(), Hst->size_* Hst->size_*sizeof(int), cudaMemcpyHostToDevice));
	  HANDLE_ERROR(cudaMemcpy(temp_rockpos, Hst->rock_pos_.data(), Hst->num_rocks_*sizeof(DvcCoord), cudaMemcpyHostToDevice));

	  PassModelFuncs<<<1,1,1>>>(Dvc, Hst->size_, Hst->num_rocks_,Hst->half_efficiency_distance_, tempGrid, temp_rockpos, Hst->num_agents_);
	  if(rollout_type=="INDEPENDENT")
		  PassActionValueFuncs<<<1,1,1>>>(Dvc,static_cast<Dvc_RandomPolicy*>(inde_lowerbound),b_lowerbound,upperbound);
	  if(rollout_type=="GRAPH")
		  PassValueFuncs<<<1,1,1>>>(static_cast<Dvc_PolicyGraph*>(graph_lowerbound),b_lowerbound,upperbound);
	  if(rollout_type=="BLIND")
	  	  PassValueFuncs<<<1,1,1>>>(east_lowerbound,b_lowerbound,approx_upperbound);

	  HANDLE_ERROR(cudaDeviceSynchronize());
  }


  void InitializeGPUPolicyGraph(PolicyGraph* hostGraph)
  {
	  int* tmp_node_list; int* tmp_edge_list;
	  HANDLE_ERROR(cudaMalloc((void**)&tmp_node_list, hostGraph->graph_size_*sizeof(int)));
	  HANDLE_ERROR(cudaMalloc((void**)&tmp_edge_list, hostGraph->graph_size_*hostGraph->num_edges_per_node_*sizeof(int)));

	  HANDLE_ERROR(cudaMemcpy(tmp_node_list, hostGraph->action_nodes_.data(), hostGraph->graph_size_*sizeof(int), cudaMemcpyHostToDevice));

	  for (int i = 0; i < hostGraph->num_edges_per_node_; i++)
	  {
		  HANDLE_ERROR(cudaMemcpy(tmp_edge_list+i*hostGraph->graph_size_, hostGraph->obs_edges_[(OBS_TYPE)i].data(), hostGraph->graph_size_*sizeof(int), cudaMemcpyHostToDevice));
	  }

	  RSPassPolicyGraph<<<1,1,1>>>(hostGraph->graph_size_,hostGraph->num_edges_per_node_,
			  tmp_node_list,tmp_edge_list );
	  HANDLE_ERROR(cudaDeviceSynchronize());
  }

  void InitializeGPUGlobals()
  {
	  HANDLE_ERROR(cudaMallocManaged((void**)&Dvc_Globals::config, sizeof(Dvc_Config)));
	  Dvc_Globals::config->search_depth=Globals::config.search_depth;
	  Dvc_Globals::config->discount=Globals::config.discount;
	  Dvc_Globals::config->root_seed=Globals::config.root_seed;
	  Dvc_Globals::config->time_per_move=Globals::config.time_per_move;  // CPU time available to construct the search tree
	  Dvc_Globals::config->num_scenarios=Globals::config.num_scenarios;
	  Dvc_Globals::config->pruning_constant=Globals::config.pruning_constant;
	  Dvc_Globals::config->xi=Globals::config.xi; // xi * gap(root) is the target uncertainty at the root.
	  Dvc_Globals::config->sim_len=Globals::config.sim_len; // Number of steps to run the simulation for.
	  Dvc_Globals::config->max_policy_sim_len=Globals::config.max_policy_sim_len; // Maximum number of steps for simulating the default policy
	  Dvc_Globals::config->noise=Globals::config.noise;
	  Dvc_Globals::config->silence=Globals::config.silence;

  }


  void DeleteGPUModel()
  {
	  HANDLE_ERROR(cudaFree(Dvc));
	  if(inde_lowerbound)HANDLE_ERROR(cudaFree(inde_lowerbound));
	  if(graph_lowerbound)HANDLE_ERROR(cudaFree(graph_lowerbound));
	  if(east_lowerbound)HANDLE_ERROR(cudaFree(east_lowerbound));

	  HANDLE_ERROR(cudaFree(b_lowerbound));
	  HANDLE_ERROR(cudaFree(upperbound));
	  HANDLE_ERROR(cudaFree(approx_upperbound));

	  HANDLE_ERROR(cudaFree(tempGrid));
	  HANDLE_ERROR(cudaFree(temp_rockpos));
  }

  void DeleteGPUGlobals()
  {
      HANDLE_ERROR(cudaFree(Dvc_Globals::config));
  }

  void DeleteGPUPolicyGraph()
  {
  }

  void InitializeDefaultParameters() {
	  	Globals::config.GPUid=1;//default GPU
		Globals::config.useGPU=true;

		Globals::config.Globals::config.use_multi_thread_=false;
		Globals::config.NUM_THREADS=10;
		Globals::config.sim_len=40;
		Globals::config.max_policy_sim_len=10;
		Globals::config.time_per_move=2;
		Globals::config.num_scenarios=1;
		Globals::config.discount=0.983;

		Obs_type=OBS_LONG64;

		Globals::config.exploration_mode=UCT;
		Globals::config.exploration_constant=/*0.8*/0.1;
		Globals::config.exploration_constant_o=0.1;

		//Globals::config.search_depth=10;//debugging


	}

};

int main(int argc, char* argv[]) {

  return TUI().run(argc, argv);
}


