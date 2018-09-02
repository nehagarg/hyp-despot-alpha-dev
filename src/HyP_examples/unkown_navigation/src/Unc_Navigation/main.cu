/*#include <despot/simple_tui.h>
#include <despot/solver/GPUdespot.h>
#include <despot/util/optionparser.h>
#include <stdlib.h>
#include <Unc_Navigation/UncNavigation.h>
#include <iostream>
#include <ostream>

#include "../../../../../src/GPUcore/GPU_Unk_nav/GPU_UncNavigation.h"
#include <despot/GPUcore/GPUpolicy_graph.h>

using namespace std;
using namespace despot;

static Dvc_UncNavigation* Dvc=NULL;
static Dvc_DefaultPolicy* inde_lowerbound=NULL;
static Dvc_PolicyGraph* graph_lowerbound=NULL;
static Dvc_TrivialParticleLowerBound* b_lowerbound=NULL;
static Dvc_UncNavigationParticleUpperBound1* upperbound=NULL;

__global__ void PassModelFuncs(Dvc_UncNavigation* model)
{
	DvcModelStep_=&(model->Dvc_Step);
	Dvc_DSPOMDP::DvcModelAlloc_=&(model->Dvc_Alloc);
	Dvc_DSPOMDP::DvcModelCopy_=&(model->Dvc_Copy);
	DvcModelCopyNoAlloc_=&(model->Dvc_Copy_NoAlloc);
	DvcModelCopyToShared_=&(model->Dvc_Copy_NoAlloc);
	//DvcModelCopyNoAlloc_(NULL, NULL, 0);
	Dvc_DSPOMDP::DvcModelFree_=&(model->Dvc_Free);
	DvcModelGet_=&(model->Dvc_Get);
	DvcModelGetBestAction_=&(model->Dvc_GetBestAction);
	DvcModelGetMaxReward_=&(model->Dvc_GetMaxReward);
}

__global__ void PassActionValueFuncs(Dvc_UncNavigation* model, Dvc_RandomPolicy* lowerbound,Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_UncNavigationParticleUpperBound1* upperbound)
{
	lowerbound->Init(model->NumActions());
	DvcDefaultPolicyAction_=&(lowerbound->Action);

	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value

	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value
	//DvcPolicyValue_=&();
	//Dvc_DSPOMDP::DvcModelFreeList_=&(dvc->Dvc_FreeList);
}

__global__ void PassValueFuncs(Dvc_PolicyGraph* lowerbound,
		Dvc_TrivialParticleLowerBound* b_lowerbound,
		Dvc_UncNavigationParticleUpperBound1* upperbound)
{
	DvcLowerBoundValue_=&(lowerbound->Value);//DvcRandomPolicy.Value
	DvcUpperBoundValue_=&(upperbound->Value);//DvcUncNavigationParticleUpperBound1.Value
	DvcChooseEdge_=&(lowerbound->Edge);
	DvcParticleLowerBound_Value_=&(b_lowerbound->Value);//DvcTrivialParticleLowerBound.Value

	//DvcPolicyValue_=&();
	//Dvc_DSPOMDP::DvcModelFreeList_=&(dvc->Dvc_FreeList);
}
__global__ void PassPolicyGraph_nav(int graph_size, int num_edges_per_node, int* action_nodes, int* obs_edges)
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
			//model = new UncNavigation(options[E_PARAMS_FILE].arg);
		} else {
			int size = 7, number = 8;
			if (options[E_SIZE])
				size = atoi(options[E_SIZE].arg);
			else {
				if(FIX_SCENARIO)
					size=8;
				else
					size=13;
				//cerr << "Specify map size using --size option" << endl;
				//exit(0);
			}
			if (options[E_NUMBER]) {
				number = atoi(options[E_NUMBER].arg);
			} else {
				number =0;
				//cerr << "Specify number of rocks using --number option" << endl;
				//exit(0);
			}

			model = new UncNavigation(size, number);


		}
		//Globals::config.useGPU=false;
		//logging::level(1);
    return model;
  }

  void InitializedGPUModel(std::string rollout_type, DSPOMDP* Hst_model)
  {
	  HANDLE_ERROR(cudaMalloc((void**)&Dvc, sizeof(Dvc_UncNavigation)));
	  if(rollout_type=="INDEPENDENT")
		  HANDLE_ERROR(cudaMalloc((void**)&inde_lowerbound, sizeof(Dvc_RandomPolicy)));
	  if(rollout_type=="GRAPH")
		  HANDLE_ERROR(cudaMalloc((void**)&graph_lowerbound, sizeof(Dvc_PolicyGraph)));

	  HANDLE_ERROR(cudaMalloc((void**)&b_lowerbound, sizeof(Dvc_TrivialParticleLowerBound)));
	  HANDLE_ERROR(cudaMalloc((void**)&upperbound, sizeof(Dvc_UncNavigationParticleUpperBound1)));

	  PassModelFuncs<<<1,1,1>>>(Dvc);
	  if(rollout_type=="INDEPENDENT")
		  PassActionValueFuncs<<<1,1,1>>>(Dvc,static_cast<Dvc_RandomPolicy*>(inde_lowerbound),b_lowerbound,upperbound);
	  if(rollout_type=="GRAPH")
		  PassValueFuncs<<<1,1,1>>>(static_cast<Dvc_PolicyGraph*>(graph_lowerbound),b_lowerbound,upperbound);
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

	  PassPolicyGraph_nav<<<1,1,1>>>(hostGraph->graph_size_,hostGraph->num_edges_per_node_,
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

	  //PassConfig<<<1,1,1>>>(Dvc_Globals::config);
	 // HANDLE_ERROR(cudaDeviceSynchronize());
  }


  void DeleteGPUModel()
  {
	  HANDLE_ERROR(cudaFree(Dvc));
	  if(inde_lowerbound)HANDLE_ERROR(cudaFree(inde_lowerbound));
	  if(graph_lowerbound)HANDLE_ERROR(cudaFree(graph_lowerbound));
	  HANDLE_ERROR(cudaFree(b_lowerbound));
	  HANDLE_ERROR(cudaFree(upperbound));
  }

  void DeleteGPUGlobals()
  {
      HANDLE_ERROR(cudaFree(Dvc_Globals::config));
      //ClearConfig<<<1,1,1>>>();
     // HANDLE_ERROR(cudaDeviceSynchronize());
  }

  void DeleteGPUPolicyGraph()
  {
	  //ClearPolicyGraph<<<1,1,1>>>();
	  //HANDLE_ERROR(cudaDeviceSynchronize());
  }

  void InitializeDefaultParameters() {
	Globals::config.GPUid=0;//default GPU
	Globals::config.useGPU=true;
	Globals::config.use_multi_thread_=false;
	Globals::config.NUM_THREADS=10;
	Globals::config.sim_len=60;
	Globals::config.time_per_move=3;
	Globals::config.num_scenarios=5000;
	Globals::config.discount=0.983;

	Obs_type=OBS_LONG64;

	Globals::config.exploration_mode=UCT;
	Globals::config.exploration_constant=0.8;
	Globals::config.exploration_constant_o=0.01;


	//DESPOT::num_Obs_element_in_GPU=1+ModelParams::N_PED_IN*2+2;

	}
};

int main(int argc, char* argv[]) {



  return TUI().run(argc, argv);
}*/



