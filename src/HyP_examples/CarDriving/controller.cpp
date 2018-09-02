#include <string>
#include <despot/planner.h>


#include "simulator.h"
#include "controller.h"
#include "ped_pomdp.h"
#include "prior.h"
#include "custom_particle_belief.h"

using namespace despot;
static DSPOMDP* ped_pomdp_model;
static SolverPrior* ped_pomdp_prior;


DSPOMDP* DrivingController::InitializeModel(option::Option* options) {
	DSPOMDP* model = new PedPomdp();
	static_cast<PedPomdp*>(model)->world_model=&Simulator::worldModel;


	ped_pomdp_model= model;


	return model;
}

World* DrivingController::InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
   //Create a custom world as defined and implemented by the user
	driving_simulator_=new Simulator(static_cast<DSPOMDP*>(model),
			Globals::config.root_seed/*random seed*/);

	model->InitGPUModel();

   //Establish connection with external system
	driving_simulator_->Connect();
   //Initialize the state of the external system
	driving_simulator_->Initialize();

	prior_ = static_cast<PedPomdp*>(model)->CreateSolverPrior(driving_simulator_, "NEURAL");
	logi << "Created solver prior " << typeid(*prior_).name() << endl;
	ped_pomdp_prior = prior_;
	//static_cast<PedPomdp*>(model)->world_model=&Simulator::worldModel;
   return driving_simulator_;
}

void DrivingController::InitializeDefaultParameters() {
	Globals::config.time_per_move = (1.0/ModelParams::control_freq) * 0.9;
	Globals::config.num_scenarios=1000;
	Globals::config.discount=0.95;
	Globals::config.sim_len=2;
	Globals::config.pruning_constant= 0.001;

	Globals::config.max_policy_sim_len=20;
	//Globals::config.search_depth = 30;

	Globals::config.GPUid=1;//default GPU
	Globals::config.useGPU=true	;
	Globals::config.use_multi_thread_=true;
	
	Globals::config.NUM_THREADS=5;

	Globals::config.exploration_mode=UCT;
	Globals::config.exploration_constant=0.3;
	Globals::config.exploration_constant_o = 1.0;

	Globals::config.silence=false;
	Obs_type=OBS_INT_ARRAY;
	DESPOT::num_Obs_element_in_GPU=1+ModelParams::N_PED_IN*2+2;

	logging::level(2);
}

std::string DrivingController::ChooseSolver(){
	return "DESPOT";
}



bool DrivingController::RunStep(Solver* solver, World* world, Logger* logger) {

	cout << "=========== Customized RunStep ============" << endl;

	logger->CheckTargetTime();

	double step_start_t = get_time_second();

	double start_t = get_time_second();

	// Sample new particles using belieftracker
	static_cast<PedPomdpBelief*>(solver->belief())->ResampleParticles(static_cast<const PedPomdp*>(ped_pomdp_model));

	ACT_TYPE action = solver->Search().action;
	double end_t = get_time_second();
	double search_time = (end_t - start_t);
	logi << "[RunStep] Time spent in " << typeid(*solver).name()
			<< "::Search(): " << search_time << endl;

	OBS_TYPE obs;
	start_t = get_time_second();
	bool terminal = world->ExecuteAction(action, obs);
	end_t = get_time_second();
	double execute_time = (end_t - start_t);
	logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;

	start_t = get_time_second();
	solver->BeliefUpdate(action, obs);


	///////////////////////// TODO : create and initialize prior_ for Controller /////////////////////////////
	assert(ped_pomdp_prior->history().Size());

	const State* cur_state=world->GetCurrentState();
	State* search_state =static_cast<const PedPomdp*>(ped_pomdp_model)->CopyForSearch(cur_state);//create a new state for search

	static_cast<PedPomdpBelief*>(solver->belief())->DeepUpdate(
			ped_pomdp_prior->history_states(),
			ped_pomdp_prior->history_states_for_search(),
			cur_state,
			search_state, action);

	ped_pomdp_prior->Add(action, cur_state);
	ped_pomdp_prior->Add_in_search(-1, search_state);

	end_t = get_time_second();
	double update_time = (end_t - start_t);
	logi << "[RunStep] Time spent in Update(): " << update_time << endl;

	return logger->SummarizeStep(step_++, round_, terminal, action, obs,
			step_start_t);
}

int main(int argc, char* argv[]) {

  return DrivingController().RunPlanning(argc, argv);
}
