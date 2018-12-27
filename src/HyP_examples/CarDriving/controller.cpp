#include <string>
#include <despot/planner.h>


#include "simulator.h"
#include "controller.h"
#include "ped_pomdp.h"
#include "prior.h"
#include "custom_particle_belief.h"
#include <despot/solver/DespotWithAlphaFunctionUpdate.h>

using namespace despot;
static DSPOMDP* ped_pomdp_model;
static SolverPrior* ped_pomdp_prior;
static ACT_TYPE action = (ACT_TYPE)(-1);
static OBS_TYPE obs =(OBS_TYPE)(-1);

DSPOMDP* DrivingController::InitializeModel(option::Option* options) {
	 DSPOMDP* model = !options[E_PARAMS_FILE] ?
	      new PedPomdp() : new PedPomdp(options[E_PARAMS_FILE].arg);
	 DESPOT::num_Obs_element_in_GPU=1+ModelParams::N_PED_IN*2+2;
	 std::cout << "Use zero car vel correction : " << ModelParams::USE_ZERO_VEL_CORRECTION << std::endl;
	 std::cout << "Num peds for planning : " << ModelParams::N_PED_IN << std::endl;
	//DSPOMDP* model = new PedPomdp();
	static_cast<PedPomdp*>(model)->world_model=&Simulator::worldModel;

	ped_pomdp_model= model;

	return model;
}

World* DrivingController::InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
   //Create a custom world as defined and implemented by the user
	driving_simulator_=new Simulator(static_cast<DSPOMDP*>(model),
			Globals::config.root_seed/*random seed*/);

	if (Globals::config.useGPU)
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
	driving_simulator_->UpdateWorld();
	double end_t = get_time_second();
	double world_update_time = (end_t - start_t);
	logi << "[RunStep] Time spent in UpdateWorld(): " << world_update_time << endl;

	start_t = get_time_second();
	solver->BeliefUpdate(action, obs);

	//assert(ped_pomdp_prior->history().Size());

	const State* cur_state=world->GetCurrentState();

	assert(cur_state);

	if(DESPOT::Debug_mode){
		/*cout << "Current simulator state before belief update: \n";
		static_cast<PedPomdp*>(ped_pomdp_model)->PrintWorldState(static_cast<const PomdpStateWorld&>(*cur_state));*/
	}

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
	start_t = get_time_second();

	// Sample new particles using belieftracker
	static_cast<PedPomdpBelief*>(solver->belief())->ResampleParticles(static_cast<const PedPomdp*>(ped_pomdp_model));

	action = solver->Search().action;
	end_t = get_time_second();
	double search_time = (end_t - start_t);
	logi << "[RunStep] Time spent in " << typeid(*solver).name()
			<< "::Search(): " << search_time << endl;

	cout << "act= " << action << endl;

	start_t = get_time_second();
	bool terminal = world->ExecuteAction(action, obs);
	end_t = get_time_second();
	double execute_time = (end_t - start_t);
	logi << "[RunStep] Time spent in ExecuteAction(): " << execute_time << endl;



	return logger->SummarizeStep(step_++, round_, terminal, action, obs,
			step_start_t);
}

void DrivingController::PlanningLoop(Solver*& solver, World* world, Logger* logger) {
	bool terminal;
	for (int i = 0; i < Globals::config.sim_len; i++) {
		terminal = RunStep(solver, world, logger);
		if (terminal)
			break;
	}

	if(!terminal){
		cout << "- final_state:\n";
		static_cast<PedPomdp*>(ped_pomdp_model)->PrintWorldState(
				static_cast<PomdpStateWorld&>(*driving_simulator_->GetCurrentState()), cout);
	}
}


int main(int argc, char* argv[]) {

	DespotWithAlphaFunctionUpdate::PedPomdpProb = true;
  return DrivingController().RunPlanning(argc, argv);
}


// ./devel/lib/hyp_despot/hyp_despot_CarDriving -s 200 -t 0.5 --nobs 25 -n 500 --max-policy-simlen 6 -v 3 --GPUID 1 --solver=BTDESPOTALPHAST --econst 0.95 --oeconst 0.01 -l DONOTHING -d 10 > ../tools/Release_21Nov_updated_gpu_do_nothing/HypDespot_Car_Driving_Results_BTDESPOTALPHAST/DoNothing/t05/n500/nobs25/d10/trial_7192.log
//./devel/lib/hyp_despot/hyp_despot_CarDriving -s 200 -t 0.5 -n 1000 --max-policy-simlen 90 -v 3 --GPUID 0 > ../tools/Release_9Nov/HypDespot_Car_Driving_Results_DESPOT/SMART_sim-length-90_zero_carvel_correction/t05/n1000/trial_{}.log
// rsync -avz neha@unicorn4.d2.comp.nus.edu.sg:/data/neha/WORK_FOLDER/AdaCompNUS/catkin_ws/src/Hyp_despot/tools ./
//seq 500 | xargs -i echo "./devel/lib/hyp_despot/hyp_despot_danger_tag  -v 3 -m ../src/HyP_examples/tag/config_files/danger_tag/openSingleR14x14-movementError6.txt -l DANGERTAG -t 15 -n 50  --solver=BTDESPOTALPHAST --GPU 0 --CPU 0 > ../tools/HypDespot_Danger_Tag_Results_BTDESPOTALPHAST/openSingleR14x14-movementError6/Release_9Nov/t15_n50/trial_{}.log 2>&1" | bash
//./devel/lib/hyp_despot/hyp_despot_unknav -n 100 --nobs 25 --solver=BTDESPOTALPHAST --CPU 1 --num_thread 10 --GPU 1 --GPUID 2 -v 3 -t 0.25 -l RANDOM -m ../src/HyP_examples/unkown_navigation/config_files/new_belief_16bits_02noise.txt -d 60
//./devel/lib/hyp_despot/hyp_despot_mars --CPU 1 --num_thread 10 --GPU 1 --GPUID 0 -v 3 --solver=BTDESPOTALPHAST --nobs 10 -n 50 -m ../src/HyP_examples/ma_rock_sample/config_files/continuous_obs_2agents.txt -t 1
