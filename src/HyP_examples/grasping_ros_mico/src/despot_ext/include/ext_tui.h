/* 
 * File:   grasping_tui.h
 * Author: neha
 *
 * Created on October 25, 2016, 3:01 PM
 */

#ifndef EXT_TUI_H
#define	EXT_TUI_H


#include <despot/planner.h>
#include <despot/util/optionparser.h>
#include "DeepLearningSolver.h"
//#include "LearningPlanningSolver.h"
//#include "PlanningDiffParamsSolver.h"
#include "LearningModel.h"
#include "UserDefinedActionSolver.h"
#include "GraspingPOMDPWorld.h"
//#include "DespotWithLearnedDefaultPolicy.h"

using namespace despot;

/*class POMDPEvaluatorExt : public POMDPEvaluator {
public:
    POMDPEvaluatorExt(DSPOMDP* model, std::string belief_type,
	Solver* solver, clock_t start_clockt, std::ostream* out,
	double target_finish_time, int num_steps):
        POMDPEvaluator(model, belief_type, solver, start_clockt, out,
	target_finish_time, num_steps)
        {
            
        }
        
    POMDPEvaluatorExt(DSPOMDP* model, std::string belief_type,
	Solver* solver, clock_t start_clockt, std::ostream* out):
        POMDPEvaluator(model, belief_type, solver, start_clockt, out)
        {
            
        }

    bool ExecuteAction(int action, double& reward, OBS_TYPE& obs) {
        std::cout << *(solver_->belief()) << std::endl;
	double random_num = random_.NextDouble();
	bool terminal = ((LearningModel*)model_)->StepActual(*state_, random_num, action, reward, obs);

	reward_ = reward;
	total_discounted_reward_ += Globals::Discount(step_) * reward;
	total_undiscounted_reward_ += reward;

	return terminal;
    }

};*/



class TUI: public Planner {
public:
  TUI() ;
  virtual ~TUI();
  

  /*virtual void InitializeEvaluator(Evaluator *&simulator,
                                    option::Option *options, DSPOMDP *model,
                                    Solver *solver, int num_runs,
                                    clock_t main_clock_start,
                                    std::string simulator_type, std::string belief_type,
                                    int time_limit, std::string solver_type); 
  */
  Solver* InitializeSolver(DSPOMDP* model, Belief* belief,
  			std::string solver_type, option::Option* options);
 // virtual Solver* InitializeSolver(DSPOMDP *model, Belief* belief,std::string solver_type,
 //                                   option::Option *options) ;
 
  
   virtual DSPOMDP* InitializeModel(option::Option* options) = 0;

  
  virtual void InitializeDefaultParameters();

  World* InitializeWorld(std::string& world_type, DSPOMDP* model, option::Option* options){
    //return InitializePOMDPWorld(world_type, model, options);
		//Create world: use POMDP model
		world_type = "pomdp";
		GraspingPOMDPWorld* world = new GraspingPOMDPWorld(model, Seeds::Next());
		//Establish connection: do nothing
		world->Connect();
		//Initialize: create start state
		world->Initialize();
		return world;
  }

  std::string ChooseSolver(){ // Specify the solver used in the planner to be DESPOT
    return "DESPOT";
  }

};



  TUI::TUI() {
  }
  
  TUI::~TUI() {}

  /*void TUI::InitializeEvaluator(Evaluator *&simulator,
                                    option::Option *options, DSPOMDP *model,
                                    Solver *solver, int num_runs,
                                    clock_t main_clock_start,
                                    std::string simulator_type, std::string belief_type,
                                    int time_limit, std::string solver_type) {

      //
  if (time_limit != -1) {
      //std::cout << "If Initializing new evaluator ##############" << std::endl;
    simulator =
        new POMDPEvaluatorExt(model, belief_type, solver, main_clock_start, &std::cout,
                           EvalLog::curr_inst_start_time + time_limit,
                           num_runs * Globals::config.sim_len);
  } else {
      //std::cout << "Else Initializing new evaluator ##############" << std::endl;
    simulator =
        new POMDPEvaluatorExt(model, belief_type, solver, main_clock_start, &std::cout);
  }
}*/
  
  
  Solver* TUI::InitializeSolver(DSPOMDP *model, Belief* belief,std::string solver_type,
                                    option::Option *options) {
      Solver *solver = NULL;
      /*if(solver_type == "LEARNINGPLANNING")
      {
          DESPOT *despotSolver = (DESPOT *)InitializeSolver(model, belief,"DESPOT",  options);
          solver = new LearningPlanningSolver((LearningModel*)model, despotSolver->lower_bound(), despotSolver->upper_bound(), NULL);
      }
      else*/ if (solver_type == "DEEPLEARNING") {
		solver = new DeepLearningSolver((LearningModel*)model, NULL);
        }
      /*else if(solver_type == "DESPOTWITHLEARNEDPOLICY")
      {
          DESPOT *despotSolver = (DESPOT *)InitializeSolver(model,belief, "DESPOT",  options);
          solver = new DespotWithLearnedDefaultPolicy((LearningModel*)model, despotSolver->lower_bound(), despotSolver->upper_bound(), NULL);
      } 
      else if (solver_type == "PLANNINGDIFFPARAMS") {
          DESPOT *despotSolver = (DESPOT *)InitializeSolver(model, belief,"DESPOT",  options);
          solver = new PlanningDiffParamsSolver((LearningModel*)model, despotSolver->lower_bound(), despotSolver->upper_bound(), NULL);

		
        }
      */
      else if (solver_type == "USERDEFINED") {
		solver = new UserDefinedActionSolver((LearningModel*)model, NULL);
        }
      else {
          solver = Planner::InitializeSolver(model,belief,solver_type, options);
      }
      return solver;
      
  }
 
  
  
  void TUI::InitializeDefaultParameters() {
  }




#endif	/* EXT_TUI_H */

