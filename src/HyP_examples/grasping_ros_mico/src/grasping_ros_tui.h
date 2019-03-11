/* 
 * File:   grasping_tui.h
 * Author: neha
 *
 * Created on October 25, 2016, 3:01 PM
 */

#ifndef GRASPING_ROS_TUI_H
#define	GRASPING_ROS_TUI_H


#include "ext_tui.h"
#include "grasping_real_arm.h"
#include "ros/ros.h"

using namespace despot;

/*class POMDPEvaluatorWithDisplay : public POMDPEvaluatorExt {
public:
    POMDPEvaluatorWithDisplay(DSPOMDP* model, std::string belief_type,
	Solver* solver, clock_t start_clockt, std::ostream* out,
	double target_finish_time, int num_steps):
        POMDPEvaluatorExt(model, belief_type, solver, start_clockt, out,
	target_finish_time, num_steps)
        {
            
        }
        
    POMDPEvaluatorWithDisplay(DSPOMDP* model, std::string belief_type,
	Solver* solver, clock_t start_clockt, std::ostream* out):
        POMDPEvaluatorExt(model, belief_type, solver, start_clockt, out)
        {
            
        }

    void InitRound() {
        POMDPEvaluatorExt::InitRound();
        ((GraspingRealArm*)model_)->DisplayBeliefs((ParticleBelief* )(solver_->belief()),std::cout);
        ((GraspingRealArm*)model_)->DisplayState(*state_, std::cout);
        
    }
    bool ExecuteAction(int action, double& reward, OBS_TYPE& obs) {

        bool ans = POMDPEvaluatorExt::ExecuteAction(action, reward, obs);
       ((GraspingRealArm*)model_)->DisplayBeliefs((ParticleBelief* )(solver_->belief()),std::cout);
        ((GraspingRealArm*)model_)->DisplayState(*state_, std::cout);
        return ans;
    }
    

};
*/
class RosTUI: public TUI {
public:
  RosTUI() {
  }



  /*void InitializeEvaluator(Evaluator *&simulator,
                                    option::Option *options, DSPOMDP *model,
                                    Solver *solver, int num_runs,
                                    clock_t main_clock_start,
                                    std::string simulator_type, std::string belief_type,
                                    int time_limit, std::string solver_type) {

      //
  if (time_limit != -1) {
      //std::cout << "If Initializing new evaluator ##############" << std::endl;
    simulator =
        new POMDPEvaluatorWithDisplay(model, belief_type, solver, main_clock_start, &std::cout,
                           EvalLog::curr_inst_start_time + time_limit,
                           num_runs * Globals::config.sim_len);
  } else {
      //std::cout << "Else Initializing new evaluator ##############" << std::endl;
    simulator =
        new POMDPEvaluatorWithDisplay(model, belief_type, solver, main_clock_start, &std::cout);
  }
}*/
  

 
  
    DSPOMDP* InitializeModel(option::Option* options) ;
    /*{
     DSPOMDP* model;
    /*if (options[E_DATA_FILE])
            {
                if (options[E_NUMBER]) {
                        int number = atoi(options[E_NUMBER].arg);
                        
                        model = new GraspingV4(options[E_DATA_FILE].arg, number);
                }
                else
                {
                    model = new GraspingV4(options[E_DATA_FILE].arg, -1);
                }
            }
            else
    
     //{
                if (options[E_NUMBER]) {
                        int number = atoi(options[E_NUMBER].arg);
                        std::cout << "Number is " <<  number;
                        model = new GraspingV4( number);
                }
                else
                {
                    model = new GraspingV4(-1);
                }
     //       }
     
    return model;
  }*/


};



DSPOMDP* InitializeModelCommon(option::Option* options) {
     DSPOMDP* model;
     
      /*if (options[E_DATA_FILE])
            {
                if (options[E_NUMBER]) {
                        int number = atoi(options[E_NUMBER].arg);
                        model = new GraspingRealArm(options[E_DATA_FILE].arg, number);
                }
                else
                {
                    model = new GraspingRealArm(options[E_DATA_FILE].arg, -1);
                }
            }
            else
            {
       
        */
       int number = -1;
       if (options[E_NUMBER]) {
                number = atoi(options[E_NUMBER].arg);
        }
        
       if (options[E_PARAMS_FILE]) {
                    
           std::cout << "Config file is " << options[E_PARAMS_FILE].arg << std::endl;
                       
                        model = new GraspingRealArm(options[E_PARAMS_FILE].arg, number );
                }
                else
                {
                    model = new GraspingRealArm(number);
                }
        //    }
   
     
    return model;
  }

void main_init(int argc, char* argv[]){

    ros::init(argc,argv,"despot" + to_string(getpid()));
}



#endif	/* GRASPING_ROS_TUI_H */

