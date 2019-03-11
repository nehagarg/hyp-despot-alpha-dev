/* 
 * File:   LearningPlanningSolver.h
 * Author: neha
 *
 * Created on September 5, 2016, 3:01 PM
 */

#ifndef PLANNINGDIFFPARAMSSOLVER_H
#define	PLANNINGDIFFPARAMSSOLVER_H

#include "LearningModel.h"
#include <despot/solver/despot.h>



class PlanningDiffParamsSolver : public Solver{
public:
    PlanningDiffParamsSolver(const LearningModel* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL) 
            : Solver(model,belief), despotSolver(model, lb, ub, belief) {}
            //(const LearningModel* model, Belief* belief, RandomStreams& streams);
    
   // LearningPlanningSolver(const LearningModel* model, DESPOT despotSolver_, DeepLearningSolver deepLearningSolver_, Belief* belief = NULL);

    virtual ~PlanningDiffParamsSolver(){}
    

    ValuedAction Search() {
      
        double uncertainty = ((LearningModel*)model_)->GetUncertaintyValue(despotSolver.Solver::belief());
        std::cout << "Belief uncertainty: " << uncertainty << std::endl;
        //int hist_size = history_.Size();
        //if ((hist_size/20) % 2 == 0)
        if(uncertainty > 4.5)
        {
        Globals::config.time_per_move = 10;
         Globals::config.num_scenarios = 500;
        }
        else
        {
           Globals::config.time_per_move = 1;
           Globals::config.num_scenarios = 100; 
        }
        
        return despotSolver.Search();
        
    }
    void Update(int action, uint64_t obs) {
        despotSolver.Update(action, obs);
    
        history_.Add(action, obs);
    }
    //virtual void Update(int action, ObservationClass obs);

    void belief(Belief* b) {
        despotSolver.belief(b);
        belief_ = b;
        history_.Truncate(0);
    }
   

private:
    DESPOT despotSolver;

    
};

#endif	/* LEARNINGPLANNINGSOLVER_H */

