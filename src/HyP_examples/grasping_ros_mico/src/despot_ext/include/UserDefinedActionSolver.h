/* 
 * File:   UserDefinedAction.h
 * Author: neha
 *
 * Created on March 23, 2017, 11:09 AM
 */

#ifndef USERDEFINEDACTION_H
#define	USERDEFINEDACTION_H
#include <despot/core/solver.h>
#include "LearningModel.h"

class UserDefinedActionSolver : public Solver
{
public:

    UserDefinedActionSolver(const LearningModel* model, Belief* belief = NULL) :
    Solver(model, belief) {
        
    }
    
    ~UserDefinedActionSolver(){}
    

    ValuedAction Search()
    {
        return ((LearningModel*)model_)->GetNextActionFromUser(history_);
    }

    void BeliefUpdate(int action, OBS_TYPE obs){
        history_.Add(action, obs);
        //belief_->Update(action, obs); //Useful for debuggin
    }
    
    void belief(Belief* b) {

	belief_ = b;
	history_.Truncate(0);

    }

};

#endif	/* USERDEFINEDACTION_H */

