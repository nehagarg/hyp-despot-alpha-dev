/* 
 * File:   DespotWithBeliefTracking.h
 * Author: neha
 *
 * Created on April 23, 2018, 1:32 PM
 */

#ifndef DESPOTWITHBELIEFTRACKING_H
#define	DESPOTWITHBELIEFTRACKING_H

#include <despot/solver/despot.h>
#include "LearningModel.h"
#include "LearningPlanningSolver.h"


class DespotStaticFunctionOverrideHelperForBeliefTracking: public DespotStaticFunctionOverrideHelper {
public:
    DespotStaticFunctionOverrideHelperForBeliefTracking(){
        //std::cout << "Initializing o helper extension" << std::endl;
        //name = "learnig planing";
    }
    virtual ~DespotStaticFunctionOverrideHelperForBeliefTracking(){}


    virtual void Expand(QNode* qnode, 
            ScenarioLowerBound* lower_bound, 
            ScenarioUpperBound* upper_bound, 
            const DSPOMDP* model, RandomStreams& streams, 
            History& history, ScenarioLowerBound* learned_lower_bound, 
            SearchStatistics* statistics, 
            DespotStaticFunctionOverrideHelper* o_helper);

 




}; 

class DespotWithBeliefTracking : public DESPOT{
public:

     DespotWithBeliefTracking(const LearningModel* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);

    virtual ~DespotWithBeliefTracking();

private:

};


#endif	/* DESPOTWITHBELIEFTRACKING_H */

