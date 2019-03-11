/* 
 * File:   DespotWithLearnedDefaultPolicy.h
 * Author: neha
 *
 * Created on July 13, 2017, 10:12 AM
 */

#ifndef DESPOTWITHLEARNEDDEFAULTPOLICY_H
#define	DESPOTWITHLEARNEDDEFAULTPOLICY_H

#include <despot/solver/despot.h>
#include "LearningModel.h"
#include "LearningPlanningSolver.h"

struct SearchStatisticsExt : SearchStatistics {
	double time_search_learned_policy;
	SearchStatisticsExt() : SearchStatistics(), time_search_learned_policy(0)
        {};

        void print(std::ostream& os) const;


	//friend std::ostream& operator<<(std::ostream& os, const SearchStatisticsExt& statitics);
};

class DespotStaticFunctionOverrideHelperExt: public DespotStaticFunctionOverrideHelper {
public:
    DespotStaticFunctionOverrideHelperExt(){
        //std::cout << "Initializing o helper extension" << std::endl;
        //name = "learnig planing";
    }
    virtual ~DespotStaticFunctionOverrideHelperExt(){}

    

    void InitMultipleLowerBounds(VNode* vnode, ScenarioLowerBound* lower_bound, 
    RandomStreams& streams, History& history, 
    ScenarioLowerBound* learned_lower_bound = NULL, 
    SearchStatistics* statistics = NULL);
 

    double GetTimeNotToBeCounted(SearchStatistics* statistics);
    


    void UpdateHistoryDuringTrial(History& history, VNode* vnode);




}; 

class DespotWithLearnedDefaultPolicy : public DESPOT {
public:
    DespotWithLearnedDefaultPolicy(const LearningModel* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);
	
    

    virtual ~DespotWithLearnedDefaultPolicy();
    


    void Update(int action, OBS_TYPE obs);

    void belief(Belief* b);


     DeepLearningSolver deepLearningSolver;
     LearningModel * getModel(){
         return ((LearningModel*)model_);
     }
protected:

   // ValuedAction Search();

    void InitStatistics();

    void CoreSearch(std::vector<State*> particles, RandomStreams& streams);
    
    //ScenarioLowerBound* learned_policy_lower_bound_;
    LearnedPolicy* deepPolicy;
   

};

#endif	/* DESPOTWITHLEARNEDDEFAULTPOLICY_H */

