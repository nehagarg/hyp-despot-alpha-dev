/* 
 * File:   DespotWithBeliefTracking.h
 * Author: neha
 *
 * Created on April 23, 2018, 1:32 PM
 */

#ifndef DESPOTWITHALPHAFUNCTIONUPDATE_H
#define	DESPOTWITHALPHAFUNCTIONUPDATE_H

#include <despot/solver/despot.h>
namespace despot{


/*class DespotStaticFunctionOverrideHelperForAlphaFunctionUpdate: public DespotStaticFunctionOverrideHelper {
public:
    DespotStaticFunctionOverrideHelperForAlphaFunctionUpdate(){
        //std::cout << "Initializing o helper extension" << std::endl;
        //name = "learnig planing";
    }
    virtual ~DespotStaticFunctionOverrideHelperForAlphaFunctionUpdate(){}


   
    void Expand(QNode* qnode, 
            ScenarioLowerBound* lower_bound, 
            ScenarioUpperBound* upper_bound, 
            const DSPOMDP* model, RandomStreams& streams, 
            History& history, ScenarioLowerBound* learned_lower_bound, 
            SearchStatistics* statistics, 
            DespotStaticFunctionOverrideHelper* o_helper);

    //void Update(QNode* qnode);
    int GetObservationParticleSize(VNode* vnode);
    
    
    


}; 
*/
class DespotWithAlphaFunctionUpdate : public DESPOT{
public:

     DespotWithAlphaFunctionUpdate(const DSPOMDP* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);

    virtual ~DespotWithAlphaFunctionUpdate(){};
    
    //void CoreSearch(std::vector<State*>& particles, RandomStreams& streams);
    
    static void Update(VNode* vnode);
    static void Update(Shared_VNode* vnode, bool real);
    static void UpdateSibling(VNode* vnode, VNode* sibling_node);
    static void UpdateSibling(Shared_VNode* vnode, Shared_VNode* sibling_node, bool real);
    static void Update(Shared_QNode* qnode, bool real);
    static void Update(QNode* qnode);
    static void Expand(QNode* qnode, ScenarioLowerBound* lower_bound,
		ScenarioUpperBound* upper_bound, const DSPOMDP* model,
		RandomStreams& streams, History& history);
    static bool PedPomdpProb;

private:

};

}
#endif	/* DESPOTWITHALPHAFUNCTIONUPDATE_H */

