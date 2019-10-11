/* 
 * File:   DespotWithLearnedDefaultPolicy.cpp
 * Author: neha
 * 
 * Created on July 13, 2017, 10:12 AM
 */

#include "DespotWithLearnedDefaultPolicy.h"

DespotWithLearnedDefaultPolicy::DespotWithLearnedDefaultPolicy(const LearningModel* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief)
: DESPOT(model, lb, ub, belief),
  deepLearningSolver(model)
{
    std::cout << "Initiallizing despot with learned policy solver ##################" << std::endl;
    deepPolicy = new LearnedPolicy(model_, model_->CreateParticleLowerBound(), belief_, &deepLearningSolver);
    o_helper_ = new DespotStaticFunctionOverrideHelperExt();
    o_helper_->solver_pointer = this;
}



DespotWithLearnedDefaultPolicy::~DespotWithLearnedDefaultPolicy() {
}

void DespotWithLearnedDefaultPolicy::InitStatistics() {
    //std::cout << "Initiallizing statistics with learned policy solver ##################" << std::endl;
    
    statistics_ = new SearchStatisticsExt();
}

void DespotWithLearnedDefaultPolicy::CoreSearch(std::vector<State*> particles, RandomStreams& streams) {
    //std::cout << "Initiallizing contruct tree with learned policy solver ##################" << std::endl;
    //DESPOT::CoreSearch(particles, streams);

    //if(history_.Size()>= 28)
    //{
    std::cout << "Rnn state history size and history size " << 
            deepPolicy->learnedSolver->rnn_state_history.size() << "," 
            << history_.Size() << 
            "," << deepPolicy->learnedSolver->get_history()->Size() << std::endl;
    ((LearningModel*)model_)->SetStoreObsHash(true);
    root_ = ConstructTree(particles, streams, lower_bound_, upper_bound_,
		model_, history_, Globals::config.time_per_move, statistics_, deepPolicy, o_helper_);
    ((LearningModel*)model_)->SetStoreObsHash(false);
    std::cout << "Rnn state history size and history size " << 
            deepPolicy->learnedSolver->rnn_state_history.size() << "," 
            << history_.Size() << 
            "," << deepPolicy->learnedSolver->get_history()->Size() << std::endl;
    //}
    //else
    //{
    //    DESPOT::CoreSearch(particles, streams);
    // }
}

/*ValuedAction DespotWithLearnedDefaultPolicy::Search() {
    int actions[] = {1,3,3,3,3,3,3,3, 3, 3, 2, 2, 1, 2, 2, 3, 2, 2,1, 1,1,1,1,1,1,1,1,0};

    
    std::cout << "Rnn state history size and history size " << 
            deepPolicy->learnedSolver->rnn_state_history.size() << "," 
            << history_.Size() << 
            "," << deepPolicy->learnedSolver->get_history()->Size() << std::endl;
    ValuedAction ans = DESPOT::Search();
    std::cout << "Rnn state history size and history size " << 
            deepPolicy->learnedSolver->rnn_state_history.size() << "," 
            << history_.Size() << 
            "," << deepPolicy->learnedSolver->get_history()->Size() << std::endl;
    if(history_.Size() < 28)
    {
        ans.action = actions[history_.Size()];
        std::cout << "Ans action" << history_.Size() << std::endl;
        std::cout << ans.action << std::endl;
    }
    return ans;
}
 */


void DespotWithLearnedDefaultPolicy::Update(int action, OBS_TYPE obs) {
    //std::cout << "Updating belief" << std::endl;
    deepLearningSolver.Update(action, obs);
    //std::cout << "Before Updating despot" << std::endl;
    DESPOT::Update(action,obs);
}

void DespotWithLearnedDefaultPolicy::belief(Belief* b) {
     deepPolicy->belief(b);
     DESPOT::belief(b);     
}

void DespotStaticFunctionOverrideHelperExt::InitMultipleLowerBounds(VNode* vnode, 
        ScenarioLowerBound* lower_bound, RandomStreams& streams, 
        History& history, ScenarioLowerBound* learned_lower_bound,
        SearchStatistics* statistics)
{
    //std::cout << "Initiallizing multiple lower bounds with learned policy solver ##################" << std::endl;
    DeepLearningSolver * deepPointer =
                &(((DespotWithLearnedDefaultPolicy*)solver_pointer)->deepLearningSolver);
    
    //std::cout << "Getting rnn state history and history size "  
    //        << deepPointer->rnn_state_history.size() << "," << history.Size() << std::endl;
    (((DespotWithLearnedDefaultPolicy*)solver_pointer)->getModel())->SetStoreObsHash(false);
    DESPOT::InitLowerBound(vnode,lower_bound, streams,history);
    (((DespotWithLearnedDefaultPolicy*)solver_pointer)->getModel())->SetStoreObsHash(true);
    //std::cout << "Returning history size after default bound"  
    //       << deepPointer->rnn_state_history.size() << ","  << history.Size() << std::endl;
   
    //double default_policy_lower_bound = vnode->lower_bound();
    double start = clock();
    
    DESPOT::InitLowerBound(vnode,learned_lower_bound, streams,history);
    
    if(vnode->rnn_state == NULL)
    {
        //DeepLearningSolver * deepPointer =
        //        &(((DespotWithLearnedDefaultPolicy*)solver_pointer)->deepLearningSolver);
        
        //assert(history.Size() == deepPointer->rnn_state_history.size() );
        //deepPointer->Search(history,0);
        if(deepPointer->rnn_state_history.size() > history.Size())
        {
            vnode->rnn_state = deepPointer->rnn_state_history[history.Size()];
            Py_INCREF(vnode->rnn_state);
            vnode->rnn_output = deepPointer->rnn_output_history[history.Size()];
            Py_INCREF(vnode->rnn_output);
        }
        else
        {
            assert(vnode->depth() == Globals::config.search_depth);
        }
    }
    else
    {
        std::cout << "Rnn state is not null after init bounds. This should not happen" << std::endl;
        assert(0==1);
    }
    double time_used =  double(clock() - start) / CLOCKS_PER_SEC;
    //Update time used by learned policy lower bound calculation so that it can be deducted later
    if (vnode->parent() !=NULL)
    {
        ((SearchStatisticsExt *) statistics)->time_search_learned_policy = 
            ((SearchStatisticsExt *) statistics)->time_search_learned_policy + 
            time_used;
    }
    //std::cout << "Returning history size after learned bound "  
    //      << deepPointer->rnn_state_history.size() << ","   << history.Size() << std::endl;
}

double DespotStaticFunctionOverrideHelperExt::GetTimeNotToBeCounted(SearchStatistics* statistics) 
{
    //std::cout << "Calling from learning planning" << std::endl;
    return ((SearchStatisticsExt *) statistics)->time_search_learned_policy;
}

void DespotStaticFunctionOverrideHelperExt::UpdateHistoryDuringTrial(History& history, VNode* vnode) {
    
    DeepLearningSolver * deepPointer =
                &(((DespotWithLearnedDefaultPolicy*)solver_pointer)->deepLearningSolver);
    
    
    //std::cout << "Rnn history size at update during trial "  
    //      << deepPointer->rnn_state_history.size() << ","   << history.Size() << std::endl;

    

    DespotStaticFunctionOverrideHelper::UpdateHistoryDuringTrial(history,vnode);
    deepPointer->TruncateRnnHistories(history.Size());
    if(vnode->rnn_state !=NULL)
        {
            deepPointer->rnn_state_history.push_back(vnode->rnn_state);
            Py_INCREF(vnode->rnn_state);
            deepPointer->rnn_output_history.push_back(vnode->rnn_output);
            Py_INCREF(vnode->rnn_output);
        }
        else
        {
            assert(vnode->depth() == Globals::config.search_depth);
        }
    
}

void SearchStatisticsExt::print(std::ostream& os) const
{
    //std::cout << "Printing from ext search statistics      " << std::endl;
    os << "Extra time used by learned policy " << time_search_learned_policy << std::endl;
    SearchStatistics::print(os);
	
}
