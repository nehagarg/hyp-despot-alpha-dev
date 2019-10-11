/* 
 * File:   LearningPlanningSolver.h
 * Author: neha
 *
 * Created on September 5, 2016, 3:01 PM
 */

#ifndef LEARNINGPLANNINGSOLVER_H
#define	LEARNINGPLANNINGSOLVER_H

#include "LearningSolverBase.h"
#include "LearningModel.h"
#include "DeepLearningSolver.h"
#include <despot/solver/despot.h>


class LearnedPolicy : public Policy {
    
public:
    DeepLearningSolver* learnedSolver;
    LearnedPolicy(const DSPOMDP* model, ParticleLowerBound* bound, Belief* belief, DeepLearningSolver* learnedSolver_)
    : Policy(model, bound, belief), learnedSolver(learnedSolver_) {
    }

    int Action(const std::vector<State*>& particles,
            RandomStreams& streams, History& history) const {
 //       std::cout << "Taking action in Learned policy with history size " << history.Size() << std::endl;
        
        return (learnedSolver->Search(history, 0)).action;
    }
    ValuedAction Search() {
        //std::cout << "Starting search\n";
	RandomStreams streams(Globals::config.num_scenarios,
		Globals::config.search_depth);
        // std::cout << "Genrated random streams\n";
	std::vector<State*> particles = belief_->Sample(Globals::config.num_scenarios);
        for (int i = 0; i < particles.size(); i++) {
		particles[i]->scenario_id = i;
	}

        //std::cout << "Sampled belief\n";
	//int action = Action(particles, streams, history_);
	//double dummy_value = Globals::NEG_INFTY;
        //TODO: make it compatible with belief tracking solver
        ValuedAction ans = Value(particles,streams,*(learnedSolver->get_history()));

	for (int i = 0; i < particles.size(); i++)
		model_->Free(particles[i]);

	return ans;
    }

};

class LearningPlanningSolver : public LearningSolverBase{
public:
    LearningPlanningSolver(const LearningModel* model, ScenarioLowerBound* lb, ScenarioUpperBound* ub, Belief* belief = NULL);
            //(const LearningModel* model, Belief* belief, RandomStreams& streams);
    
   // LearningPlanningSolver(const LearningModel* model, DESPOT despotSolver_, DeepLearningSolver deepLearningSolver_, Belief* belief = NULL);

    virtual ~LearningPlanningSolver();
    

    virtual ValuedAction Search();
    virtual void Update(int action, uint64_t obs);
    //virtual void Update(int action, ObservationClass obs);
    virtual ValuedAction GetLowerBoundForLearnedPolicy();

    virtual void belief(Belief* b);
   

private:
    PyObject *correct_svm, *wrong_svm;
    PyObject *load_function, *run_function;
    void load_svm_model(std::string svm_model_prefix);
    PyObject* get_svm_model_output(PyObject* rnn_output, int svm_type); //svm type 0 for correct svm, 1 for wrong svm
    bool ShallISwitchFromLearningToPlanning(History h);
    bool ShallISwitchFromPlanningToLearning(History h);
    std::pair<int, int> GetSVMOutput(History h);
    std::pair<int, int> GetSVMOutputUsingPythonFunction(History h);
    std::pair<int, int> GetSVMOutputUsingCommandLine(History h);
    bool SecondSVMRequired();
    
    int switching_method;
    int next_action = -1;
    DESPOT despotSolver;
    DeepLearningSolver deepLearningSolver;
    bool currentSolverLearning;
    LearnedPolicy* deepPolicy;
    
};

#endif	/* LEARNINGPLANNINGSOLVER_H */

/*
 Switching method:
 0 threshold based switching from learned policy to despot policy
 1 2 svms based switching
 2 1 svm based switching
 3 lower bound of deep policy based switching with switch threshold providing policy sim length
 4 lower bound of deep policy with 2 svms check based switching with switch threshold providing policy sim length
 5 lower bound of deep policy with 1 svm check based switching with switch threshold providing policy sim length
 
 
 
 */

