/* 
 * File:   DeepLearningSolver.h
 * Author: neha
 *
 * Created on August 25, 2016, 1:43 PM
 */

#ifndef DEEPLEARNINGSOLVER_H
#define	DEEPLEARNINGSOLVER_H
#include "LearningModel.h"
#include "LearningSolverBase.h"
#include "Python.h"


class DeepLearningSolver : public LearningSolverBase {
public:
    DeepLearningSolver(const LearningModel* model, Belief* belief = NULL);
    
    ~DeepLearningSolver();

    virtual ValuedAction Search();
    virtual void Update(int action, uint64_t obs);
    virtual ValuedAction Search(History , int print_info = 1);
    
    /*virtual void Update(int action, ObservationClass obs){
        Solver::Update(action, obs.GetHash());
    }*/
    
    std::vector< PyObject*> rnn_state_history;
    std::vector< PyObject*> rnn_output_history; //required for svm model input
    void TruncateRnnHistories(int t_size);
private:
    PyObject *h_to_a_model;
    PyObject *load_function, *run_function;

    void load_deep_policy(std::string learned_model_name);
    PyObject* get_state_and_next_action(PyObject* rnn_state, int action, uint64_t obs, int print_info = 1);
    
    ValuedAction SearchUsingPythonFunction(History h, int print_info = 1);
    ValuedAction SearchUsingCommandLineCall(History h);
        
/*private:
    std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		result += buffer;
    }
    pclose(pipe);
    return result;
    }
    
    int deepLearningIdsToModelIds[10] = {6, 4, 7, 3, 1, 9, 2, 8, 5, 0}; //This was used for adaboost solver
 */ 
};



#endif	/* DEEPLEARNINGSOLVER_H */

