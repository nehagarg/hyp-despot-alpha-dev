/* 
 * File:   LearningModel.h
 * Author: neha
 *
 * Created on September 17, 2015, 4:54 PM
 */

#ifndef LEARNINGMODEL_H
#define	LEARNINGMODEL_H

#include <despot/interface/pomdp.h>
#include "history_with_reward.h"



class LearningModel : public DSPOMDP {
public:
    LearningModel();
    LearningModel(std::string modelParamFileName, std::string problem_name_);
    virtual ~LearningModel();
    virtual std::vector<HistoryWithReward*> LearningData() const = 0;
    virtual uint64_t GetInitialObs() const = 0;
    virtual double GetDistance(int action1, uint64_t obs1, int action2, uint64_t obs2) const = 0;
    virtual void PrintObs(uint64_t obs, std::ostream& out = std::cout) const = 0;
    virtual void GenerateAdaboostTestFile(uint64_t obs, History h) const;
    virtual int GetStartStateIndex() const;
    virtual bool StepActual(State& state, double random_num, int action, double& reward, uint64_t& obs) const;
    virtual std::string GetPythonExecutionString() const;
    virtual std::string GetPythonExecutionString(History h) const;
    virtual std::string GetPythonExecutionStringForJointTraining(History h) const;
    virtual double GetUncertaintyValue(Belief* b) const;
    virtual ValuedAction GetNextActionFromUser(History h) const;

    virtual void GetInputSequenceForLearnedmodel(History h, std::ostream& oss) const;
    virtual int GetSwitchingMethod() const;
    virtual void SetStoreObsHash(bool value) const;
    
    virtual int NumObservations() const;

    std::string learned_model_name = "";
    int automatic_switching_method = 0; // 0  for threshold switching 1 for switching wirh both correct and wrong prediction 2 for switching with only correct prediction
    std::string svm_model_prefix = "";
    int switch_threshold = 10;
    std::string problem_name = "vrep";
    std::vector <int> hand_defined_actions;
    //mutable int next_action = -1; //Obtained from joint model
    mutable bool store_obs_hash = false;
    
    private:
        std::string python_exec(const char* cmd) const {
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
};

#endif	/* LEARNINGMODEL_H */

