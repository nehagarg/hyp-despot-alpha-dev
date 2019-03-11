/* 
 * File:   LearningModel1.cpp
 * Author: neha
 * 
 * Created on September 18, 2015, 2:54 PM
 */

#include "LearningModel.h"
#include "yaml-cpp/yaml.h"


LearningModel::LearningModel() {
 

}

LearningModel::LearningModel(std::string modelParamFileName, std::string problem_name_): problem_name(problem_name_){
    if (modelParamFileName.empty())
    {
        return;
    }
    YAML::Node config = YAML::LoadFile(modelParamFileName);
    if(config["learned_model_name"])
    {
        learned_model_name = config["learned_model_name"].as<std::string>();
    }
    
    if(config["switching_method"])
    {
        automatic_switching_method = config["switching_method"].as<int>();
    }
    
    if(config["svm_model_prefix"])
    {
        svm_model_prefix = config["svm_model_prefix"].as<std::string>();
    }
    
    if(config["switching_threshold"])
    {
        switch_threshold = config["switching_threshold"].as<int>();
    }
    if(config["hand_defined_actions"])
    {
        std::string hand_defined_action_string  = config["hand_defined_actions"].as<std::string>();
        std::stringstream stream(hand_defined_action_string);
        int n;
        while(stream >> n)
        {
            hand_defined_actions.push_back(n);
        }
    }
}


LearningModel::~LearningModel() {
}

void LearningModel::GenerateAdaboostTestFile(uint64_t obs, History h) const {

}

int LearningModel::GetStartStateIndex() const {
    return -1;
}

bool LearningModel::StepActual(State& state, double random_num, int action, double& reward, uint64_t& obs) const {
    return Step(state, random_num, action, reward, obs);
}

std::string LearningModel::GetPythonExecutionString() const {
    std::string cmd_string = "cd python_scripts/deepLearning ; python model.py " + to_string((int)(GetStartStateIndex())) + "; cd - ;";
    return cmd_string;
}

void LearningModel::GetInputSequenceForLearnedmodel(History h, std::ostream& oss) const {

}

std::string LearningModel::GetPythonExecutionString(History h) const {
    std::ostringstream oss;
    
    //if (next_action !=-1)
    //{
    //    oss << "echo " << next_action ;
    //    next_action = -1;
    //}
    //else
    //{
        oss << "cd python_scripts/deepLearning ; python model.py -p " << problem_name << " -a test -i ";
        GetInputSequenceForLearnedmodel(h, oss);
        oss << "-m " << learned_model_name<< " ; cd - ;" ;
    //}       
    return oss.str();
}

   
std::string LearningModel::GetPythonExecutionStringForJointTraining(History h) const
  {
      std::ostringstream oss;
      oss << "cd python_scripts/deepLearning ; python joint_training_model.py -p " << problem_name << " -a test -i  ";
      GetInputSequenceForLearnedmodel(h, oss);
      oss << "-m " << learned_model_name ;
      oss << " -o " << svm_model_prefix << " ; cd - ;" ;
      return oss.str();
  }
double LearningModel::GetUncertaintyValue(Belief* b) const {
    return -1.0;
}

ValuedAction LearningModel::GetNextActionFromUser(History h) const {
    if (h.Size() < hand_defined_actions.size())
    {
        return ValuedAction(hand_defined_actions[h.Size()],1);
    }
    else
    {
        int next_action;
        std::cout << "Input next action" << std::endl;
        std::cin >> next_action;
        return ValuedAction(next_action, 1);
    }
}



int LearningModel::GetSwitchingMethod() const {
    return automatic_switching_method;
}

void LearningModel::SetStoreObsHash(bool value) const {
    store_obs_hash = value;
}

int LearningModel::NumObservations() const
{
	return -1;
}

