/* 
 * File:   LearningSolverBase.h
 * Author: neha
 *
 * Created on September 5, 2016, 3:47 PM
 */

#ifndef LEARNINGSOLVERBASE_H
#define	LEARNINGSOLVERBASE_H

#include <despot/core/solver.h>
#include "LearningModel.h"
class LearningSolverBase : public Solver {
public:
    LearningSolverBase(const LearningModel* model, Belief* belief = NULL) : Solver(model, belief){}
    
    ~LearningSolverBase(){}
    
    History* get_history()
    {
        return &history_;
    }

    
  
protected:
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
    
    int adaboostIdsToModelIds[10] = {6, 4, 7, 3, 1, 9, 2, 8, 5, 0};
};


#endif	/* LEARNINGSOLVERBASE_H */

