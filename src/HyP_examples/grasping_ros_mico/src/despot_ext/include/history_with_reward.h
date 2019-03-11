/* 
 * File:   learned_data.h
 * Author: neha
 *
 * Created on November 7, 2014, 2:28 PM
 */

#ifndef HISTORY_WITH_REWARD_H
#define	HISTORY_WITH_REWARD_H

#include <despot/core/history.h>
#include <despot/interface/pomdp.h>
#include "observation.h"
using namespace despot;

// This class encapsulates a history of actions and observations.
class HistoryWithReward : public History {
public:
	void Add(int action, uint64_t obs, double reward) {
            History::Add(action, obs);
            rewards_.push_back(reward);
	}

	void RemoveLast() { 
            History::RemoveLast();
            rewards_.pop_back();
		
	}

	void Truncate(int d) {
            History::Truncate(d);
		rewards_.resize(d);
		
	}
        
        double Reward(int i)
        {
            if(i == 0)
            {
                return rewards_.back();
            }
            return (rewards_.back() - rewards_[i-1]);
        }
        

        ObservationClass GetInitialObs()
        {
            return initial_obs;
        }
        
        void SetInitialObs(uint64_t obs)
        {
            initial_obs = ObservationClass(obs);
        }
        
        void SetInitialObs(ObservationClass obs)
        {
            initial_obs = obs;
        }
        
        State* GetInitialState()
        {
            return initial_state;
        }
        void SetInitialState(State* state)
        {
            initial_state = state;
        }
        
        void Print(std::ostream& out = std::cout)
        {
            out<< "Initial Observation:" << initial_obs.GetHash() << std::endl;
            for(int i = 0; i < History::Size(); i ++)
            {
                out << "(" << Action(i) << "," << Observation(i) << "," << Reward(i) << ")," ;
            }
            out << std::endl;
        }



private:
  std::vector<double> rewards_; //cumulative reward
  ObservationClass initial_obs;
  State * initial_state; //only kept to for printing
  
};


#endif	/* LEARNED_DATA_H */

