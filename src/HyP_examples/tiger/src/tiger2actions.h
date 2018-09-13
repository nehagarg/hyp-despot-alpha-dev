/* 
 * File:   tiger2actions.h
 * Author: neha
 *
 * Created on April 26, 2018, 9:04 AM
 */

#ifndef TIGER2ACTIONS_H
#define	TIGER2ACTIONS_H
#include "tiger.h"
namespace despot {
class Tiger2actions : public Tiger {
public:
    static const int LISTEN2;
    static const int OBS_SCALE;
    static const double LISTEN_MEAN;
    static const double LISTEN2_MEAN;
    static const double LISTEN_VARIANCE;
    
    
    
    Tiger2actions(){};

    virtual ~Tiger2actions() {};
    
    bool Step(State& s, double random_num, int action, double& reward,
		OBS_TYPE& obs) const;
    int NumActions() const;
    double ObsProb(OBS_TYPE obs, const State& s, int a) const;
    
    void PrintObs(const State& state, OBS_TYPE obs, std::ostream& out = std::cout) const;
    void PrintAction(int action, std::ostream& out = std::cout) const;

private:
    
    
};
}
#endif	/* TIGER2ACTIONS_H */

