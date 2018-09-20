/* 
 * File:   danger_tag.h
 * Author: neha
 *
 * Created on June 19, 2018, 2:51 PM
 */

#ifndef DANGER_TAG_H
#define	DANGER_TAG_H

#include "laser_tag/laser_tag.h"
namespace despot {
class DangerTag : public LaserTag {
public:
    DangerTag();
    DangerTag(std::string params_file);
    virtual ~DangerTag();
    
    //static double DEFAULT_TAG_REWARD = 10;
    //static double DEFAULT_DANGER_PENALTY = -3;
    //static double DEFAULT_MOVEMENT_ERROR = 0.0;
    //static double DEFAULT_OBSERVE_REWARD = 0.0;
    
    static const int NUM_DIRECTIONS = 8;
    
    Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;

    State* CreateStartState(std::string type) const;
    
    int NumActions() const {
		return NUM_DIRECTIONS + 1;
	}
	// Last action is Tag
    int TagAction() const {
		return NUM_DIRECTIONS;
	}
    

    void PrintAction(int action, std::ostream& out) const;
    


        inline ValuedAction GetBestAction() const
        {
            return ValuedAction(0,DANGER_PENALTY);
        }

private:

};
}
#endif	/* DANGER_TAG_H */

