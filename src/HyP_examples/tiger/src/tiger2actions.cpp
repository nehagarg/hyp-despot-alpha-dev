/* 
 * File:   tiger2actions.cpp
 * Author: neha
 * 
 * Created on April 26, 2018, 9:04 AM
 */

#include "tiger2actions.h"
using namespace std;

namespace despot {
const int Tiger2actions::LISTEN2 = 3;
const int Tiger2actions::OBS_SCALE = 10000;
const double Tiger2actions::LISTEN_MEAN = 0.15;
const double Tiger2actions::LISTEN2_MEAN = 0.4;
const double Tiger2actions::LISTEN_VARIANCE = 0.1;

int Tiger2actions::NumActions() const {
    return 4;
}

  int Tiger2actions::NumObservations() const {
    //cout<<__FUNCTION__<<": Obs space too large! INF used instead"<<endl;
    return std::numeric_limits<int>::max();
  }
bool Tiger2actions::StepOld(State& s, double random_num, int action, double& reward, OBS_TYPE& obs) const {
        TigerState& state = static_cast<TigerState&>(s);
	bool terminal = false;

	if (action == LEFT || action == RIGHT) {
		reward = state.tiger_position != action ? 10 : -100;
		state.tiger_position = random_num <= 0.5 ? LEFT : RIGHT;
		obs = 2*Tiger2actions::OBS_SCALE; // can use arbitary observation
                terminal = true;
	} else

        {
            double lower_limit;
            double upper_limit;
            int prob_sign = 1;
            if(state.tiger_position == Tiger::LEFT)
            {
                prob_sign = -1;
            }
            if (action == Tiger::LISTEN){

		reward = -1;


                lower_limit = 0.5  +(prob_sign*Tiger2actions::LISTEN_MEAN) - Tiger2actions::LISTEN_VARIANCE;
                upper_limit = 0.5 + (prob_sign*Tiger2actions::LISTEN_MEAN) + Tiger2actions::LISTEN_VARIANCE;

            }
            if (action == Tiger2actions::LISTEN2){

		reward = -1.2;
		lower_limit = 0.5  +(prob_sign*Tiger2actions::LISTEN2_MEAN) - Tiger2actions::LISTEN_VARIANCE;
                upper_limit = 0.5  +(prob_sign*Tiger2actions::LISTEN2_MEAN) + Tiger2actions::LISTEN_VARIANCE;

            }
            double real_obs = (random_num*(upper_limit-lower_limit)) + lower_limit;
            obs = round(real_obs*Tiger2actions::OBS_SCALE);
        }

	return terminal;
}

bool Tiger2actions::Step(State& s, double random_num, int action, double& reward, OBS_TYPE& obs) const {
        TigerState& state = static_cast<TigerState&>(s);
	bool terminal = false;

	if (action == LEFT || action == RIGHT) {
		reward = state.tiger_position != action ? 10 : -100;
		state.tiger_position = random_num <= 0.5 ? LEFT : RIGHT;
		obs = 2*Tiger2actions::OBS_SCALE; // can use arbitary observation
                terminal = true;
	} else 
            
        {

	  double prob_tiger_right = 0.0;
            if (action == Tiger::LISTEN){
            
            	reward = -1;
                
                prob_tiger_right = 0.5  +Tiger2actions::LISTEN_MEAN;


            }   
            if (action == Tiger2actions::LISTEN2){
            
            	reward = -1.2;
				prob_tiger_right = 0.5  +(Tiger2actions::LISTEN2_MEAN);
            }
            double prob_bucket_double = random_num * Tiger2actions::OBS_SCALE;
	    int prob_bucket = (int)prob_bucket_double;
	    double remaining_prob = prob_bucket_double - prob_bucket;
	    prob_tiger_right = prob_tiger_right + (Tiger2actions::LISTEN_VARIANCE*(double)prob_bucket / (double)Tiger2actions::OBS_SCALE);

            if(remaining_prob > prob_tiger_right)
            {
            	prob_tiger_right = 1-prob_tiger_right;
            }
            if(state.tiger_position == Tiger::LEFT)
            {
            	prob_tiger_right = 1-prob_tiger_right;
            }
            //double real_obs = (random_num*(upper_limit-lower_limit)) + lower_limit;
            obs = int(prob_tiger_right*Tiger2actions::OBS_SCALE*10);
        }
        
	return terminal;
}

double Tiger2actions::ObsProb(OBS_TYPE obs, const State& s, int a) const {
        const TigerState& state = static_cast<const TigerState&>(s);

	if (a < Tiger::LISTEN)
		return obs == 2*Tiger2actions::OBS_SCALE;

        double obs_prob = (1.0*obs)/(Tiger2actions::OBS_SCALE*10);
	return state.tiger_position == Tiger::LEFT ? (1-obs_prob):obs_prob;
}

void Tiger2actions::PrintAction(int action, std::ostream& out) const {
    if (action == Tiger::LEFT) {
		out << "Open left" << endl;
	} else if (action == Tiger::RIGHT) {
		out << "Open right" << endl;
	} else if (action == Tiger::LISTEN){
		out << "Listen1" << endl;
	}else if (action == Tiger2actions::LISTEN2){
            out << "Listen2" << endl;
        }
            
}

void Tiger2actions::PrintObs(const State& state, OBS_TYPE obs, std::ostream& out) const {
    out << obs << endl;
}

}
