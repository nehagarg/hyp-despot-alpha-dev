/* 
 * File:   danger_tag.cpp
 * Author: neha
 * 
 * Created on June 19, 2018, 2:51 PM
 */

#include "danger_tag.h"
namespace despot {
    DangerTag::DangerTag() : LaserTag(){

    }

    DangerTag::DangerTag(std::string params_file) : LaserTag(params_file) {

    }
    
    DangerTag::~DangerTag() {

    }

    State* DangerTag::CreateStartState(std::string type) const {
	int n, opp_start_index, rob_start_index;

	if (opp_start_input == NULL) {
		opp_start_index = Random::RANDOM.NextInt(floor_.NumCells());
	} else {
		opp_start_index = floor_.GetIndex(*opp_start_input);
	}

	if (rob_start_positions.size() == 0) {
		rob_start_index = Random::RANDOM.NextInt(floor_.NumCells());
	} else {
		rob_start_index = rob_start_positions[Random::RANDOM.NextInt(rob_start_positions.size())];
	}

	n = RobOppIndicesToStateIndex(rob_start_index, opp_start_index);

	return new TagState(*states_[n]);
    }
    
    Belief* DangerTag::InitialBelief(const State* start, std::string type) const {
            assert(start != NULL);

            int start_opp = 0;
            int end_opp = floor_.NumCells();
            if (opp_start_input != NULL) {
		
		start_opp= floor_.GetIndex(*opp_start_input);
                end_opp = start_opp + 1;
	}
            std::vector<State*> particles;
            for (int opp = start_opp; opp < end_opp; opp++) {
                    if (rob_start_positions.size() == 0) {
                            for (int rob = 0; rob < floor_.NumCells(); rob++) {
                                    TagState* state = static_cast<TagState*>(Allocate(RobOppIndicesToStateIndex(rob, opp), 1.0 / ((end_opp-start_opp) * floor_.NumCells())));
                                    particles.push_back(state);
                            }
                    } else {
                            for (int robi = 0; robi < rob_start_positions.size(); robi++) {
                                    TagState* state = static_cast<TagState*>(Allocate(RobOppIndicesToStateIndex(rob_start_positions[robi], opp), 1.0 / ((end_opp-start_opp) * rob_start_positions.size())));
                                    particles.push_back(state);
                            }
                    }
            }

            ParticleBelief* belief = new ParticleBelief(particles, this);
            belief->state_indexer(this);
            return belief;
    }
    
    void DangerTag::PrintAction(int action, std::ostream& out) const {
        if (action < TagAction()) {
		out << Compass::CompassString[action] << std::endl;
	} else {
		out << "Tag" << std::endl;
	}

    }
        

}
