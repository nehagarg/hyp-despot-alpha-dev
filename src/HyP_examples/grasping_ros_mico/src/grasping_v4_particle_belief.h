#ifndef GRASPING_V4_PARTICLE_BELIEF_H
#define	GRASPING_V4_PARTICLE_BELIEF_H

#include <despot/core/particle_belief.h>

using namespace despot;

class GraspingParticleBelief : public ParticleBelief {
public:   
    GraspingParticleBelief(std::vector<State*> particles, const DSPOMDP* model,
	Belief* prior, bool split) : ParticleBelief(particles, model,
	 prior, split)
    {
        
    }
    
    std::string text() const {
	std::ostringstream oss;
	std::map<int, double> pdf;
	//oss << "Particles:" << endl;
	for (int i = 0; i < particles_.size(); i ++) {
		 //model_->PrintState(*particles_[i], oss);
		 //oss << " " << i << " = " << *particles_[i] << endl;;
		//pdf[particles_[i]->text()] += particles_[i]->weight;
                pdf[i] += particles_[i]->weight;
	}

	oss << "pdf for " << particles_.size() << " particles:" << std::endl;
        //oss << "pdf for " << pdf.size() << " particles:" << endl;
	std::vector<std::pair<int, double> > pairs = SortByValue(pdf);
	int i = 0;
        double max_value;
        for (auto& it : pairs) {
	//	oss << " " << it.first << " = " << it.second << endl;
                oss << " "<< it.second << " ";
                model_->PrintState(*particles_[it.first], oss);
                if(i==0){
                    max_value = it.second;
                    
                }
                
                if (max_value > it.second)
                {
                   //  i++;
                 //  break;
                }
                i++;
                if (i>=100)
                {
                    break; //Print only top 500 particles to reduce the file size
                }
	}
        //oss << i << " paricles with max_weight" << endl;
	return oss.str();
}
};
#endif	/* GRASPING_V4_PARTICLE_BELIEF_H */
