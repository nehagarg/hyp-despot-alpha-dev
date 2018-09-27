#include <despot/interface/default_policy.h>
#include <despot/interface/pomdp.h>
#include <unistd.h>
#include <despot/GPUcore/thread_globals.h>

using namespace std;

namespace despot {

/* =============================================================================
 * DefaultPolicy class
 * =============================================================================*/

DefaultPolicy::DefaultPolicy(const DSPOMDP* model, ParticleLowerBound* particle_lower_bound) :
	ScenarioLowerBound(model),
	particle_lower_bound_(particle_lower_bound) {
	assert(particle_lower_bound_ != NULL);
}

DefaultPolicy::~DefaultPolicy() {
    }

    ValuedAction DefaultPolicy::Value(const std::vector<State*>& particles, RandomStreams& streams, History& history, std::vector<double>& alpha_vector_lower_bound) const {
        return Value(particles, streams, history, alpha_vector_lower_bound, true);
    }

    ValuedAction DefaultPolicy::Value(const vector<State*>& particles,
	RandomStreams& streams, History& history) const
    {
        std::vector<double> dummy;
        return Value(particles,streams,history, dummy, false);
    }
    ValuedAction DefaultPolicy::Value(const std::vector<State*>& particles, RandomStreams& streams, History& history, std::vector<double>& alpha_vector_lower_bound, bool compute_alpha_vector) const 
 {
	vector<State*> copy;
	for (int i = 0; i < particles.size(); i++)
		copy.push_back(model_->Copy(particles[i]));

	initial_depth_ = history.Size();
	ValuedAction va = RecursiveValue(copy, streams, history, alpha_vector_lower_bound, compute_alpha_vector);

	for (int i = 0; i < copy.size(); i++)
		model_->Free(copy[i]);

	return va;
}

ValuedAction DefaultPolicy::RecursiveValue(const vector<State*>& particles,
	RandomStreams& streams, History& history, std::vector<double>& alpha_vector_lower_bound, bool compute_alpha_vector) const {
	if (streams.Exhausted()
		|| (history.Size() - initial_depth_
			>= Globals::config.max_policy_sim_len)) {

            if(compute_alpha_vector)
            {
                return particle_lower_bound_->Value(particles, alpha_vector_lower_bound);
            }
            else
            {
		return particle_lower_bound_->Value(particles);
            }
	} else {
		ACT_TYPE action = Action(particles, streams, history);

                OBS_TYPE obs;
		double reward;
                if(compute_alpha_vector) //Assuming state independent action policy here.
                {
                    ValuedAction va = ValuedAction(action,0.0);
                    va.value_array = &alpha_vector_lower_bound;
                    
                    std::vector<State*> new_particles;
                    for (int i = 0; i < particles.size(); i++) {
                        State* particle = particles[i];
			bool terminal = model_->Step(*particle,
				streams.Entry(particle->scenario_id), action, reward, obs);
                        (*va.value_array)[particle->scenario_id] += reward;
                        if(!terminal)
                        {
                            new_particles.push_back(particle);
                        }
                        
                    }
		    if(new_particles.size() > 0)
		      {
			history.Add(action, obs);
			streams.Advance();
			std::vector<double> va1_alpha_vector;
			va1_alpha_vector.resize(Globals::config.num_scenarios, 0);
			ValuedAction va1 = RecursiveValue(new_particles, streams, history, va1_alpha_vector, compute_alpha_vector);
			for(int i = 0; i < new_particles.size(); i++)
			  {
			    State* particle = new_particles[i];
			    (*va.value_array)[particle->scenario_id] += Globals::Discount() * (*va1.value_array)[particle->scenario_id];
                        
			  }
			va1.value_array->clear();
			//delete va1.value_array;
			streams.Back();
			history.RemoveLast();
		      }
                    return va;
                }
		double value = 0;

		map<OBS_TYPE, vector<State*> > partitions;
		
		for (int i = 0; i < particles.size(); i++) {
			State* particle = particles[i];
			bool terminal = model_->Step(*particle,
				streams.Entry(particle->scenario_id), action, reward, obs);

			value += reward * particle->weight;

			if (!terminal) {
				partitions[obs].push_back(particle);
			}
		}

	    if(DoPrintCPU) printf("action, ave_reward= %d %f\n",action,value);


		for (map<OBS_TYPE, vector<State*> >::iterator it = partitions.begin();
			it != partitions.end(); it++) {
			OBS_TYPE obs = it->first;
			history.Add(action, obs);
			streams.Advance();
			ValuedAction va = RecursiveValue(it->second, streams, history, alpha_vector_lower_bound, compute_alpha_vector);
			value += Globals::Discount() * va.value;
			streams.Back();
			history.RemoveLast();
		}

		return ValuedAction(action, value);
	}
}

void DefaultPolicy::Reset() {
}

ParticleLowerBound* DefaultPolicy::particle_lower_bound() const {
	return particle_lower_bound_;
}

} // namespace despot
