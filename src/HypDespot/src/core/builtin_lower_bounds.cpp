#include <despot/core/builtin_lower_bounds.h>
#include <despot/interface/pomdp.h>
#include <despot/core/node.h>
#include <despot/solver/pomcp.h>

using namespace std;

namespace despot {

/* =============================================================================
 * POMCPScenarioLowerBound class
 * =============================================================================*/

POMCPScenarioLowerBound::POMCPScenarioLowerBound(const DSPOMDP* model,
	POMCPPrior* prior,
	Belief* belief) :
	ScenarioLowerBound(model/*, belief*/),
	prior_(prior) {
	explore_constant_ = model_->GetMaxReward()
		- model_->GetBestAction().value;
}

ValuedAction POMCPScenarioLowerBound::Value(const vector<State*>& particles,
	RandomStreams& streams, History& history) const {
	prior_->history(history);
	VNode* root = POMCP::CreateVNode(0, particles[0], prior_, model_);
	// Note that particles are assumed to be of equal weight
	for (int i = 0; i < particles.size(); i++) {
		State* particle = particles[i];
		State* copy = model_->Copy(particle);
		POMCP::Simulate(copy, streams, root, model_, prior_);
		model_->Free(copy);
	}

	ValuedAction va = POMCP::OptimalAction(root);
	va.value *= State::Weight(particles);
	delete root;
	return va;
    }

    ValuedAction POMCPScenarioLowerBound::Value(const std::vector<State*>& particles, RandomStreams& streams, History& history, std::vector<double>& alpha_vector_lower_bound) const {
        cerr << __FUNCTION__ << " function hasn't been defined yet!" << endl;
    }

/* =============================================================================
 * TrivialParticleLowerBound class
 * =============================================================================*/

TrivialParticleLowerBound::TrivialParticleLowerBound(const DSPOMDP* model) :
	ParticleLowerBound(model) {
}

ValuedAction TrivialParticleLowerBound::Value(
	const vector<State*>& particles) const {
	ValuedAction va = model_->GetBestAction();
	va.value *= State::Weight(particles) / (1 - Globals::Discount());
	return va;
    }

    ValuedAction TrivialParticleLowerBound::Value(const std::vector<State*>& particles, std::vector<double>& alpha_vector_lower_bound) const {
        ValuedAction va = model_->GetBestAction();
        for(int i = 0; i < Globals::config.num_scenarios; i++)
        {
            alpha_vector_lower_bound[i] = va.value/(1 - Globals::Discount());
        }
        va.value_array = &alpha_vector_lower_bound;
        
        return va;
        
    }


/* =============================================================================
 * TrivialBeliefLowerBound class
 * =============================================================================*/

TrivialBeliefLowerBound::TrivialBeliefLowerBound(const DSPOMDP* model) :
	BeliefLowerBound(model) {
}

ValuedAction TrivialBeliefLowerBound::Value(const Belief* belief) const {
	ValuedAction va = model_->GetBestAction();
	va.value *= 1.0 / (1 - Globals::Discount());
	return va;
}

} // namespace despot
