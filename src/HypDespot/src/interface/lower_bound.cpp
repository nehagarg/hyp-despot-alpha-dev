#include <despot/interface/lower_bound.h>
#include <despot/interface/pomdp.h>
#include <despot/core/node.h>
#include <despot/solver/pomcp.h>

using namespace std;

namespace despot {

/* =============================================================================
 * ValuedAction class
 * =============================================================================*/

ValuedAction::ValuedAction() :
	action(-1),
	value(0),
        value_array(NULL){
}

ValuedAction::ValuedAction(ACT_TYPE _action, double _value) :
	action(_action),
	value(_value),
        value_array(NULL) {
}

ValuedAction::ValuedAction(int _action, std::vector<double>* _value_array):
action(_action),
	value_array(_value_array),
        value(0)
{
    
}
ostream& operator<<(ostream& os, const ValuedAction& va) {
	//os << "(" << va.action << ", " << va.value << ")";
	//return os;
        
        os << "(" << va.action << ", " << va.value << ", [";
        if(va.value_array!=NULL)
        {
            for (int i = 0; i < va.value_array->size(); i++)
            {
                os << (*va.value_array)[i] << ", " ;
            }
        
        }
        os << "])" ;
	return os;
}

/* =============================================================================
 * ScenarioLowerBound class
 * =============================================================================*/

ScenarioLowerBound::ScenarioLowerBound(const DSPOMDP* model) :
	model_(model){
}

void ScenarioLowerBound::Init(const RandomStreams& streams) {
}

void ScenarioLowerBound::Reset() {
}

void ScenarioLowerBound::Learn(VNode* tree) {
}

/* =============================================================================
 * ParticleLowerBound class
 * =============================================================================*/

ParticleLowerBound::ParticleLowerBound(const DSPOMDP* model) :
	ScenarioLowerBound(model) {
}

ValuedAction ParticleLowerBound::Value(const vector<State*>& particles,
	RandomStreams& streams, History& history) const {
	return Value(particles);
    }

    ValuedAction ParticleLowerBound::Value(const std::vector<State*>& particles, RandomStreams& streams, History& history, std::vector<double>& alpha_vector_lower_bound) const {
       return Value(particles, alpha_vector_lower_bound);     
    }


/* =============================================================================
 * BeliefLowerBound class
 * =============================================================================*/

BeliefLowerBound::BeliefLowerBound(const DSPOMDP* model) :
	model_(model) {
}

void BeliefLowerBound::Learn(VNode* tree) {
}

} // namespace despot
