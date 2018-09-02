#include <despot/GPUinterface/GPUlower_bound.h>
#include <despot/GPUcore/GPUbuiltin_lower_bound.h>

#include <despot/GPUinterface/GPUpomdp.h>

using namespace std;

namespace despot {

/* =============================================================================
 * TrivialParticleLowerBound class
 * =============================================================================*/


DEVICE Dvc_ValuedAction Dvc_TrivialParticleLowerBound::Value(
		int scenarioID, Dvc_State * particles) {
	Dvc_ValuedAction va = DvcModelGetBestAction_();
	va.value *= 1.0 / (1 - Dvc_Globals::Dvc_Discount(Dvc_config));
	return va;
}

__global__ void PassLbValueFunc(Dvc_TrivialParticleLowerBound* lowerbound)
{
	DvcParticleLowerBound_Value_=&(lowerbound->Value);
}

HOST void Dvc_TrivialParticleLowerBound::AssignFunctionPointers(){
	PassLbValueFunc<<<1,1,1>>>(this);
	HANDLE_ERROR(cudaDeviceSynchronize());
}


} // namespace despot
