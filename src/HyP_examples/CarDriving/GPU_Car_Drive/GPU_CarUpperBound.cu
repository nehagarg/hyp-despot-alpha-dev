#include "GPU_CarUpperBound.h"
#include <despot/GPUinterface/GPUupper_bound.h>

/*Implemented in GPU_Car_Drive.cu*/
/*
__global__ void PassActionValueFuncs(
		Dvc_PedPomdpParticleUpperBound1* upperbound)
{
	DvcUpperBoundValue_ = &(upperbound->Value);
}


HOST void Dvc_PedPomdpParticleUpperBound1::AssignFunctionPointers(){

	PassActionValueFuncs<<<1,1,1>>>(this);

	HANDLE_ERROR(cudaDeviceSynchronize());
}*/