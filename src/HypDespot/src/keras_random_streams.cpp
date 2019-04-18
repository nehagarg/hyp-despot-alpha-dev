#include <despot/keras_random_streams.h>
#include <despot/util/seeds.h>
#include <vector>
#include <sstream>
#include <cstring>
#include <assert.h>
using namespace std;

namespace despot {


KerasRandomStreams::KerasRandomStreams(int num_streams, int length, int latent_dimension) :
	RandomStreams(num_streams, length),
	latent_dimension_(latent_dimension){

	if(latent_dimension > 0)
	{
		 //unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

		 std::default_random_engine generator( Globals::config.root_seed);
		 std::normal_distribution<float> distribution(0,1);
		//length = depth
		//num_strems = num_particles
		keras_streams_.resize(length);
		for(int i = 0; i < length; i++)
		{
			keras_streams_[i].resize(num_streams*latent_dimension);
			for(int j = 0; j < (num_streams*latent_dimension); j++)
			{
				keras_streams_[i][j] = distribution(generator);
			}
		}
	}


}


std::vector<float>& KerasALLParticlesEntry(int position = -1) const {
	if(position == -1)
	{
		return keras_streams_[position_];
	}
	else
	{
		return keras_streams_[position];
	}
}



} // namespace despot
