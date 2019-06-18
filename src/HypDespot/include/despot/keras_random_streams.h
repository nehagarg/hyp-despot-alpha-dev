#ifndef KERAS_RANDOM_STREAMS_H
#define KERAS_RANDOM_STREAMS_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <despot/util/random.h>
#include <despot/random_streams.h>
#include <despot/interface/pomdp.h>

namespace despot {

/**
 * A KerasRandomStreams object represents multiple random number sequences, where each
 * entry is independently and identically drawn from normal distributio with 0 mean and 1 variance.
 */
class KerasRandomStreams : public RandomStreams {
//private:
//	mutable int position_;
private:
	int latent_dimension_;

public:
//	  std::vector<std::vector<double> > streams_; // streams_[i] is associated with i-th particle
	std::vector<std::vector<float> > keras_streams_;
	std::vector<std::vector<float> > keras_uniform_streams_;
	// streams_[i][j*lantent_dim] is associated with jth particle at position i. For each particle, latent_dim entries are there
	/**
	 * Constructs multiple random sequences of the same length.
	 *
	 * @param num_streams number of sequences
	 * @param length sequence length
	 */
	KerasRandomStreams(int num_streams, int length, int latent_dim);
	KerasRandomStreams(){;}

	//std::vector<double> KerasParticleEntry(int stream) const;
	//std::vectoy<double> KerasParticleEntry(int stream, int position) const;
	const std::vector<float>& KerasALLParticlesEntry(int position = -1) const;
	const void KerasParticlesEntry(const std::vector<State*>& particles, std::vector<float>& random_vector, bool uniform_stream = false , int position = -1) const;
	const void KerasParticlesEntry(const std::vector<int>& particles, std::vector<float>& random_vector, bool uniform_stream = false, int position = -1) const;
};

} // namespace despot

#endif
