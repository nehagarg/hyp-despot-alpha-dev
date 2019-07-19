/*
 * KerasModels.h
 *
 *  Created on: May 27, 2019
 *      Author: neha
 */

#ifndef KERASMODELS_H_
#define KERASMODELS_H_

#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/public/session_options.h>

class KerasModels {
public:
	KerasModels(int num_actions);
	virtual ~KerasModels();
	void load_keras_models();
	void run_observation_session(const std::vector<float>& keras_particle_batch, const std::vector<float>& keras_obs_particle_batch, int action,
			std::vector<float>&random_number_vecctor, std::vector<tensorflow::Tensor>& outputs) const;
	void run_transition_session(const std::vector<float>& keras_particle_batch, int action, std::vector<float>&random_number_vecctor,
			std::vector<tensorflow::Tensor>& outputs);
	void run_observation_encoder_session(const std::vector<float>& image_batch, int action,	std::vector<tensorflow::Tensor>& outputs);
  std::vector<tensorflow::Session *> transition_model_sessions;
  std::vector<tensorflow::Session *> observation_model_sessions;
  std::vector<tensorflow::Session *> observation_encoder_model_sessions;
  std::vector<tensorflow::Tensor> default_observation_prob_output;
  std::vector<tensorflow::Tensor> default_transition_output;
};



#endif /* KERASMODELS_H_ */
