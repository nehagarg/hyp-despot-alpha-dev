/*
 * KerasModels.cpp
 *
 *  Created on: May 27, 2019
 *      Author: neha
 */

#include "KerasModels.h"
#include "inference_cc.h"
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/public/session_options.h>
#include <fstream>
#include "ActionSpecification.h"
#include "RobotInterface.h"


//using namespace despot;

KerasModels::KerasModels(int num_actions) {
	for(int i = 0; i < num_actions; i++)
	{
		//tensorflow::Session * observation_sess;
		//tensorflow::Session * transition_sess;
		transition_model_sessions.push_back(NULL);
		observation_model_sessions.push_back(NULL);
	}
//Assuming maximum 500 particles
	int batch_size = 500;
	tensorflow::TensorShape data_shape({batch_size, 1});
	tensorflow::Tensor prob_data(tensorflow::DT_FLOAT, data_shape);
	auto data_ = prob_data.flat<float>().data();
	for (int i = 0; i < batch_size; ++i){
	  data_[i + 0] = 0;
	}

	tensorflow::Tensor terminal_data(tensorflow::DT_FLOAT, data_shape);
	auto terminal_data_ = terminal_data.flat<float>().data();
	for (int i = 0; i < batch_size; ++i){
	  terminal_data_[i + 0] = 1;
	}

	tensorflow::Tensor reward_data(tensorflow::DT_FLOAT, data_shape);
	auto reward_data_ = reward_data.flat<float>().data();
	for (int i = 0; i < batch_size; ++i){
	  reward_data_[i + 0] = -1000;
	}
	default_observation_prob_output.push_back(prob_data);
	default_transition_output.push_back(prob_data); //Pushing in place of next state as it will not be accessed for default case
	default_transition_output.push_back(terminal_data);
	default_transition_output.push_back(reward_data);
	default_transition_output.push_back(prob_data); //Pushing in place of next obs as it will not be accessed for default case


}

KerasModels::~KerasModels() {

}

void KerasModels::load_keras_models()
{

	std::stringstream gpuID_str;
	gpuID_str << RobotInterface::gpuID;
	for(int i = 0; i < transition_model_sessions.size(); i ++)
	{
		std::stringstream sstr;
		sstr << "scripts/transition_model/full_transition_model_v1_";
		sstr << i;
		sstr << ".pb";
//const std::string graph_fn = "./transition/decoder_transition_model.pb";
				const std::string graph_fn = sstr.str()	;
	  const std::string checkpoint_fn = "";
	  std::ifstream file(graph_fn);
	  // prepare session
	  if(file)
	  {
		  tensorflow::Session * transition_sess;
		  transition_model_sessions[i] = transition_sess;
		  std::cout << "Loading file" << graph_fn << std::endl;
		  tensorflow::SessionOptions options;
		  options.config.mutable_gpu_options()->set_visible_device_list(gpuID_str.str());
		  TF_CHECK_OK(tensorflow::NewSession(options, &transition_model_sessions[i]));
		  TF_CHECK_OK(LoadModel(transition_model_sessions[i], graph_fn, checkpoint_fn));
	  }
	  else
	  {
		  std::cout << "File not found" << graph_fn << std::endl;
	  }
	}


	for(int i = 0; i < observation_model_sessions.size(); i ++)
		{
			std::stringstream sstr;
			sstr << "scripts/observation_model/full_observation_model_v1_";
			sstr << i;
			sstr << ".pb";
	//const std::string graph_fn = "./transition/decoder_transition_model.pb";
					const std::string graph_fn = sstr.str()	;
		  const std::string checkpoint_fn = "";
		  std::ifstream file(graph_fn);
		  // prepare session

		  if(file)
		  {
			  tensorflow::Session * observation_sess;
			  observation_model_sessions[i] = observation_sess;
			  std::cout << "Loading file" << graph_fn << std::endl;
			  tensorflow::SessionOptions options;
			  options.config.mutable_gpu_options()->set_visible_device_list(gpuID_str.str());
			  TF_CHECK_OK(tensorflow::NewSession(options, &observation_model_sessions[i]));
			  TF_CHECK_OK(LoadModel(observation_model_sessions[i], graph_fn, checkpoint_fn));
		  }
		  else
		  {
			  std::cout << "File not found" << graph_fn << std::endl;
		  }
		}
}


void KerasModels::run_observation_session(const std::vector<float>& obs_batch, const std::vector<float>& all_particle_batch, int action,
			std::vector<float>&random_number_vecctor, std::vector<tensorflow::Tensor>& outputs) const
{
	double start_t = despot::get_time_second();
	if(observation_model_sessions[action] != NULL)
	{
		//std::cout << "Running observation model for action " << action << std::endl;
		//std::cout << "Obs batch size " << obs_batch.size() << std::endl;
		//std::cout << "particle batch size " << all_particle_batch.size() << std::endl;
		int batch_size = all_particle_batch.size()/RobotInterface::KerasInputVectorSize();
		tensorflow::TensorShape state_input_shape({batch_size, RobotInterface::KerasInputVectorSize()});
		tensorflow::TensorShape obs_input_shape({batch_size, RobotInterface::KerasObservationVectorSize()});
		tensorflow::Tensor state_input(tensorflow::DT_FLOAT, state_input_shape);
		tensorflow::Tensor obs_input(tensorflow::DT_FLOAT, obs_input_shape);

		std::copy_n(all_particle_batch.begin(),all_particle_batch.size(),state_input.flat<float>().data());
		std::copy_n(obs_batch.begin(),obs_batch.size(),obs_input.flat<float>().data());
		tensor_dict feed_dict = {
		      {"input_state1:0", state_input},{"obs_input:0", obs_input}
		  };
		TF_CHECK_OK(
				observation_model_sessions[action]->Run(feed_dict, {"model_1_1/final_dese_layer/BiasAdd:0"}, {}, &outputs));
	}
	else
	{
		outputs = default_observation_prob_output;
	}
	double end_t = despot::get_time_second();
	//std::cout << "Observation Time taken for action " << action << " " << end_t - start_t << std::endl;
}
	void KerasModels::run_transition_session(const std::vector<float>& keras_particle_batch, int action, std::vector<float>&random_number_vecctor,
			std::vector<tensorflow::Tensor>& outputs)
	{
		double start_t = despot::get_time_second();
		if(transition_model_sessions[action] != NULL)
			{
			//std::cout << "Running transition model for action " << action << std::endl;
			//std::cout << "Particle batch size " << keras_particle_batch.size() << std::endl;
				int batch_size = keras_particle_batch.size()/RobotInterface::KerasInputVectorSize();
				tensorflow::TensorShape state_input_shape({batch_size, RobotInterface::KerasInputVectorSize()});
				tensorflow::Tensor state_input(tensorflow::DT_FLOAT, state_input_shape);
				std::copy_n(keras_particle_batch.begin(),keras_particle_batch.size(),state_input.flat<float>().data());

				if(action == A_PICK)
				{
					tensorflow::TensorShape random_input_shape({batch_size, 1});
					tensorflow::Tensor random_input(tensorflow::DT_FLOAT, random_input_shape);
					std::copy_n(random_number_vecctor.begin(),random_number_vecctor.size(),random_input.flat<float>().data());
					tensor_dict feed_dict = {
					      {"input_state_1:0", state_input},{"random_number:0", random_input}
					  };
					//Getiing reward in place of next state and obs as they will not be used
					TF_CHECK_OK(
									transition_model_sessions[action]->Run(feed_dict, {"pick_action_reward/Select_1:0",
											"get_terminal_value_pick/ones_like:0", "pick_action_reward/Select_1:0","pick_action_reward/Select_1:0" }, {}, &outputs));
					//std::cout << "Run transition session" << std::endl;
				}
				else {
					tensorflow::TensorShape random_input_shape({batch_size, RobotInterface::LatentDimensionSize()});
					tensorflow::Tensor random_input(tensorflow::DT_FLOAT, random_input_shape);
					std::copy_n(random_number_vecctor.begin(),random_number_vecctor.size(),random_input.flat<float>().data());
					if(action == A_CLOSE) //trained with prob_decoder
					{
						tensor_dict feed_dict = {
												  {"input_state_6:0", state_input},{"z_sampling_6:0", random_input}
											  };
						TF_CHECK_OK(
									transition_model_sessions[action]->Run(feed_dict, {"full_model/concatenate_next_state_output/concat:0",
																	"full_model/get_terminal_value_non_pick/zeros_like:0", "full_model/close_action_reward/Select_2:0", "decoder_6/dense_12/BiasAdd:0"}, {}, &outputs));
					}
					else
					{
						tensor_dict feed_dict = {
										{"input_state_5:0", state_input},{"z_sampling_5:0", random_input}
										};
						if(action == A_OPEN)
						{
						TF_CHECK_OK(
								transition_model_sessions[action]->Run(feed_dict, {"full_model/concatenate_next_state_output/concat:0",
																					"full_model/get_terminal_value_non_pick/zeros_like:0",
																					"full_model/open_action_reward/Select_2:0",
																					"decoder_5/dense_12/BiasAdd:0"}, {}, &outputs));
						}
						else
						{
							TF_CHECK_OK(
								transition_model_sessions[action]->Run(feed_dict, {"full_model/concatenate_next_state_output/concat:0",
																					"full_model/get_terminal_value_non_pick/zeros_like:0",
																					"full_model/move_action_reward/Select_2:0",
																					"decoder_5/dense_12/BiasAdd:0"}, {}, &outputs));
						}
					}
				}
			}


			else
			{
				outputs = default_transition_output;
			}
		double end_t = despot::get_time_second();
		//std::cout << "Transition Time taken for action " << action << " " << end_t - start_t << std::endl;
		//std::cout << "Transition reward for action " << action << " " << outputs[2].flat<float>().data()[0]<< std::endl;
	}


