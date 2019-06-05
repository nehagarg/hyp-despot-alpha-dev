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

//using namespace despot;

KerasModels::KerasModels(int num_actions) {
	for(int i = 0; i < num_actions; i++)
	{
		tensorflow::Session * observation_sess;
		tensorflow::Session * transition_sess;
		transition_model_sessions.push_back(transition_sess);
		observation_model_sessions.push_back(observation_sess);
	}

}

KerasModels::~KerasModels() {

}

void KerasModels::load_keras_models()
{

	for(int i = 0; i < transition_model_sessions.size(); i ++)
	{
		std::stringstream sstr;
		sstr << "scripts/transition_model/vae_transition_model_";
		sstr << i;
		sstr << ".pb";
//const std::string graph_fn = "./transition/decoder_transition_model.pb";
				const std::string graph_fn = sstr.str()	;
	  const std::string checkpoint_fn = "";

	  // prepare session

	  tensorflow::SessionOptions options;
	  TF_CHECK_OK(tensorflow::NewSession(options, &transition_model_sessions[i]));
	  TF_CHECK_OK(LoadModel(transition_model_sessions[i], graph_fn, checkpoint_fn));
	}


	for(int i = 0; i < observation_model_sessions.size(); i ++)
		{
			std::stringstream sstr;
			sstr << "scripts/observation_model/vae_observation_model_";
			sstr << i;
			sstr << ".pb";
	//const std::string graph_fn = "./transition/decoder_transition_model.pb";
					const std::string graph_fn = sstr.str()	;
		  const std::string checkpoint_fn = "";

		  // prepare session

		  tensorflow::SessionOptions options;
		  TF_CHECK_OK(tensorflow::NewSession(options, &observation_model_sessions[i]));
		  TF_CHECK_OK(LoadModel(observation_model_sessions[i], graph_fn, checkpoint_fn));
		}
}


