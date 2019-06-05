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
  std::vector<tensorflow::Session *> transition_model_sessions;
  std::vector<tensorflow::Session *> observation_model_sessions;
};



#endif /* KERASMODELS_H_ */
