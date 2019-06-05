#ifndef INFERENCECC_H_
#define INFERENCECC_H_

//Adapted from  2018, Patrick Wieschollek <mail@patwie.com>
#include <tensorflow/core/protobuf/meta_graph.pb.h>
#include <tensorflow/core/public/session.h>
#include <tensorflow/core/public/session_options.h>
#include <iostream>
#include <string>
#include <despot/util/util.h>
typedef std::vector<std::pair<std::string, tensorflow::Tensor>> tensor_dict;

/**
 * @brief load a previous store model
 * @details [long description]
 *
 * in Python run:
 *
 *    saver = tf.train.Saver(tf.global_variables())
 *    saver.save(sess, './exported/my_model')
 *    tf.train.write_graph(sess.graph, '.', './exported/graph.pb, as_text=False)
 *
 * this relies on a graph which has an operation called `init` responsible to
 * initialize all variables, eg.
 *
 *    sess.run(tf.global_variables_initializer())  # somewhere in the python
 * file
 *
 * @param sess active tensorflow session
 * @param graph_fn path to graph file (eg. "./exported/graph.pb")
 * @param checkpoint_fn path to checkpoint file (eg. "./exported/my_model",
 * optional)
 * @return status of reloading
 */
tensorflow::Status LoadModel(tensorflow::Session *sess, std::string graph_fn,
                             std::string checkpoint_fn = "") {
  tensorflow::Status status;

  // Read in the protobuf graph we exported
  //tensorflow::MetaGraphDef graph_def;
  tensorflow::GraphDef graph_def;
  status = ReadBinaryProto(tensorflow::Env::Default(), graph_fn, &graph_def);
  if (status != tensorflow::Status::OK()) return status;

  // create the graph in the current session
  //status = sess->Create(graph_def.graph_def());
  status = sess->Create(graph_def);
  if (status != tensorflow::Status::OK()) return status;

  // restore model from checkpoint, iff checkpoint is given
  /*if (checkpoint_fn != "") {
    const std::string restore_op_name = graph_def.saver_def().restore_op_name();
    const std::string filename_tensor_name =
        graph_def.saver_def().filename_tensor_name();

    tensorflow::Tensor filename_tensor(tensorflow::DT_STRING,
                                       tensorflow::TensorShape());
    filename_tensor.scalar<std::string>()() = checkpoint_fn;

    tensor_dict feed_dict = {{filename_tensor_name, filename_tensor}};
    status = sess->Run(feed_dict, {}, {restore_op_name}, nullptr);
    if (status != tensorflow::Status::OK()) return status;
  } else {
    // virtual Status Run(const std::vector<std::pair<string, Tensor> >& inputs,
    //                  const std::vector<string>& output_tensor_names,
    //                  const std::vector<string>& target_node_names,
    //                  std::vector<Tensor>* outputs) = 0;
    status = sess->Run({}, {}, {"init"}, nullptr);
    if (status != tensorflow::Status::OK()) return status;
  }*/

  return tensorflow::Status::OK();
}

void inference_main(int argc) {
  const std::string graph_fn = "./exported/my_model.meta";
  const std::string checkpoint_fn = "./exported/my_model";

  // prepare session
  tensorflow::Session *sess;
  tensorflow::SessionOptions options;
  TF_CHECK_OK(tensorflow::NewSession(options, &sess));
  TF_CHECK_OK(LoadModel(sess, graph_fn, checkpoint_fn));

  // prepare inputs
  tensorflow::TensorShape data_shape({1, 7});
  tensorflow::Tensor data(tensorflow::DT_FLOAT, data_shape);

  // same as in python file
  auto data_ = data.flat<float>().data();
  for (int i = 0; i < 7; ++i) data_[i] = 1;

  tensor_dict feed_dict = {
      {"input", data},
  };

  std::vector<tensorflow::Tensor> outputs;
  TF_CHECK_OK(sess->Run(feed_dict, {"output", "dense/kernel:0", "dense/bias:0"},
                        {}, &outputs));

  std::cout << "input           " << data.DebugString() << std::endl;
  std::cout << "output          " << outputs[0].DebugString() << std::endl;
  std::cout << "dense/kernel:0  " << outputs[1].DebugString() << std::endl;
  std::cout << "dense/bias:0    " << outputs[2].DebugString() << std::endl;

  return ;
}

tensorflow::Session* start_session(int argc)
{
  //const std::string graph_fn = "./exported/my_model.meta";
  //const std::string checkpoint_fn = "./exported/my_model";
  const std::string graph_fn = "./transition/decoder_transition_model.pb";
  const std::string checkpoint_fn = "";

  // prepare session
  tensorflow::Session *sess;
  tensorflow::SessionOptions options;
  TF_CHECK_OK(tensorflow::NewSession(options, &sess));
  TF_CHECK_OK(LoadModel(sess, graph_fn, checkpoint_fn));

  return sess;
}
double inference_keras_main(tensorflow::Session* sess){
  // prepare inputs
	int batch_size = 1;
  tensorflow::TensorShape data_shape({batch_size, 9});
  tensorflow::TensorShape data_shape1({batch_size, 2});
  tensorflow::Tensor data(tensorflow::DT_FLOAT, data_shape);
  tensorflow::Tensor data1(tensorflow::DT_FLOAT, data_shape1);
  // same as in python file
  auto data_ = data.flat<float>().data();
  auto data1_ = data1.flat<float>().data();
  //auto data_ = data.flat<float>().data();
  //for (int i = 0; i < 7; ++i) data_[i] = 1;
  //data_[0] = 42;
  //data_[1] = 43;
  //0.33,0.08,0.47,0.08,0.01, 1.1,1.5,0,0
  for (int i = 0; i < batch_size; ++i){
  data_[9*i + 0] = 0.33;
  data_[9*i + 1] = 0.08;
  data_[9*i + 2] = 0.47;
  data_[9*i + 3] = 0.08;
  data_[9*i + 4] = 0.01;
  data_[9*i + 5] = 1.1;
  data_[9*i + 6] = 1.5;
  data_[9*i + 7] = 0;
  data_[9*i + 8] = 0;
  data1_[2*i + 0] = 0.9;
  data1_[2*i + 1] = 0.5;
}

	double start_t = despot::get_time_second();
  //tensor_dict feed_dict = {
  //    {"Intermediate_input", data},
//  };
  tensor_dict feed_dict = {
      {"input_state:0", data},{"z_sampling:0", data1}
  };

  std::vector<tensorflow::Tensor> outputs;
  //TF_CHECK_OK(
  //    sess->Run(feed_dict, {"Output/BiasAdd"}, {}, &outputs));
  TF_CHECK_OK(
      sess->Run(feed_dict, {"dense_2/BiasAdd"}, {}, &outputs));
//dense_2/BiasAdd:0
  std::cout << "input           " << data.DebugString() << std::endl;
  std::cout << "output          " << outputs[0].DebugString() << std::endl;

  return start_t;
}
#endif /* INFERENCECC_H_ */
