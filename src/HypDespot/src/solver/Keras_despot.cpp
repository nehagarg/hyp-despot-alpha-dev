#include <despot/solver/despot.h>
//#include <tensorflow/core/protobuf/meta_graph.pb.h>
//#include <tensorflow/core/public/session.h>
//#include <tensorflow/core/public/session_options.h>
#include "tensorflow/core/framework/tensor.h"
namespace despot {
/*void DESPOT::Keras_InitRootBounds(VNode* vnode,const DSPOMDP* model, RandomStreams& streams,
		History& history, tensorflow::Session *sess_lower_upper_bound) {
	streams.position(vnode->depth());
	int batch_size = vnode->particleIDs().size();
	int input_vector_size = model->KerasInputVectorSize();
	int latent_dimension_size = model->LatentDimensionSize();
	tensorflow::TensorShape data_shape({batch_size, input_vector_size});
	tensorflow::TensorShape data_shape1({batch_size,latent_dimension_size});
	tensorflow::Tensor data(tensorflow::DT_FLOAT, data_shape);
	tensorflow::Tensor data1(tensorflow::DT_FLOAT, data_shape1);
	// same as in python file
	auto data_ = data.flat<float>().data();
	auto data1_ = data1.flat<float>().data();
	std::copy_n(vnode->particle_keras_batch().begin(), batch_size * input_vector_size, data_);

	std::vector<float> all_particle_streams = ((KerasRandomStreams&)streams).KerasALLParticlesEntry();
	std::vector<int> particle_ids = vnode->particleIDs();
	//Copy random variables
	for(int i = 0; i < batch_size; i++)
	{
		std::copy_n(all_particle_streams[particle_ids[i]*latent_dimension_size], latent_dimension_size, data1_+(sizeof(float)*i*latent_dimension_size));
	}



}
*/


void DESPOT::Keras_Expand_Action(VNode* vnode, ScenarioLowerBound* lb,
		ScenarioUpperBound* ub, const DSPOMDP* model, RandomStreams& streams,
		History& history) {

	//int ThreadID = 0;
	//if (Globals::config.use_multi_thread_)
	//	ThreadID = Globals::MapThread(this_thread::get_id());
	int NumActions = model->NumActions();
	int NumObs = model->NumObservations();
	int NumScenarios = Globals::config.num_scenarios;

	//Globals::Global_print_expand(this_thread::get_id(), vnode, vnode->depth(), vnode->edge());

	//if(Globals::config.use_multi_thread_)
	//	static_cast<Shared_VNode*>(vnode)->is_waiting_=true;

	//HitCount++;
	//Globals::AddExpanded();
	auto start_total = Time::now();

	/*Update streams, history, and particles into GPU*/
	//PrepareGPUDataForNode(vnode, model, ThreadID, streams);
	streams.position(vnode->depth());

	int NumParticles = vnode->particles().size();
	/*std::cout << "Num particles : " << NumParticles << "," << vnode->num_GPU_particles_ << std::endl;
	std::cout << "Particles ids " ;
	for(int i = 0; i < vnode->particleIDs().size(); i++)
	{
		std::cout << vnode->particleIDs()[i] << " ";
	}
	std::cout << std::endl;
	*/



	/*Run Monte Carlo simulations in GPU: update particles and perform rollouts*/
	//MCSimulation(vnode, ThreadID,model, streams,history,true);

	//Step the particles for all actions
	//std::vector<bool> terminal_vector;
	//std::vector<float> reward_vector;
	const std::vector<State*>& particles=vnode->particles();
	std::vector<float> random_number_vector;
	std::vector<float> uniform_random_number_vector;

	((KerasRandomStreams&)streams).KerasParticlesEntry(particles,uniform_random_number_vector, true);

	((KerasRandomStreams&)streams).KerasParticlesEntry(particles,random_number_vector);

	const std::vector<float>& keras_particle_batch = vnode->particle_keras_batch();

	std::vector<std::vector<float>> generated_obs;


	if(Globals::config.track_alpha_vector)
	{
		/*Expand common QNode*/

		std::vector<std::vector<int>> obs_ids_all_a;
		std::vector<std::map<OBS_TYPE, std::vector<int> >> partitions_all_a;
		int num_particles_all_a = 0;
		logd << "Num particles " << NumParticles << "[";
		/*for (int i = 0; i < NumParticles; i++) {
		    logd << particleIDs[i] << "," ;
		}*/
		logd << std::endl;
		for (int action = 0; action < NumActions; action++) {

			logd << "GPU Expand Action " << action << std::endl;
			QNode* qnode;
			if(action >= vnode->children().size())
			{
				//Create new Qnode for action
				//if (Globals::config.use_multi_thread_)
				//	qnode = new Shared_QNode(static_cast<Shared_VNode*>(vnode), action);
				//else
					qnode = new QNode(vnode, action);

				vnode->children().push_back(qnode);
			}
			else
			{
			qnode = vnode->Child(action);
			}
			//if(Globals::config.use_multi_thread_ && Globals::config.exploration_mode==UCT)
			//			static_cast<Shared_QNode*>(qnode)->visit_count_=1.1;
			vnode->common_parent_->common_children_.push_back(qnode);
			QNode* common_qnode = qnode;
			common_qnode->populating_node = qnode;
			common_qnode->step_reward_vector.resize(Globals::config.num_scenarios,0);
			if(Globals::config.use_sawtooth_upper_bound)
			{
				common_qnode->vnode_upper_bound_per_particle.resize(Globals::config.num_scenarios, 0);
			}

			std::vector<tensorflow::Tensor> outputs;
			if(action == 10) //Hack for grasping
			{
				model->StepKerasParticles(keras_particle_batch, action, uniform_random_number_vector, outputs);
			}
			else
			{
				model->StepKerasParticles(keras_particle_batch, action, random_number_vector, outputs);
			}

			double step_reward = 0;
			std::map<OBS_TYPE, std::vector<int> > partitions;
			partitions_all_a.push_back(partitions);
			int num_begin = 0;
			int num_end = 0;
			auto terminal_vector = outputs[1].flat<float>().data();
			auto reward_vector = outputs[2].flat<float>().data();
			auto stepped_particle_batch = outputs[0].flat<float>().data();
			auto obs_vector = outputs[3].flat<float>().data();
			std::vector<float> generated_obs_per_action;


			for (int i = 0; i < NumParticles; i++) {

				int parent_PID = particles[i]->scenario_id; // parent_PID corresponds to scenario id
				logd << "Steppig particle " << parent_PID << std::endl;
				common_qnode->step_reward_vector[parent_PID] = Globals::Discount(vnode->depth()) * reward_vector[i];
				step_reward +=  reward_vector[i]* vnode->particle_weights[parent_PID];
				OBS_TYPE obs = parent_PID;

				if ((int)terminal_vector[i] == 0 ) {
					num_particles_all_a++;
					if(parent_PID == (num_end))
							{
								num_end++;
							}
					else
					{
						common_qnode->particle_keras_batch.insert(common_qnode->particle_keras_batch.end(),
								stepped_particle_batch + num_begin*model->KerasInputVectorSize(),
								stepped_particle_batch + num_end*model->KerasInputVectorSize());
						generated_obs_per_action.insert(generated_obs_per_action.end(),
								obs_vector + num_begin*model->KerasObservationVectorSize(),
								obs_vector + num_end*model->KerasObservationVectorSize());
						num_begin = parent_PID;
						num_end = num_begin +1;
					}
					common_qnode->particleIDs_.push_back(parent_PID);
					common_qnode->particles_.push_back(particles[i]); //Required to keep size of particles_ consistent with particleIds
					if(partitions_all_a[action].size() < Globals::config.num_obs)
					{
						partitions_all_a[action][obs].push_back(parent_PID);
					}
				}



			} //Loop over NumParticles
			if(num_end > num_begin)
			{
				common_qnode->particle_keras_batch.insert(common_qnode->particle_keras_batch.end(),
												stepped_particle_batch + num_begin*model->KerasInputVectorSize(),
												stepped_particle_batch + num_end*model->KerasInputVectorSize());
				generated_obs_per_action.insert(generated_obs_per_action.end(),
												obs_vector + num_begin*model->KerasObservationVectorSize(),
												obs_vector + num_end*model->KerasObservationVectorSize());
			}
			step_reward = Globals::Discount(vnode->depth()) * step_reward
					- Globals::config.pruning_constant;//pruning_constant is used for regularization

			qnode->step_reward = step_reward;
			//common_qnode->num_GPU_particles_ = common_qnode->particleIDs_.size();

			logd << "Particles survived " << common_qnode->particleIDs_.size()  << std::endl;
			generated_obs.push_back(generated_obs_per_action);
		} // First Loop close over actions


		if(num_particles_all_a > 0)
			{


			//DESPOT::PrepareGPUDataForCommonQNode(vnode->common_parent_, model, ThreadID, streams, particleIds_all_a, num_particles_all_a);
			//GPU_Cal_Obs_Prob(vnode, ThreadID, model);

		//TODO call obs prob computation and init bound computation
		for (int action = 0; action < NumActions; action++) {
			QNode* qnode = vnode->Child(action);
			QNode* common_qnode = qnode;
			std::map<OBS_TYPE, VNode*>& children = qnode->children();
			VNode* residual_vnode;
			if(common_qnode->particleIDs_.size() > 0)
			{


				/*if (Globals::config.use_multi_thread_)
						{
							residual_vnode = new Shared_VNode(vnode->depth() + 1,
													 static_cast<Shared_QNode*>(qnode), static_cast<Shared_QNode*>(common_qnode),
													 Globals::RESIDUAL_OBS);
							if (Globals::config.exploration_mode == UCT)
								static_cast<Shared_VNode*>(residual_vnode)->visit_count_ = 1.1;
						}
						else*/
						{
							residual_vnode = new VNode(vnode->depth() + 1,
									qnode,common_qnode, Globals::RESIDUAL_OBS);
						}
					children[Globals::RESIDUAL_OBS] = residual_vnode;
				//residual_vnode->observation_particle_size = 1; //Not used anywhere probably
				residual_vnode->extra_node = true;
				residual_vnode->obs_probs_holder = residual_vnode;
				residual_vnode->obs_probs.resize(Globals::config.num_scenarios, 0);
				residual_vnode->num_GPU_particles_ = common_qnode->particleIDs_.size();

			}
			//Create child nodes

	        //std::map iterator is ordered. So obs order would be the same
		for (std::map<OBS_TYPE, std::vector<int> >::iterator it = partitions_all_a[action].begin();
			it != partitions_all_a[action].end(); it++) {
			OBS_TYPE obs = it->first;
	                //int observation_particle_size_ = partitions[obs].size();
			VNode* child_vnode;
			/*if (Globals::config.use_multi_thread_)
			        			{
			        				child_vnode = new Shared_VNode(vnode->depth() + 1,
			        				                         static_cast<Shared_QNode*>(qnode), static_cast<Shared_QNode*>(common_qnode),
			        				                         obs);
			        				if (Globals::config.exploration_mode == UCT)
			        					static_cast<Shared_VNode*>(child_vnode)->visit_count_ = 1.1;
			        			}
			        			else*/
			        			{
			        				child_vnode = new VNode(vnode->depth() + 1,
			        							qnode, common_qnode, obs);
			        			}
	                //vnode->observation_particle_size = observation_particle_size_;
	                child_vnode->obs_probs.resize(Globals::config.num_scenarios, 0);
			logd << " New node created!" << std::endl;
			children[obs] = child_vnode;
	                child_vnode->obs_probs_holder = child_vnode;
	                child_vnode->num_GPU_particles_ = common_qnode->particleIDs_.size();
	                if(obs == Globals::RESIDUAL_OBS)
	                {
	                    child_vnode->extra_node = true;
	                }

		}
		} // Third loop over actions

		//ReadBackData(ThreadID, Obs_Prob_offset, Obs_ProbSize);

		for (int action = 0; action < NumActions; action++) {
			QNode* qnode = vnode->Child(action);
			QNode* common_qnode = qnode;
			std::map<OBS_TYPE, VNode*>& children = qnode->children();
			VNode* residual_vnode;

			if(common_qnode->particleIDs_.size() > 0)
			{
				residual_vnode = children[Globals::RESIDUAL_OBS];
				//Prepare data for computing observation probabilities
				std::vector<float> all_particle_batch = common_qnode->particle_keras_batch;
				//std::vector<float> all_particle_batch = generated_obs[action];
				//std::cout << "Generated_obs size " << all_particle_batch.size() << std::endl;
				std::vector<float> obs_batch;
				std::vector<float> random_number_vector; //not needed now as we pass observation
				//const std::vector<float>& all_random_number_vector =
					//((KerasRandomStreams&)streams).KerasALLParticlesEntry();

				//Make #obs*#particles size vector
				while(2*all_particle_batch.size() <= partitions_all_a[action].size()*common_qnode->particle_keras_batch.size())
				{
					auto end_iterator = all_particle_batch.end();
					all_particle_batch.insert(all_particle_batch.end(),all_particle_batch.begin(),end_iterator);
				}
				if(all_particle_batch.size() < partitions_all_a[action].size()*common_qnode->particle_keras_batch.size())
				{
					int all_particle_batch_size = all_particle_batch.size();
					int target_size = partitions_all_a[action].size()*common_qnode->particle_keras_batch.size();
					all_particle_batch.insert(all_particle_batch.end(),all_particle_batch.begin(),
							all_particle_batch.begin() + target_size - all_particle_batch_size);
				}

				for(int i = 0; i < partitions_all_a[action].size(); i++)
				{
					while(2*(obs_batch.size() - i*(common_qnode->particle_keras_batch.size())*model->KerasObservationVectorSize()/model->KerasInputVectorSize()) <=
							common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize())
					{
						if((obs_batch.size() -
								i*(common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize())) == 0)
								{
									obs_batch.insert(obs_batch.end(),
											generated_obs[action].begin()+i*model->KerasObservationVectorSize(),
											generated_obs[action].begin()+(i+1)*model->KerasObservationVectorSize());
									//random_number_vector.insert(random_number_vector.end(),
											//all_random_number_vector.begin() + (common_qnode->particleIDs_[i])*model->LatentDimensionSize(),
											//all_random_number_vector.begin() + (common_qnode->particleIDs_[i]+1)*model->LatentDimensionSize());
								}
						else
						{
							auto end_iterator = obs_batch.end();

							obs_batch.insert(obs_batch.end(),
									obs_batch.begin() + i*(common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize()),
											end_iterator);
							//auto random_num_end_iterator = random_number_vector.end();
							//random_number_vector.insert(random_number_vector.end(),
							//		random_number_vector.begin() + i*common_qnode->particle_keras_batch.size()*model->LatentDimensionSize()/model->KerasInputVectorSize(),
							//		random_num_end_iterator);

						}
						if(obs_batch.size() < (i+1)*common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize())
						{
							int diff = (i+1)*common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize()
									- obs_batch.size();
							obs_batch.insert(obs_batch.end(),
											obs_batch.begin() +
											i*(common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize()),
											obs_batch.begin() +
											i*(common_qnode->particle_keras_batch.size()*model->KerasObservationVectorSize()/model->KerasInputVectorSize()) + diff);
							//random_number_vector.insert(random_number_vector.end(),
							//		random_number_vector.begin()+ i*common_qnode->particle_keras_batch.size()*model->LatentDimensionSize()/model->KerasInputVectorSize(),
							//		random_number_vector.begin()+ (i*common_qnode->particle_keras_batch.size() + diff)*model->LatentDimensionSize()/model->KerasInputVectorSize());
						}
					}

				}
				std::vector<tensorflow::Tensor> obs_tensorflow_outputs;
				model->GetObservationProbability(obs_batch, all_particle_batch, action,random_number_vector,
						obs_tensorflow_outputs);
				auto obs_outputs = obs_tensorflow_outputs[0].flat<float>().data();
			//}

			 double max_prob_sum = 0.0;
			 int obs_id = 0;
			for (std::map<OBS_TYPE, std::vector<int> >::iterator it = partitions_all_a[action].begin();
						it != partitions_all_a[action].end(); it++) {
				OBS_TYPE obs = it->first;
				VNode* child_vnode = qnode->Child(obs);
			double total_weight = 0;
	                for(int i = 0; i < common_qnode->particleIDs_.size();i++)
	                {
	                	int scenario_id = common_qnode->particleIDs_[i];
	                   // double prob = Hst_obs_prob_all_a_p_obs[ThreadID][action * NumScenarios*Globals::config.num_obs + (obs_id*NumScenarios) + scenario_id];
	                	double prob = obs_outputs[ (obs_id*common_qnode->particleIDs_.size()) + i];

	                	//int scenario_id = common_qnode->particles_[i]->scenario_id;
	                    //prob = model->ObsProb(obs, *common_qnode->particles_[i], qnode->edge());


	                   logd << "Obs Prob: for obs " <<  obs << " " << prob << " ";

			 // Terminal state is not required to be explicitly represented and may not have any observation
				child_vnode->particle_weights[common_qnode->particleIDs_[i]] = vnode->particle_weights[common_qnode->particleIDs_[i]]* prob;
				total_weight += child_vnode->particle_weights[common_qnode->particleIDs_[i]];
	                        //Total weight should not be zero as one particle actually produced that observation
	                        child_vnode->obs_probs[common_qnode->particleIDs_[i]] = prob;

	                        residual_vnode->obs_probs[common_qnode->particleIDs_[i]] =  residual_vnode->obs_probs[common_qnode->particleIDs_[i]]+ prob;
	                        if(residual_vnode->obs_probs[common_qnode->particleIDs_[i]] > max_prob_sum)
	                        {
	                            max_prob_sum = residual_vnode->obs_probs[common_qnode->particleIDs_[i]];
	                        }


	                }
	                obs_id++;
	                child_vnode->prob_o_given_b = total_weight;
	                //std::cout << "Max prob sum " << max_prob_sum << std::endl;
	                for(int i = 0; i < common_qnode->particleIDs_.size(); i++)
	                {
	                    if(total_weight > 0) //total weight might be zero if particle weight is zero
	                    {
	                    child_vnode->particle_weights[common_qnode->particleIDs_[i]] = child_vnode->particle_weights[common_qnode->particleIDs_[i]]/total_weight;
	                    }


	                }

	                logd << " Creating node for obs " << obs << std::endl;


	        //Update upper bound lower bound
			history.Add(qnode->edge(), obs);

			DESPOT::InitBounds(child_vnode, lb, ub, streams, history);
			history.RemoveLast();
	        //Init bounds using data from GPU




			logd << " New node's bounds: (" << child_vnode->lower_bound() << child_vnode->lower_bound_alpha_vector<< ", "
				<< child_vnode->upper_bound() << child_vnode->common_parent()->default_upper_bound_alpha_vector << ")" << std::endl;
	                //lower_bound += vnode->lower_bound();
			//upper_bound += vnode->upper_bound();
			//lower_bound += vnode->lower_bound()*observation_particle_size_/observation_particle_size;
			//upper_bound += vnode->upper_bound()*observation_particle_size_/observation_particle_size;
	        }


		//Scale probs
			for (std::map<OBS_TYPE, VNode*>::iterator it = children.begin();
				it != children.end(); it++) {
				VNode* child_vnode = it->second;
		                if(!child_vnode->extra_node)
		                {
		            for(int i = 0; i < common_qnode->particleIDs_.size();i++)
		            {
		                child_vnode->obs_probs[common_qnode->particleIDs_[i]] = child_vnode->obs_probs[common_qnode->particleIDs_[i]]/max_prob_sum;
		            }
		            child_vnode->prob_o_given_b = child_vnode->prob_o_given_b/max_prob_sum;
		            }
		        }
		//}
	        //Residual node
	     //   if(common_qnode->particleIDs_.size() > 0)
	      //  {
	            double total_weight = 0;
	                for(int i = 0; i < common_qnode->particleIDs_.size();i++)
	                {
	                    double prob = 1 - (residual_vnode->obs_probs[common_qnode->particleIDs_[i]]/max_prob_sum);



	                    //std::cout << "Obs Prob: res" <<  prob << " ";

			 // Terminal state is not required to be explicitly represented and may not have any observation
				residual_vnode->particle_weights[common_qnode->particleIDs_[i]] = vnode->particle_weights[common_qnode->particleIDs_[i]]* prob;
				total_weight += residual_vnode->particle_weights[common_qnode->particleIDs_[i]];

	                        residual_vnode->obs_probs[common_qnode->particleIDs_[i]] = prob;




	                }
	                residual_vnode->prob_o_given_b = total_weight;
	                for(int i = 0; i < common_qnode->particleIDs_.size(); i++)
	                {
	                    if(total_weight > 0) //total weight might be zero for residual node
	                    {
	                    residual_vnode->particle_weights[common_qnode->particleIDs_[i]] = residual_vnode->particle_weights[common_qnode->particleIDs_[i]]/total_weight;
	                    }

	                }

	                logd << " Creating node for obs " << Globals::RESIDUAL_OBS << std::endl;


			history.Add(qnode->edge(), Globals::RESIDUAL_OBS);
			//common_qnode lower bound upper bound already updated at vnodes. So not updated here
			DESPOT::InitBounds(residual_vnode, lb, ub, streams, history);
			history.RemoveLast();

			logd << " New node's bounds: (" << residual_vnode->lower_bound() << residual_vnode->lower_bound_alpha_vector<< ", "
				<< residual_vnode->upper_bound() << residual_vnode->common_parent()->default_upper_bound_alpha_vector << ")" << std::endl;
	                //lower_bound += vnode->lower_bound();

	        }


		}//Fourth Loop over actions
			}

	}
	else
	{

/*
	//Expand v-node
	for (int action = 0; action < NumActions; action++) {
		//Partition particles by observation

		std::map<OBS_TYPE, std::vector<State*> > partitions;
		std::map<OBS_TYPE, std::vector<int> > partitions_ID;
		for (int i = 0; i < NumParticles; i++) {
			int parent_PID = particleIDs[i];
			OBS_TYPE obs = parent_PID;

			if (Hst_term_all_a_and_p[ThreadID][action * NumScenarios + parent_PID] == false) {
				partitions[obs].push_back(NULL);
				partitions_ID[obs].push_back(i);
			}
		}




		QNode* qnode = vnode->Child(action);

		//if(Globals::config.use_multi_thread_ && Globals::config.exploration_mode==UCT)
		//	static_cast<Shared_QNode*>(qnode)->visit_count_=1.1;

		if (partitions.size() == 0 && false) {
			cout<<"[Qnode] depth="<<vnode->depth()+1<<" obs="<< vnode->edge()<<" qnode "<<action<<" all particle termination: reward="<<Hst_r_all_a[action];
			cout<<" parent lb:"<<qnode->parent()->lower_bound()<<endl;
		} else {
		}

		double lower_bound = 0, upper_bound = 0;
		Hst_r_all_a[ThreadID][action] = Globals::Discount(vnode->depth())
				* Hst_r_all_a[ThreadID][action]
				- Globals::config.pruning_constant; //pruning_constant is used for regularization
		lower_bound = (Hst_r_all_a[ThreadID][action]);
		upper_bound = (Hst_r_all_a[ThreadID][action]);

		bool DoPrint= DESPOT::Print_nodes;
		if (FIX_SCENARIO == 1 && DoPrint) {
			cout.precision(10);
			if(action==0) cout<<endl;
			cout << "step reward (d= " << vnode->depth() + 1 << " ): "
					<< Hst_r_all_a[ThreadID][action] / (1.0f/Globals::config.num_scenarios * NumParticles)
					<< endl;
		}


		std::map<OBS_TYPE, VNode*>& children = qnode->children();
		for (std::map<OBS_TYPE, std::vector<State*> >::iterator it =
				partitions.begin(); it != partitions.end(); it++) {
			OBS_TYPE obs = it->first;
			logd << " Creating node for obs " << obs << endl;

			VNode* child_vnode;


				child_vnode = new VNode(partitions[obs], partitions_ID[obs],
						vnode->depth() + 1, qnode, obs);


			//Create GPU particles for the new v-node
			child_vnode->weight_=partitions[obs].size()/((float)NumScenarios);

			logd << " New node created!" << endl;
			children[obs] = child_vnode;

			//Calculate initial bounds
			double vnode_lower_bound = 0;
			double vnode_upper_bound = 0;
			double vnode_utility_upper = 0;

			for (int i = 0; i < child_vnode->particleIDs().size(); i++) {
				int parent_PID = child_vnode->particleIDs()[i];

				vnode_lower_bound += Hst_lb_all_a_p[ThreadID][action
						* NumScenarios + parent_PID].value;
				vnode_upper_bound += Hst_ub_all_a_p[ThreadID][action
						* NumScenarios + parent_PID];
				vnode_utility_upper += Hst_uub_all_a_p[ThreadID][action
						* NumScenarios + parent_PID];
			}

			child_vnode->lower_bound(vnode_lower_bound);
			child_vnode->upper_bound(vnode_upper_bound-Globals::config.pruning_constant);
			child_vnode->utility_upper_bound(vnode_utility_upper);
			int first_particle = action * NumScenarios
					+ child_vnode->particleIDs()[0];
			child_vnode->default_move(
					ValuedAction(
							Hst_lb_all_a_p[ThreadID][first_particle].action,
							vnode_lower_bound));
			logd << " New node's bounds: (" << child_vnode->lower_bound()
					<< ", " << child_vnode->upper_bound() << ")" << endl;

			if (child_vnode->upper_bound() < child_vnode->lower_bound()
			// close gap because no more search can be done on leaf node
					|| child_vnode->depth() == Globals::config.search_depth - 1) {
				child_vnode->upper_bound(child_vnode->lower_bound());
			}



			if (FIX_SCENARIO == 1 || DoPrint) {
				cout.precision(10);
				cout << " [GPU Vnode] New node's bounds: (d= "
						<< child_vnode->depth() << " ,obs=" << obs << " , lb= "
						<< child_vnode->lower_bound() / child_vnode->weight_
						<< " ,ub= "
						<< child_vnode->upper_bound() / child_vnode->weight_
						<< " ,uub= "
						<< child_vnode->utility_upper_bound()
								/ child_vnode->weight_ << " ,weight= "
						<< child_vnode->weight_ << " )";
				if(child_vnode->Weight()==1.0/Globals::config.num_scenarios) cout<<", particle_id="<< child_vnode->particles()[0]->scenario_id;
					cout<<", WEU="<<WEU(child_vnode);
				cout  << endl;
			}

			lower_bound += child_vnode->lower_bound();
			upper_bound += child_vnode->upper_bound();

		}


		qnode->step_reward = Hst_r_all_a[ThreadID][action];

		qnode->lower_bound(lower_bound);
		qnode->upper_bound(upper_bound);
		qnode->utility_upper_bound(
				upper_bound + Globals::config.pruning_constant);
		qnode->default_value = lower_bound;

		qnode->Weight();
		if (FIX_SCENARIO == 1 || DoPrint) {
			cout.precision(10);
			cout << " [GPU Qnode] New qnode's bounds: (d= " << vnode->depth() + 1
					<< " ,action=" << action << ", lb= "
					<< qnode->lower_bound() / qnode->Weight() << " ,ub= "
					<< qnode->upper_bound() / qnode->Weight() << " ,uub= "
					<< qnode->utility_upper_bound() / qnode->Weight()
					<< " ,weight= " << qnode->Weight() << " )" << endl;
		}


	}*/
	}

	//if(Globals::config.use_multi_thread_)
	//	static_cast<Shared_VNode*>(vnode)->is_waiting_=false;

	//double oldValue=TotalExpansionTime.load();
	//TotalExpansionTime.compare_exchange_weak(oldValue,oldValue+ Globals::ElapsedTime(start_total));
}



}
