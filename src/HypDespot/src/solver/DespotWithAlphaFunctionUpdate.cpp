#include <despot/solver/DespotWithAlphaFunctionUpdate.h>
#include <despot/util/logging.h>
#include <map>

namespace despot {
    
bool DespotWithAlphaFunctionUpdate::PedPomdpProb = false;
    DespotWithAlphaFunctionUpdate::DespotWithAlphaFunctionUpdate(const DSPOMDP* model, 
        ScenarioLowerBound* lb, 
        ScenarioUpperBound* ub, 
        Belief* belief): DESPOT(model, lb, ub, belief)
        {
            std::cout << "Despot With alphafunction update" << std::endl;
            //o_helper_ = new DespotStaticFunctionOverrideHelperForAlphaFunctionUpdate();
            //o_helper_->solver_pointer = this;
        }
    
    
    void DespotWithAlphaFunctionUpdate::Expand(QNode* qnode, ScenarioLowerBound* lb,
		ScenarioUpperBound* ub, const DSPOMDP* model,
		RandomStreams& streams, History& history)
//    void DespotStaticFunctionOverrideHelperForAlphaFunctionUpdate::Expand(QNode* qnode, ScenarioLowerBound* lb,
//	ScenarioUpperBound* ub, const DSPOMDP* model,
//	RandomStreams& streams,
//	History& history,  ScenarioLowerBound* learned_lower_bound, 
//        SearchStatistics* statistics, 
//        DespotStaticFunctionOverrideHelper* o_helper)
    {
        logd << "Expanding in Despot With Alpha function update" << std::endl;
        VNode* parent = qnode->parent();
        if((Globals::config.useGPU && parent->PassGPUThreshold())|| Globals::config.use_keras_model)
		{
        	/*if(Globals::config.use_multi_thread_)
        	{
        		(static_cast<Shared_QNode*>(parent->common_parent_))->lock();
        	}*/
			if(parent->common_parent_->common_children_.size()==0)
			{
				if(Globals::config.useGPU && parent->PassGPUThreshold())
				{
					//DESPOT::GPU_Expand_Action(parent,lb,ub,model,streams,history);
				}
				if(Globals::config.use_keras_model)
				{
					DESPOT::Keras_Expand_Action(parent, lb, ub, model, streams,
						                  history);
				}
			}
			/*if(Globals::config.use_multi_thread_)
			{
				(static_cast<Shared_QNode*>(parent->common_parent_))->unlock();
			}*/

		}

        QNode* common_qnode;
        /*if(Globals::config.use_multi_thread_)
        {



		//std::cout << "Before locking Mutex for action " << qnode->edge() << std::endl;

			common_qnode = (static_cast<Shared_VNode*>(parent))->CommonChild(qnode->edge());
			(static_cast<Shared_QNode*>(common_qnode))->lock();

        }
        else*/
        {
        		common_qnode = parent->CommonChild(qnode->edge());

        }
        QNode* populated_qnode = NULL;
        
	streams.position(parent->depth());
	std::map<OBS_TYPE, VNode*>& children = qnode->children();

	const std::vector<State*>& particles = parent->particles();
        //int observation_particle_size = parent->observation_particle_size;


	double step_reward = 0;

        //std::vector<OBS_TYPE> observations;
	// Partition particles by observation
	std::map<OBS_TYPE, std::vector<int> > partitions;
	std::map<OBS_TYPE, std::vector<int> > obs_vectors;
        //std::map<OBS_TYPE, std::vector<State*> > partitions_belief_; //Stores belief particles
        
	OBS_TYPE obs;
	double reward;
        
        if(common_qnode->step_reward_vector.size() == 0)
        {
            common_qnode->step_reward_vector.resize(Globals::config.num_scenarios,0);
            if(Globals::config.use_sawtooth_upper_bound)
            {
                common_qnode->vnode_upper_bound_per_particle.resize(Globals::config.num_scenarios, 0);
            }
            common_qnode->populating_node = qnode;
            double node_factor = 1.0; //observation_particle_size/Globals::config.num_scenarios; 
            

            int  num_particles_pushed = 0;
            for (int i = 0; i < particles.size(); i++) {
                State* particle = particles[i];
                logd << " Original: " << *particle << std::endl;

                State* copy = model->Copy(particle);

                logd << " Before step: " << *copy << std::endl;

                bool terminal = model->Step(*copy, streams.Entry(copy->scenario_id),
                        qnode->edge(), reward, obs);
                common_qnode->step_reward_vector[particle->scenario_id] = Globals::Discount(parent->depth())*reward *node_factor;
                //qnode->lower_bound_alpha_vector[particle->scenario_id] = qnode->step_reward_vector[particle->scenario_id];
                //qnode->upper_bound_alpha_vector[particle->scenario_id] = qnode->step_reward_vector[particle->scenario_id];
                step_reward += reward * parent->particle_weights[particle->scenario_id];

                logd << " After step: " << *copy << " " << (reward * parent->particle_weights[particle->scenario_id])
                        << " " << reward << " " << parent->particle_weights[particle->scenario_id] << std::endl;

                if (!terminal) {

                    common_qnode->particles_.push_back(copy);
                    num_particles_pushed++;
                    if(partitions.size() < Globals::config.num_obs)
                    {
                    	partitions[obs].push_back(particle->scenario_id);
                    	if(DespotWithAlphaFunctionUpdate::PedPomdpProb)
							{
								obs_vectors[obs] = model->ObserveVector(*copy);
							}
                    }
				}

                else {
                        model->Free(copy);
                }

               
            }
            step_reward = Globals::Discount(parent->depth()) * step_reward*node_factor
		- Globals::config.pruning_constant;//pruning_constant is used for regularization
        
            qnode->step_reward = step_reward;
        
        
        /*std::vector<double> residual_obs_prob;
        if(qnode->particles_.size() > 0)
        {
            residual_obs_prob.resize(Globals::config.num_scenarios, 0);
            partitions[Globals::RESIDUAL_OBS].push_back(0);
        }*/
        VNode* residual_vnode;
        if(common_qnode->particles_.size() > 0)
        {
        	/*if (Globals::config.use_multi_thread_)
        			{
        				residual_vnode = new Shared_VNode(parent->depth() + 1,
        				                         static_cast<Shared_QNode*>(qnode), static_cast<Shared_QNode*>(common_qnode),
        				                         Globals::RESIDUAL_OBS);
        				if (Globals::config.exploration_mode == UCT)
        					static_cast<Shared_VNode*>(residual_vnode)->visit_count_ = 1.1;
        			}
        			else*/
        			{
        				residual_vnode = new VNode(parent->depth() + 1,
        						qnode,common_qnode, Globals::RESIDUAL_OBS);
        			}
                children[Globals::RESIDUAL_OBS] = residual_vnode;
            //residual_vnode->observation_particle_size = 1; //Not used anywhere probably
            residual_vnode->extra_node = true;
            residual_vnode->obs_probs_holder = residual_vnode;
            residual_vnode->obs_probs.resize(Globals::config.num_scenarios, 0);
            
            
        }
        
	
        //std::cout << "Step reward = " << step_reward << std::endl;
	//double lower_bound = step_reward;
	//double upper_bound = step_reward;

        //std::vector<double> residual_obs_prob;
        //residual_obs_prob.resize(Globals::config.num_scenarios, 1);
        //
	// Create new belief nodes
        double max_prob_sum = 0.0;
	for (std::map<OBS_TYPE, std::vector<int> >::iterator it = partitions.begin();
		it != partitions.end(); it++) {
		OBS_TYPE obs = it->first;
		std::vector<int> obs_vec;
		if(DespotWithAlphaFunctionUpdate::PedPomdpProb)
			{
				obs_vec = obs_vectors[obs] ;
				logd << "Creating node for obs " << obs << " " ;
				/*for(int j = 0; j < obs_vec.size();j++)
				  {
				    std::cout << obs_vec[j] << " " ;
				  }
				std::cout << std::endl;
				*/
			}


                //int observation_particle_size_ = partitions[obs].size();
		VNode* vnode;
		if (Globals::config.use_multi_thread_)
		        			/*{
		        				vnode = new Shared_VNode(parent->depth() + 1,
		        				                         static_cast<Shared_QNode*>(qnode), static_cast<Shared_QNode*>(common_qnode),
		        				                         obs);
		        				if (Globals::config.exploration_mode == UCT)
		        					static_cast<Shared_VNode*>(vnode)->visit_count_ = 1.1;
		        			}
		        			else*/
		        			{
		        				vnode = new VNode(parent->depth() + 1,
		        							qnode, common_qnode, obs);
		        			}
                //vnode->observation_particle_size = observation_particle_size_;
                vnode->obs_probs.resize(Globals::config.num_scenarios, 0);
		logd << " New node created!" << std::endl;
		children[obs] = vnode;
                vnode->obs_probs_holder = vnode;
                
                if(obs == Globals::RESIDUAL_OBS)
                {
                    vnode->extra_node = true;
                }
		double total_weight = 0;
                for(int i = 0; i < common_qnode->particles_.size();i++)
                {
                    double prob;
                    //int scenario_id = common_qnode->particles_[i]->scenario_id;
                    if(DespotWithAlphaFunctionUpdate::PedPomdpProb)
					{
	  
						prob = model->ObsProb(obs_vec, *common_qnode->particles_[i], qnode->edge());
					}
                    else
                    {
                    	prob = model->ObsProb(obs, *common_qnode->particles_[i], qnode->edge());
                    }
                    
                   // std::cout << "Obs Prob: for obs " <<  obs << " " << prob << " ";
                
		 // Terminal state is not required to be explicitly represented and may not have any observation
			vnode->particle_weights[common_qnode->particles_[i]->scenario_id] = parent->particle_weights[common_qnode->particles_[i]->scenario_id]* prob;
			total_weight += vnode->particle_weights[common_qnode->particles_[i]->scenario_id];
                        //Total weight should not be zero as one particle actually produced that observation
                        vnode->obs_probs[common_qnode->particles_[i]->scenario_id] = prob;
                        
                        residual_vnode->obs_probs[common_qnode->particles_[i]->scenario_id] =  residual_vnode->obs_probs[common_qnode->particles_[i]->scenario_id]+ prob;
                        if(residual_vnode->obs_probs[common_qnode->particles_[i]->scenario_id] > max_prob_sum)
                        {
                            max_prob_sum = residual_vnode->obs_probs[common_qnode->particles_[i]->scenario_id];
                        }
                        
                        
                }
                vnode->prob_o_given_b = total_weight;
                //std::cout << "Max prob sum " << max_prob_sum << std::endl;
                for(int i = 0; i < common_qnode->particles_.size(); i++)
                {
                    if(total_weight > 0) //total weight might be zero if particle weight is zero
                    {
                    vnode->particle_weights[common_qnode->particles_[i]->scenario_id] = vnode->particle_weights[common_qnode->particles_[i]->scenario_id]/total_weight;
                    }
                    
                    
                }
                
                logd << " Creating node for obs " << obs << std::endl;
		

		history.Add(qnode->edge(), obs);
		DESPOT::InitBounds(vnode, lb, ub, streams, history);
		history.RemoveLast();
                
		logd << " New node's bounds: (" << vnode->lower_bound() << vnode->lower_bound_alpha_vector<< ", "
			<< vnode->upper_bound() << vnode->common_parent()->default_upper_bound_alpha_vector << ")" << std::endl;
                //lower_bound += vnode->lower_bound();
		//upper_bound += vnode->upper_bound();
		//lower_bound += vnode->lower_bound()*observation_particle_size_/observation_particle_size;
		//upper_bound += vnode->upper_bound()*observation_particle_size_/observation_particle_size;
        }
        
        //Scale probs
	for (std::map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;
                if(!vnode->extra_node)
                {
            for(int i = 0; i < common_qnode->particles_.size();i++)
            {    
                vnode->obs_probs[common_qnode->particles_[i]->scenario_id] = vnode->obs_probs[common_qnode->particles_[i]->scenario_id]/max_prob_sum;
            }
            vnode->prob_o_given_b = vnode->prob_o_given_b/max_prob_sum;
            }
        }
        //Residual node
        if(common_qnode->particles_.size() > 0)
        {
            double total_weight = 0;
                for(int i = 0; i < common_qnode->particles_.size();i++)
                {
                    double prob = 1 - (residual_vnode->obs_probs[common_qnode->particles_[i]->scenario_id]/max_prob_sum);
                    
                
                    
                    //std::cout << "Obs Prob: res" <<  prob << " ";
                
		 // Terminal state is not required to be explicitly represented and may not have any observation
			residual_vnode->particle_weights[common_qnode->particles_[i]->scenario_id] = parent->particle_weights[common_qnode->particles_[i]->scenario_id]* prob;
			total_weight += residual_vnode->particle_weights[common_qnode->particles_[i]->scenario_id];
                        //Total weight should not be zero as one particle actually produced that observation
                        residual_vnode->obs_probs[common_qnode->particles_[i]->scenario_id] = prob;
                        
                        
                        
                        
                }
                residual_vnode->prob_o_given_b = total_weight;
                for(int i = 0; i < common_qnode->particles_.size(); i++)
                {
                    if(total_weight > 0) //total weight might be zero for residual node
                    {
                    residual_vnode->particle_weights[common_qnode->particles_[i]->scenario_id] = residual_vnode->particle_weights[common_qnode->particles_[i]->scenario_id]/total_weight;
                    }
                    
                }
                
                logd << " Creating node for obs " << Globals::RESIDUAL_OBS << std::endl;
		

		history.Add(qnode->edge(), Globals::RESIDUAL_OBS);
		DESPOT::InitBounds(residual_vnode, lb, ub, streams, history);
		history.RemoveLast();
                
		logd << " New node's bounds: (" << residual_vnode->lower_bound() << residual_vnode->lower_bound_alpha_vector<< ", "
			<< residual_vnode->upper_bound() << residual_vnode->common_parent()->default_upper_bound_alpha_vector << ")" << std::endl;
                //lower_bound += vnode->lower_bound();

        }
	/*if(Globals::config.use_multi_thread_){
	  //std::cout << "Unlocking inside if" << std::endl;

	  (static_cast<Shared_QNode*>(common_qnode))->unlock();
	}*/
        }
        //Copy from populated qnode
        else{
			  /*if(Globals::config.use_multi_thread_){
			  //std::cout << "Going inside else" << std::endl;
			  (static_cast<Shared_QNode*>(common_qnode))->unlock();
			  }*/
            populated_qnode = common_qnode->populating_node; //common_qnode->parent()->Child(qnode->edge()) can be wrong with multiple threads. So keeping a pointer to poluating node
            if(populated_qnode == qnode)
            {
            	//Can happen while using GPU expansion
            	;
            }
            else
            {
	      logd << "Copying from existing qnode \n";
	     /* if(Globals::config.useGPU && parent->PassGPUThreshold())
	      {

	    	  if(Globals::config.use_multi_thread_ && Globals::config.exploration_mode==UCT)
	    	  {
	    		  assert(static_cast<Shared_QNode*>(qnode)->visit_count_ == 0);
	 				static_cast<Shared_QNode*>(qnode)->visit_count_=1.1;
	    	  }
	      }*/
            std::map<OBS_TYPE, VNode*>& populated_children = populated_qnode->children();
            for (std::map<OBS_TYPE, VNode*>::iterator it = populated_children.begin();
		it != populated_children.end(); it++)
            {
                OBS_TYPE obs = it->first;
                VNode* vnode;
               /* if (Globals::config.use_multi_thread_)
				{
					vnode = new Shared_VNode(parent->depth() + 1,
											 static_cast<Shared_QNode*>(qnode), static_cast<Shared_QNode*>(common_qnode),
											 obs);
					if (Globals::config.exploration_mode == UCT)
						static_cast<Shared_VNode*>(vnode)->visit_count_ = 1.1;
				}
				else*/
				{
                vnode = new VNode(parent->depth() + 1,
			qnode, common_qnode, obs);
				}
                //vnode->observation_particle_size = 1;
		logd << " New node created!" << std::endl;
                if(obs == Globals::RESIDUAL_OBS)
                {
                    vnode->extra_node = true;
                }
		children[obs] = vnode;
                vnode->obs_probs_holder = it->second;
		if(Globals::config.useGPU)
		  {
		    vnode->num_GPU_particles_ = common_qnode->particleIDs_.size();
		  }
		   
                //vnode->obs_probs.insert(vnode->obs_probs.begin(),it->second->obs_probs.begin(), it->second->obs_probs.end());
                double total_weight = 0;
                for(int i = 0; i < common_qnode->particles_.size();i++)
                {
		  int scenario_id;
		  if(Globals::config.useGPU && parent->PassGPUThreshold())
		    {
		      scenario_id = common_qnode->particleIDs_[i];
		    }
		  else
		    {
		      scenario_id = common_qnode->particles_[i]->scenario_id;
		    }
                    double prob = vnode->obs_probs_holder->obs_probs[scenario_id];
                    
                   logd << "Obs Prob: for obs " <<  obs << " " << prob << " ";
                
		 // Terminal state is not required to be explicitly represented and may not have any observation
			vnode->particle_weights[scenario_id] = parent->particle_weights[scenario_id]* prob;
			total_weight += vnode->particle_weights[scenario_id];
                        //Total weight should not be zero as one particle actually produced that observation
                        
                        
                        
                }
                vnode->prob_o_given_b = total_weight;
                for(int i = 0; i < common_qnode->particles_.size(); i++)
                {
                    if(total_weight > 0) //total weight might be zero if particle weight is zero
                    {
		      int scenario_id;
		      if(Globals::config.useGPU && parent->PassGPUThreshold())
			{
			  scenario_id = common_qnode->particleIDs_[i];
			}
		      else
			{
			  scenario_id = common_qnode->particles_[i]->scenario_id;
			}
                    vnode->particle_weights[scenario_id] = vnode->particle_weights[scenario_id]/total_weight;
                    }
                    
                    
                }
                
                logd << " Creating node for obs " << obs << std::endl;
		

		history.Add(qnode->edge(), obs);
		DESPOT::InitBounds(vnode, lb, ub, streams, history);
		history.RemoveLast();
                
		logd << " New node's bounds: (" << vnode->lower_bound() << vnode->lower_bound_alpha_vector<< ", "
			<< vnode->upper_bound() << vnode->common_parent()->default_upper_bound_alpha_vector << ")" << std::endl;
 
                
            }
        }
        }
       
               
        
        //std::cout << "Upper bound = " << upper_bound - step_reward<< " Num particles pushed " << num_particles_pushed << std::endl;
        qnode->lower_bound_alpha_vector.resize(Globals::config.num_scenarios,0);
        //qnode->upper_bound_alpha_vector.resize(Globals::config.num_scenarios,0);
        qnode->lower_bound(Globals::NEG_INFTY);
        qnode->upper_bound(Globals::POS_INFTY);
        if(Globals::config.use_sawtooth_upper_bound)
        {
	  /*if(Globals::config.use_multi_thread_){
	    //std::cout << "Unlocking inside if" << std::endl;

	    (static_cast<Shared_QNode*>(common_qnode))->lock();
	  }*/
	  
          if(common_qnode->qnode_upper_bound_per_particle.size()==0)
          {
              common_qnode->qnode_upper_bound_per_particle.resize(Globals::config.num_scenarios, Globals::POS_INFTY);
          }
	  /*if(Globals::config.use_multi_thread_){
	    //std::cout << "Unlocking inside if" << std::endl;

	    (static_cast<Shared_QNode*>(common_qnode))->unlock();
	  }*/
	  
        }

	/*if(Globals::config.use_multi_thread_){
	  DespotWithAlphaFunctionUpdate::Update(static_cast<Shared_QNode*>(qnode), false, 7);
	}
	else*/ {
	  DespotWithAlphaFunctionUpdate::Update(qnode,7);
	}
	//qnode->step_reward = step_reward;
	//qnode->lower_bound(lower_bound);
	//qnode->upper_bound(upper_bound);
	//qnode->utility_upper_bound = upper_bound + Globals::config.pruning_constant;

	qnode->default_value = qnode->lower_bound(); // for debugging

    }
    
    /*int DespotStaticFunctionOverrideHelperForAlphaFunctionUpdate::GetObservationParticleSize(VNode* vnode)
    {
        return vnode->particles().size();
        
    }*/
    
    /*void DespotWithAlphaFunctionUpdate::CoreSearch(std::vector<State*>& particles, RandomStreams& streams) {
    
    root_ = ConstructTree(particles, streams, lower_bound_, upper_bound_,
		model_, history_, Globals::config.time_per_move, statistics_, NULL, o_helper_);
    
    }*/
    
    

/*int DespotWithAlphaFunctionUpdate::Update(Shared_VNode* vnode, bool real)
{
	//lock_guard < mutex > lck(vnode->GetMutex());//lock v_node during updation
	int upper_bound_changed = 0;
	int lower_bound_changed = 0;
	int boundary_upper_bound_changed = 0;
	vnode->lock();
	if (((VNode*) vnode)->depth() > 0 && real) {
		if ( Globals::config.exploration_mode == VIRTUAL_LOSS) //release virtual loss
			vnode->exploration_bonus += CalExplorationValue(((VNode*) vnode)->depth());
		else if ( Globals::config.exploration_mode == UCT) //release virtual loss
			vnode->exploration_bonus += CalExplorationValue(((VNode*) vnode)->depth())
										* ((VNode*) vnode)->Weight();
	}
	if (((VNode*) vnode)->IsLeaf()) {
	  vnode->unlock();
			return (4*boundary_upper_bound_changed + 2*upper_bound_changed + lower_bound_changed);
		}
	vnode->unlock();
	double lower = ((VNode*) vnode)->default_move().value;
	double upper = ((VNode*) vnode)->default_move().value;

	ValuedAction max_lower_action = ((VNode*) vnode)->default_move();
	ValuedAction max_upper_action = ((VNode*) vnode)->default_move();
	        //bool estimated_upper_exists = false;
	        //ValuedAction max_estimated_upper_bound;
	for (int action = 0; action < ((VNode*) vnode)->children().size(); action++) {
		//QNode* qnode = vnode->Child(action);
		Shared_QNode* qnode =
				    static_cast<Shared_QNode*>(((VNode*) vnode)->Child(action));
		lock_guard < mutex > lck1(qnode->GetMutex());//lock q_node during updation
				double qnode_lower_bound = 0;
				double qnode_upper_bound = 0;
				if(Globals::config.track_alpha_vector){


					qnode_lower_bound = ((QNode*)qnode)->lower_bound();
					qnode_upper_bound = ((QNode*)qnode)->upper_bound();
					if(qnode_lower_bound > lower)
					{
						lower = qnode_lower_bound;
						max_lower_action.action = action;
						max_lower_action.value = lower;
						max_lower_action.value_array = &(((QNode*)qnode)->lower_bound_alpha_vector);
					}

					if(qnode_upper_bound > upper)
					{
						upper = qnode_upper_bound;
						max_upper_action.action = action;
						max_upper_action.value = upper;
						//max_upper_action.value_array = &(qnode->upper_bound_alpha_vector);

					}

				}
	}
	if(Globals::config.use_sawtooth_upper_bound)
			{
				Shared_QNode* common_parent = vnode->common_parent();
				std::vector<double> vnode_upper_bound_per_particle;
				vnode_upper_bound_per_particle = common_parent->default_lower_bound_alpha_vector;
				for (int action = 0; action < vnode->children().size(); action++) {
					Shared_QNode* common_qnode = vnode->CommonChild(action);
					lock_guard < mutex > lck1(common_qnode->GetMutex());//lock common_qnode during updation
					for (int i = 0; i < Globals::config.num_scenarios; i++)
					{
						vnode_upper_bound_per_particle[i] = max(vnode_upper_bound_per_particle[i], common_qnode->qnode_upper_bound_per_particle[i]);
					}
				}
				//vnode->belief_mult_es = 0; //Not calculating it here as it is always recalculated at update sibling in multithreaded case
				lock_guard < mutex > lck1(common_parent->GetMutex());//lock common_parent during updation
				for (int i = 0; i < Globals::config.num_scenarios; i++){
					if(vnode_upper_bound_per_particle[i] < common_parent->vnode_upper_bound_per_particle[i])
					{
						boundary_upper_bound_changed = 1;
						common_parent->vnode_upper_bound_per_particle[i] = vnode_upper_bound_per_particle[i];
					}
					//vnode->belief_mult_es += vnode->particle_weights[i]*common_parent->vnode_upper_bound_per_particle[i];
				}
			}

		vnode->lock();
		if (lower > ((VNode*)vnode)->lower_bound()) {
			lower_bound_changed = 1;
			((VNode*)vnode)->lower_bound(lower);
					((VNode*)vnode)->lower_bound_alpha_vector = max_lower_action;
		}
		if (upper < ((VNode*)vnode)->upper_bound()) {
			upper_bound_changed = 1;
			((VNode*)vnode)->upper_bound(upper);
					//vnode->upper_bound_alpha_vector = max_upper_action;

		}
		vnode->unlock();
		return (4*boundary_upper_bound_changed + 2*upper_bound_changed + lower_bound_changed);
		        //std::cout << "Update Estimated value " <<  vnode->has_estimated_upper_bound_value << " array size " << vnode->estimated_upper_bound_alpha_vector.value_array->size() << std::endl;

}*/
int DespotWithAlphaFunctionUpdate::Update(VNode* vnode) {
    int upper_bound_changed = 0;
    int lower_bound_changed = 0;
    int boundary_upper_bound_changed = 0;

	if (vnode->IsLeaf()) {
		return (4*boundary_upper_bound_changed + 2*upper_bound_changed + lower_bound_changed);
	}

	double lower = vnode->default_move().value;
	double upper = vnode->default_move().value;
        
        
        
        ValuedAction max_lower_action = vnode->default_move();
        ValuedAction max_upper_action = vnode->default_move();
        bool estimated_upper_exists = false;
        ValuedAction max_estimated_upper_bound;
	for (int action = 0; action < vnode->children().size(); action++) {
		QNode* qnode = vnode->Child(action);
                
		
                double qnode_lower_bound = 0;
                double qnode_upper_bound = 0;
                if(Globals::config.track_alpha_vector){
                    
                    /*for (int i = 0; i < Globals::config.num_scenarios; i++)
                    { 
                        //int particle_index = qnode->particles_[i]->scenario_id;
                        qnode_lower_bound += vnode->particle_weights[i]*qnode->lower_bound_alpha_vector[i];
                        qnode_upper_bound += vnode->particle_weights[i]*qnode->upper_bound_alpha_vector[i];
                        
                    }*/
                    qnode_lower_bound = qnode->lower_bound();
                    qnode_upper_bound = qnode->upper_bound();
                    if(qnode_lower_bound > lower)
                    {
                        lower = qnode_lower_bound;
                        max_lower_action.action = action;
                        max_lower_action.value = lower;
                        max_lower_action.value_array = &(qnode->lower_bound_alpha_vector);
                    }
                    
                    if(qnode_upper_bound > upper)
                    {
                        upper = qnode_upper_bound;
                        max_upper_action.action = action;
                        max_upper_action.value = upper;
                        //max_upper_action.value_array = &(qnode->upper_bound_alpha_vector);
                        
                    }
                    
                }
                
	}
        if(Globals::config.use_sawtooth_upper_bound)
        {
            QNode* common_parent = vnode->common_parent();
            std::vector<double> vnode_upper_bound_per_particle;
            vnode_upper_bound_per_particle = common_parent->default_lower_bound_alpha_vector;
            for (int action = 0; action < vnode->children().size(); action++) { 
                QNode* common_qnode = vnode->CommonChild(action);
                for (int i = 0; i < Globals::config.num_scenarios; i++)
                {
                    vnode_upper_bound_per_particle[i] = max(vnode_upper_bound_per_particle[i], common_qnode->qnode_upper_bound_per_particle[i]);
                }
            }
            vnode->belief_mult_es = 0;
            for (int i = 0; i < Globals::config.num_scenarios; i++){
                if(vnode_upper_bound_per_particle[i] < common_parent->vnode_upper_bound_per_particle[i])
                {
                	boundary_upper_bound_changed = 1;
                    common_parent->vnode_upper_bound_per_particle[i] = vnode_upper_bound_per_particle[i];
                }
                vnode->belief_mult_es += vnode->particle_weights[i]*common_parent->vnode_upper_bound_per_particle[i];
            }
        }

	if (lower > vnode->lower_bound()) {
		lower_bound_changed = 1;
		vnode->lower_bound(lower);
                vnode->lower_bound_alpha_vector = max_lower_action;
	}
	if (upper < vnode->upper_bound()) {
		upper_bound_changed = 1;
		vnode->upper_bound(upper);
                //vnode->upper_bound_alpha_vector = max_upper_action;
                
	}
	return (4*boundary_upper_bound_changed + 2*upper_bound_changed + lower_bound_changed);
        //std::cout << "Update Estimated value " <<  vnode->has_estimated_upper_bound_value << " array size " << vnode->estimated_upper_bound_alpha_vector.value_array->size() << std::endl;
	/*if (utility_upper < vnode->utility_upper_bound) {
		vnode->utility_upper_bound = utility_upper;
	}*/
}
    


/*void DespotWithAlphaFunctionUpdate::Update(Shared_QNode* qnode, bool real, int vnode_update_status)
{


	if(vnode_update_status == 0)
		        {
		        	return;
		        }
		int lower_bound_changed = vnode_update_status % 2;
		int upper_bound_changed = (vnode_update_status % 4)/2;
		int boundary_upper_bound_changed = vnode_update_status / 4;
	//lock_guard < mutex > lck(qnode->GetMutex());//lock v_node during updation
	double upper = 0;

	Shared_QNode* common_qnode = qnode->parent()->CommonChild(qnode->edge());
	std::vector<double> lower_bound_vector;
	lower_bound_vector.resize(Globals::config.num_scenarios, 0);
	int cur_depth = -1;
	map<OBS_TYPE, VNode*>& children = ((QNode*) qnode)->children();
	for (map<OBS_TYPE, VNode*>::iterator it = children.begin();
		        it != children.end(); it++) {
			Shared_VNode* vnode = static_cast<Shared_VNode*>(it->second);
			lock_guard < mutex > lck1(vnode->GetMutex());
			for (int i = 0; i < Globals::config.num_scenarios; i++)
			{
				//int particle_index = qnode->particles_[i]->scenario_id;
				lower_bound_vector[i] += vnode->obs_probs_holder->obs_probs[i]*
						 (*vnode->lower_bound_alpha_vector.value_array)[i];
				//upper_bound_vector[i] += vnode->obs_probs_holder->obs_probs[i]*(*vnode->upper_bound_alpha_vector.value_array)[i];
				  upper += vnode->obs_probs_holder->obs_probs[i]*qnode->parent()->particle_weights[i]*((VNode*)vnode)->upper_bound();

				//obs_probablity_sum[i] = obs_probablity_sum[i] + vnode->obs_probs[i];
				//std::cout << "Scenario " << i << " " << lower_bound_vector[i]  << "," << upper_bound_vector[i] << std::endl;

			}
			cur_depth = vnode->depth();
	}
	double lower = 0;
	for (int i = 0; i < Globals::config.num_scenarios; i++)
		{

			 //lower_bound_vector[i] = (lower_bound_vector[i]) + qnode->step_reward_vector[i];
			//upper_bound_vector[i] = (upper_bound_vector[i]) + qnode->step_reward_vector[i];
			//if(obs_probablity_sum[i] > 0)
			//{
			   // std::cout << i << " " << obs_probablity_sum[i] << std::endl;
			//int particle_index = qnode->particles_[i]->scenario_id;
			lower_bound_vector[i] = lower_bound_vector[i] + common_qnode->step_reward_vector[i];
			//upper_bound_vector[i] = upper_bound_vector[i] + common_qnode->step_reward_vector[i];

			if(Globals::config.use_sawtooth_upper_bound)
			{
				lock_guard < mutex > lck1(common_qnode->GetMutex()); //locking common q node while update
				double particle_upper = common_qnode->step_reward_vector[i] + common_qnode->vnode_upper_bound_per_particle[i];
				if(particle_upper < common_qnode->qnode_upper_bound_per_particle[i])
				{
					common_qnode->qnode_upper_bound_per_particle[i] = particle_upper;
				}
			}

			//}
			//else
			//{
			//   lower_bound_vector[i] =  qnode->step_reward_vector[i];
			//upper_bound_vector[i] =  qnode->step_reward_vector[i];
		   // }


			lower += qnode->parent()->particle_weights[i]*lower_bound_vector[i];
			//upper += qnode->parent()->particle_weights[i]*upper_bound_vector[i];
			upper += qnode->parent()->particle_weights[i]*common_qnode->step_reward_vector[i];

		}

	lock_guard < mutex > lck(qnode->GetMutex());//lock q_node during updation
	if (Globals::config.exploration_mode == VIRTUAL_LOSS && real)
		{
			if (cur_depth >= 0)
				qnode->exploration_bonus += CalExplorationValue(cur_depth);
			else
				qnode->exploration_bonus = 0;
		}

	if (lower > ((QNode*)qnode)->lower_bound()) {
			((QNode*)qnode)->lower_bound(lower);
	                for (int i = 0; i < Globals::config.num_scenarios; i++)
	            {
	                    qnode->lower_bound_alpha_vector[i] = lower_bound_vector[i];
	                }
		}
		if (upper < ((QNode*)qnode)->upper_bound()) {
			((QNode*)qnode)->upper_bound(upper);

		}

}*/
void DespotWithAlphaFunctionUpdate::Update(QNode* qnode, int vnode_update_status) {
        //double lower = qnode->step_reward;
	//double upper = qnode->step_reward;
	//double utility_upper = qnode->step_reward
	//	+ Globals::config.pruning_constant;
        
        
        //std::vector<double> obs_probablity_sum;

	if(vnode_update_status == 0)
	        {
	        	return;
	        }
	int lower_bound_changed = vnode_update_status % 2;
	int upper_bound_changed = (vnode_update_status % 4)/2;
	int boundary_upper_bound_changed = vnode_update_status / 4;

     double upper = 0;   
            
        //obs_probablity_sum.resize(Globals::config.num_scenarios,0);
    QNode* common_qnode = qnode->parent()->CommonChild(qnode->edge());
        std::vector<double> lower_bound_vector;
        //std::vector<double> upper_bound_vector;
        //std::vector<double> estimated_upper_bound_vector;
        lower_bound_vector.resize(Globals::config.num_scenarios, 0);
        //upper_bound_vector.resize(Globals::config.num_scenarios, 0);
        //qnode->has_estimated_upper_bound_value = false;

        
	std::map<OBS_TYPE, VNode*>& children = qnode->children();
	for (std::map<OBS_TYPE, VNode*>::iterator it = children.begin();
		it != children.end(); it++) {
		VNode* vnode = it->second;
                
		//std::cout << "Obs is " << it->first << std::endl;
		//std::cout << "Vnode " << vnode->lower_bound_alpha_vector << " " << vnode->upper_bound_alpha_vector << std::endl;
                //std::cout << "Vnode particle weights " << vnode->particle_weights[0] << " " << vnode->particle_weights[1] << std::endl;
                //std::cout << "Vnode obs prob " << vnode->obs_probs[0] << " " << vnode->obs_probs[1] << std::endl;
                
                    for (int i = 0; i < Globals::config.num_scenarios; i++)
                    {
                        //int particle_index = qnode->particles_[i]->scenario_id;
                        lower_bound_vector[i] += vnode->obs_probs_holder->obs_probs[i]*  
                                 (*vnode->lower_bound_alpha_vector.value_array)[i];
                        //upper_bound_vector[i] += vnode->obs_probs_holder->obs_probs[i]*(*vnode->upper_bound_alpha_vector.value_array)[i];
                          upper += vnode->obs_probs_holder->obs_probs[i]*qnode->parent()->particle_weights[i]*vnode->upper_bound();
                        
                        //obs_probablity_sum[i] = obs_probablity_sum[i] + vnode->obs_probs[i];
                        //std::cout << "Scenario " << i << " " << lower_bound_vector[i]  << "," << upper_bound_vector[i] << std::endl;
                        
                    }
                    
                
                
	}
        
        
        double lower = 0;
        
        //double estimated_upper = 0;
        //std::cout << "Qnode weights" << " " << qnode->parent()->particle_weights[0]  << "," << qnode->parent()->particle_weights[1] << std::endl;
            for (int i = 0; i < Globals::config.num_scenarios; i++)
            {
                
                 //lower_bound_vector[i] = (lower_bound_vector[i]) + qnode->step_reward_vector[i];
                //upper_bound_vector[i] = (upper_bound_vector[i]) + qnode->step_reward_vector[i];
                //if(obs_probablity_sum[i] > 0)
                //{
                   // std::cout << i << " " << obs_probablity_sum[i] << std::endl;
                //int particle_index = qnode->particles_[i]->scenario_id;
                lower_bound_vector[i] = lower_bound_vector[i] + common_qnode->step_reward_vector[i];
                //upper_bound_vector[i] = upper_bound_vector[i] + common_qnode->step_reward_vector[i];
                
                if(Globals::config.use_sawtooth_upper_bound)
                {
                    double particle_upper = common_qnode->step_reward_vector[i] + common_qnode->vnode_upper_bound_per_particle[i];
                    if(particle_upper < common_qnode->qnode_upper_bound_per_particle[i])
                    {
                        common_qnode->qnode_upper_bound_per_particle[i] = particle_upper;
                    }
                }
                    
                //}
                //else
                //{
                //   lower_bound_vector[i] =  qnode->step_reward_vector[i];
                //upper_bound_vector[i] =  qnode->step_reward_vector[i]; 
               // }
                
                
                lower += qnode->parent()->particle_weights[i]*lower_bound_vector[i];
                //upper += qnode->parent()->particle_weights[i]*upper_bound_vector[i];
                upper += qnode->parent()->particle_weights[i]*common_qnode->step_reward_vector[i];
                
            }
        /*if(upper < -100)
        {
          //  std::cout << "Upper is " << upper << std::endl;
        }*/
        if (lower > qnode->lower_bound()) {
		qnode->lower_bound(lower);
                for (int i = 0; i < Globals::config.num_scenarios; i++)
            {
                    qnode->lower_bound_alpha_vector[i] = lower_bound_vector[i];
                }
	}
	if (upper < qnode->upper_bound()) {
		qnode->upper_bound(upper);
               /* for (int i = 0; i < Globals::config.num_scenarios; i++)
            {
                    qnode->upper_bound_alpha_vector[i] = upper_bound_vector[i];
                }*/
	}
        
        
	
}

/*void DespotWithAlphaFunctionUpdate::UpdateSibling(Shared_VNode* vnode, Shared_VNode* sibling_node, bool real, int vnode_update_status)
{
	if (DESPOT::Gap(sibling_node) <=0.0)
		return;
	if(vnode->IsLeaf())
		return;
	if(vnode_update_status == 0)
	{
		return;
	}
	int lower_bound_changed = vnode_update_status % 2;
	int upper_bound_changed = (vnode_update_status % 4)/2;
	int boundary_upper_bound_changed = vnode_update_status / 4;

	if(lower_bound_changed == 1)
	{
	double qnode_lower_bound = 0;
	std::vector<double> vnode_lower_bound_vector;
	ValuedAction vnode_valued_action;
	vnode->lock();//lock v_node to read lower bound alpha vector
	vnode_lower_bound_vector = *(vnode->lower_bound_alpha_vector.value_array);

	vnode_valued_action = vnode->lower_bound_alpha_vector;
	vnode->unlock();
	for (int i = 0; i < Globals::config.num_scenarios; i++)
	{
		//int particle_index = qnode->particles_[i]->scenario_id;
		qnode_lower_bound += sibling_node->particle_weights[i]*vnode_lower_bound_vector[i];


	}
	sibling_node->lock();
	if (qnode_lower_bound > ((VNode*)sibling_node)->lower_bound()) {
		((VNode*)sibling_node)->lower_bound(qnode_lower_bound);
		sibling_node->lower_bound_alpha_vector.action = vnode_valued_action.action;
		sibling_node->lower_bound_alpha_vector.value = qnode_lower_bound;
		sibling_node->lower_bound_alpha_vector_ = vnode_lower_bound_vector;
		sibling_node->lower_bound_alpha_vector.value_array = &(sibling_node->lower_bound_alpha_vector_);

	}
	sibling_node->unlock();
	}
	if(Globals::config.use_sawtooth_upper_bound)
	        {
		if(upper_bound_changed == 1 || boundary_upper_bound_changed == 1)
		{
		double vnode_belief_mult_es, vnode_upper_bound;
		vnode->lock();
		//vnode_belief_mult_es = vnode->belief_mult_es;
		vnode_upper_bound = ((VNode*)vnode)->upper_bound();
		vnode->unlock();
		std::vector<double> vnode_common_parent_vnode_upper_bound_per_particle;
		vnode->common_parent()->lock();
		vnode_common_parent_vnode_upper_bound_per_particle = vnode->common_parent()->vnode_upper_bound_per_particle;

		 vnode->common_parent()->unlock();



		 	 //Find the minimum ratio weights
	            double min_ratio = Globals::POS_INFTY;
		    double sibling_node_belief_mult_es = 0;
		    vnode_belief_mult_es = 0; //Calculate this again with same upperbound points for shared update
		    for (int i = 0; i < Globals::config.num_scenarios; i++)
	            {
	                if(vnode->particle_weights[i] != 0)
	                {
	                    double weight_ratio = sibling_node->particle_weights[i]/vnode->particle_weights[i];
	                    if(weight_ratio < min_ratio)
	                    {
	                        min_ratio = weight_ratio;
	                    }

	                }
	                //Calculate belief multiplication with individual upper bounds for sibling node

	                sibling_node_belief_mult_es += sibling_node->particle_weights[i]*vnode_common_parent_vnode_upper_bound_per_particle[i];
	                vnode_belief_mult_es += vnode->particle_weights[i]*vnode_common_parent_vnode_upper_bound_per_particle[i];
	                //No need to calculate this for vnode as it is calculated while updating it



	            }
		    sibling_node->lock();
		    //std::cout << "Min ratio is " << min_ratio << " b' belief_multi_es " << vnode->belief_mult_es << " b belief_multi_es " << sibling_node->belief_mult_es << std::endl;
	            double sawtooth_upper_bound = (min_ratio*(vnode_upper_bound - vnode_belief_mult_es)) + sibling_node_belief_mult_es;
		    //std::cout << "Calculated sawtooth bound " << sawtooth_upper_bound << std::endl;
	            if(sawtooth_upper_bound < ((VNode*)sibling_node)->upper_bound())
	            {
	                ((VNode*)sibling_node)->upper_bound(sawtooth_upper_bound);
	            }
	            sibling_node->unlock();
		}
	        }

}*/
    void DespotWithAlphaFunctionUpdate::UpdateSibling(VNode* vnode, VNode* sibling_node, int vnode_update_status) {
        /*if (sibling_node->IsLeaf()) {
		return;
	}*/

        if (DESPOT::Gap(sibling_node) <=0.0)
            return;
        if(vnode->IsLeaf())
        		return;
        if(vnode_update_status == 0)
        {
        	return;
        }
        int lower_bound_changed = vnode_update_status % 2;
        int upper_bound_changed = (vnode_update_status % 4)/2;
        int boundary_upper_bound_changed = vnode_update_status / 4;

        if(lower_bound_changed == 1)
        {
	//std::cout << "Updating sibing ";
        double qnode_lower_bound = 0;
        //double qnode_upper_bound = 0;
	for (int i = 0; i < Globals::config.num_scenarios; i++)
            { 
                //int particle_index = qnode->particles_[i]->scenario_id;
                qnode_lower_bound += sibling_node->particle_weights[i]*(*vnode->lower_bound_alpha_vector.value_array)[i];
                

            }
        //std::cout << " new lower bound " << qnode_lower_bound << " Current lower bound " << sibling_node->lower_bound() << std::endl;
        if (qnode_lower_bound > sibling_node->lower_bound()) {
		sibling_node->lower_bound(qnode_lower_bound);
                sibling_node->lower_bound_alpha_vector.action = vnode->lower_bound_alpha_vector.action;
                sibling_node->lower_bound_alpha_vector.value = qnode_lower_bound;
                sibling_node->lower_bound_alpha_vector_ = *(vnode->lower_bound_alpha_vector.value_array);
                //sibling_node->lower_bound_alpha_vector_.insert(sibling_node->lower_bound_alpha_vector_.begin(),vnode->lower_bound_alpha_vector.value_array->begin(), vnode->lower_bound_alpha_vector.value_array->end() );
                sibling_node->lower_bound_alpha_vector.value_array = &(sibling_node->lower_bound_alpha_vector_);
                //Estimate upper bound only if lower bound updated

               
        }
        }
        if(Globals::config.use_sawtooth_upper_bound)
        {
        	if(upper_bound_changed == 1 || boundary_upper_bound_changed == 1)
        	{
	  /*
	  std::cout << "Computing sawtooth" <<std::endl;
	  //print es
	  std::cout << "Singleton values [" ;
	  for (int i = 0; i < Globals::config.num_scenarios; i++)
	    {
	      std::cout << vnode->common_parent()->vnode_upper_bound_per_particle[i] << ",";
	    }
	  std::cout << "]" << std::endl;
	  std::cout << "b' weights [";
	  for (int i = 0; i < Globals::config.num_scenarios; i++)
	    {
	      std::cout << vnode->particle_weights[i] << ",";
	    }
	  std::cout << "]"<< std::endl;
	  std::cout << "b weights [";
	  for (int i = 0; i < Globals::config.num_scenarios; i++)
	    {
	      std::cout << sibling_node->particle_weights[i] << ",";
	    }
	  std::cout << "]"<< std::endl;
	  std::cout << "b' upper bound " << vnode->upper_bound() << " b upper bound " << sibling_node->upper_bound() << std::endl;
	  */
            //Find the minimum ratio weights
            double min_ratio = Globals::POS_INFTY;
	    sibling_node->belief_mult_es = 0;
	    for (int i = 0; i < Globals::config.num_scenarios; i++)
            { 
                if(vnode->particle_weights[i] != 0)
                {
                    double weight_ratio = sibling_node->particle_weights[i]/vnode->particle_weights[i];
                    if(weight_ratio < min_ratio)
                    {
                        min_ratio = weight_ratio;
                    }
                    
                }
                //Calculate belief multiplication with individual upper bounds for sibling node
         
                sibling_node->belief_mult_es += sibling_node->particle_weights[i]*vnode->common_parent()->vnode_upper_bound_per_particle[i];
                
                //No need to calculate this for vnode as it is calculated while updating it
                

                
            }
	    //std::cout << "Min ratio is " << min_ratio << " b' belief_multi_es " << vnode->belief_mult_es << " b belief_multi_es " << sibling_node->belief_mult_es << std::endl;
            double sawtooth_upper_bound = (min_ratio*(vnode->upper_bound() - vnode->belief_mult_es)) + sibling_node->belief_mult_es;
	    //std::cout << "Calculated sawtooth bound " << sawtooth_upper_bound << std::endl;
            if(sawtooth_upper_bound < sibling_node->upper_bound())
            {
                sibling_node->upper_bound(sawtooth_upper_bound);
            }
        	}
        }
        
        //std::cout << "Udate sibling Estimated value " <<  sibling_node->has_estimated_upper_bound_value << " array size " << sibling_node->estimated_upper_bound_alpha_vector.value_array->size() << std::endl;
    }



}
