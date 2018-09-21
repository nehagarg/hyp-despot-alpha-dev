#include <despot/solver/DespotWithAlphaFunctionUpdate.h>
#include <despot/util/logging.h>
#include <map>

namespace despot {
    
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
        QNode* common_qnode = parent->CommonChild(qnode->edge());
        QNode* populated_qnode = NULL;
        
	streams.position(parent->depth());
	std::map<OBS_TYPE, VNode*>& children = qnode->children();

	const std::vector<State*>& particles = parent->particles();
        //int observation_particle_size = parent->observation_particle_size;


	double step_reward = 0;

        //std::vector<OBS_TYPE> observations;
	// Partition particles by observation
	std::map<OBS_TYPE, std::vector<int> > partitions;
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
                    partitions[obs].push_back(particle->scenario_id);
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
            residual_vnode = new VNode(parent->depth() + 1,
			qnode,common_qnode, Globals::RESIDUAL_OBS);
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
                //int observation_particle_size_ = partitions[obs].size();
                
                VNode* vnode = new VNode(parent->depth() + 1,
			qnode, common_qnode, obs);
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
                    prob = model->ObsProb(obs, *common_qnode->particles_[i], qnode->edge());
                
                    
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
        
       
        }
        //Copy from populated qnode
        else{
            populated_qnode = common_qnode->parent()->Child(qnode->edge());
            std::map<OBS_TYPE, VNode*>& populated_children = populated_qnode->children();
            for (std::map<OBS_TYPE, VNode*>::iterator it = populated_children.begin();
		it != populated_children.end(); it++)
            {
                OBS_TYPE obs = it->first;
                VNode* vnode = new VNode(parent->depth() + 1,
			qnode, common_qnode, obs);
                //vnode->observation_particle_size = 1;
		logd << " New node created!" << std::endl;
                if(obs == Globals::RESIDUAL_OBS)
                {
                    vnode->extra_node = true;
                }
		children[obs] = vnode;
                vnode->obs_probs_holder = it->second;
                //vnode->obs_probs.insert(vnode->obs_probs.begin(),it->second->obs_probs.begin(), it->second->obs_probs.end());
                double total_weight = 0;
                for(int i = 0; i < common_qnode->particles_.size();i++)
                {
                    double prob = vnode->obs_probs_holder->obs_probs[common_qnode->particles_[i]->scenario_id];
                    
                   // std::cout << "Obs Prob: for obs " <<  obs << " " << prob << " ";
                
		 // Terminal state is not required to be explicitly represented and may not have any observation
			vnode->particle_weights[common_qnode->particles_[i]->scenario_id] = parent->particle_weights[common_qnode->particles_[i]->scenario_id]* prob;
			total_weight += vnode->particle_weights[common_qnode->particles_[i]->scenario_id];
                        //Total weight should not be zero as one particle actually produced that observation
                        
                        
                        
                }
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
 
                
            }
            
        }
       
               
        
        //std::cout << "Upper bound = " << upper_bound - step_reward<< " Num particles pushed " << num_particles_pushed << std::endl;
        qnode->lower_bound_alpha_vector.resize(Globals::config.num_scenarios,0);
        //qnode->upper_bound_alpha_vector.resize(Globals::config.num_scenarios,0);
        qnode->lower_bound(Globals::NEG_INFTY);
        qnode->upper_bound(Globals::POS_INFTY);
        if(Globals::config.use_sawtooth_upper_bound)
        {
          if(common_qnode->qnode_upper_bound_per_particle.size()==0)
          {
              common_qnode->qnode_upper_bound_per_particle.resize(Globals::config.num_scenarios, Globals::POS_INFTY);
          }
        }
        DespotWithAlphaFunctionUpdate::Update(qnode);
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
    
    


void DespotWithAlphaFunctionUpdate::Update(VNode* vnode) {
    if (vnode->IsLeaf()) {
		return;
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
            QNode* common_parent = vnode->parent();
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
                    common_parent->vnode_upper_bound_per_particle[i] = vnode_upper_bound_per_particle[i];
                }
                vnode->belief_mult_es += vnode->particle_weights[i]*common_parent->vnode_upper_bound_per_particle[i];
            }
        }

	if (lower > vnode->lower_bound()) {
		vnode->lower_bound(lower);
                vnode->lower_bound_alpha_vector = max_lower_action;
	}
	if (upper < vnode->upper_bound()) {
		vnode->upper_bound(upper);
                //vnode->upper_bound_alpha_vector = max_upper_action;
                
	}
        //std::cout << "Update Estimated value " <<  vnode->has_estimated_upper_bound_value << " array size " << vnode->estimated_upper_bound_alpha_vector.value_array->size() << std::endl;
	/*if (utility_upper < vnode->utility_upper_bound) {
		vnode->utility_upper_bound = utility_upper;
	}*/
}
    

void DespotWithAlphaFunctionUpdate::Update(QNode* qnode) {
        //double lower = qnode->step_reward;
	//double upper = qnode->step_reward;
	//double utility_upper = qnode->step_reward
	//	+ Globals::config.pruning_constant;
        
        
        //std::vector<double> obs_probablity_sum;

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

    void DespotWithAlphaFunctionUpdate::UpdateSibling(VNode* vnode, VNode* sibling_node) {
        /*if (sibling_node->IsLeaf()) {
		return;
	}*/

        if (DESPOT::Gap(sibling_node) <=0.0)
            return;
                
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
                sibling_node->lower_bound_alpha_vector_.insert(sibling_node->lower_bound_alpha_vector_.begin(),vnode->lower_bound_alpha_vector.value_array->begin(), vnode->lower_bound_alpha_vector.value_array->end() );  
                sibling_node->lower_bound_alpha_vector.value_array = &(sibling_node->lower_bound_alpha_vector_);
                //Estimate upper bound only if lower bound updated

               
        }
        if(Globals::config.use_sawtooth_upper_bound)
        {
            //Find the minimum ratio weights
            double min_ratio = Globals::POS_INFTY;
            for (int i = 0; i < Globals::config.num_scenarios; i++)
            { 
                if(vnode->particle_weights[i] != 0)
                {
                    double weight_ratio = sibling_node->particle_weights[i]/vnode->particle_weights[i];
                    if(min_ratio < weight_ratio)
                    {
                        min_ratio = weight_ratio;
                    }
                    
                }
                //Calculate belief multiplication with individual upper bounds for sibling node
                sibling_node->belief_mult_es = 0;
                sibling_node->belief_mult_es += sibling_node->particle_weights[i]*vnode->common_parent()->vnode_upper_bound_per_particle[i];
                
                //No need to calculate this for vnode as it is calculated while updating it
                

                
            }
            double sawtooth_upper_bound = (min_ratio*(vnode->upper_bound() - vnode->belief_mult_es)) + sibling_node->belief_mult_es;
            if(sawtooth_upper_bound < sibling_node->upper_bound())
            {
                sibling_node->upper_bound(sawtooth_upper_bound);
            }
        }
        
        //std::cout << "Udate sibling Estimated value " <<  sibling_node->has_estimated_upper_bound_value << " array size " << sibling_node->estimated_upper_bound_alpha_vector.value_array->size() << std::endl;
    }



}
