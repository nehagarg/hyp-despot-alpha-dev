/*
 * File:   grasping.cpp
 * Author: neha
 *
 * Created on September 3, 2014, 10:45 AM
 */

#include "grasping_real_arm.h"
#include <despot/util/floor.h>
#include <despot/interface/lower_bound.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/builtin_policy.h>
#include <math.h>
#include "yaml-cpp/yaml.h"


#include "hyp_despot/Belief.h"
#include "hyp_despot/State.h"
#include "Display/parameters.h"


#include <string>
#include "boost/bind.hpp"
#include "VrepLogFileInterface.h"


GraspingMicoParticleBelief::GraspingMicoParticleBelief(std::vector<State*> particles,
        const DSPOMDP* model, Belief* prior, bool split): GraspingParticleBelief(particles, model,
	 prior, split), grasping_model_(static_cast<const GraspingRealArm*>(model)) {

}



   void GraspingMicoParticleBelief::Update(int action, OBS_TYPE obs) {
	history_.Add(action, obs);

	std::vector<State*> updated;
	double total_weight = 0;
	double reward;
	OBS_TYPE o;
	// Update particles
        std::cout << "Updating particles" << std::endl;
	for (int i = 0; i <particles_.size(); i++) {
		State* particle = particles_[i];
                //model_->PrintState(*particle);
		bool terminal = model_->Step(*particle, Random::RANDOM.NextDouble(),
			action, reward, o);
                if(!terminal)
                {
                	//Not needed for now
                    //grasping_model_->SyncParticleState(*particle,obs);
                }
                //model_->PrintState(*particle);
		double prob = model_->ObsProb(obs, *particle, action);
                //std::cout << "Obs Prob:" <<  prob << std::endl;

		if (!terminal && prob) { // Terminal state is not required to be explicitly represented and may not have any observation
			particle->weight *= prob;
			total_weight += particle->weight;
			updated.push_back(particle);
		} else {
			model_->Free(particle);
		}
	}

	logi << "[ParticleBelief::Update] " << updated.size()
		<< " particles survived among " << particles_.size() << std::endl;
	particles_ = updated;

	// Resample if the particle set is empty
	if (particles_.size() == 0) {
		logw << "Particle set is empty!" << std::endl;
		if (prior_ != NULL) {
			logw
				<< "Resampling by drawing random particles from prior which are consistent with history"
				<< std::endl;
			particles_ = Resample(num_particles_, *prior_, history_);
		} else {
			logw
				<< "Resampling by searching initial particles which are consistent with history"
				<< std::endl;
			particles_ = Resample(num_particles_, initial_particles_, model_,
				history_);
		}

		if (particles_.size() == 0 && state_indexer_ != NULL) {
			logw
				<< "Resampling by searching states consistent with last (action, observation) pair"
				<< std::endl;
			particles_ = Resample(num_particles_, model_, state_indexer_,
				action, obs);
		}

		if (particles_.size() == 0) {
			logw << "Resampling failed - Using initial particles" << std::endl;
			for (int i = 0; i < initial_particles_.size(); i ++)
				particles_.push_back(model_->Copy(initial_particles_[i]));
		}

		//Update total weight so that effective number of particles are computed correctly
		total_weight = 0;
                for (int i = 0; i < particles_.size(); i++) {
		    State* particle = particles_[i];
                    total_weight = total_weight + particle->weight;
                }
	}


	double weight_square_sum = 0;
	for (int i = 0; i < particles_.size(); i++) {
		State* particle = particles_[i];
		particle->weight /= total_weight;
		weight_square_sum += particle->weight * particle->weight;
	}

	// Resample if the effective number of particles is "small"
	double num_effective_particles = 1.0 / weight_square_sum;
	if (num_effective_particles < num_particles_ / 2.0) {
             logi << "Resampling because effective number of particles (" << num_effective_particles << ") is small" << std::endl;

		std::vector<State*> new_belief = ParticleBelief::Sample(num_particles_, particles_,
			model_);
		for (int i = 0; i < particles_.size(); i++)
			model_->Free(particles_[i]);

		particles_ = new_belief;
	}
}


GraspingRealArm::GraspingRealArm(int start_state_index_, int interfaceType) {

     std::cout << "Initializing grasping real arm with interface type" << interfaceType <<  std::endl;
     start_state_index = start_state_index_;
     InitializeRobotInterface(interfaceType);

     //Calling this constructor gives segmentation fault on calling robotinterface functions
     //Because cannot call constructor from constructor
    //GraspingRealArm(start_state_index_, vrepInterfacePointer);
}

GraspingRealArm::GraspingRealArm(int start_state_index_, VrepInterface* robotInterface_) {

    start_state_index = start_state_index_;
    robotInterface = robotInterface_;

}

GraspingRealArm::GraspingRealArm(std::string modelParamFileName, int start_state_index_) : LearningModel(modelParamFileName, "vrep"){
    YAML::Node config = YAML::LoadFile(modelParamFileName);
    start_state_index = start_state_index_;
     int interface_type = 0;
    if(config["interface_type"])
    {
        interface_type = config["interface_type"].as<int>();
    }

    if(config["object_mapping"])
    {
        for(int i = 0; i < config["object_mapping"].size(); i++)
        {
            RobotInterface::object_id_to_filename.push_back(config["object_mapping"][i].as<std::string>());
        }
    }



    if(config["low_friction_table"])
    {
        RobotInterface::low_friction_table = config["low_friction_table"].as<bool>();
    }
    else
    {
        RobotInterface::low_friction_table = false;
    }

    if(config["version5"])
    {
        RobotInterface::version5 = config["version5"].as<bool>();
        LearningModel::problem_name  = "vrep/ver5";
    }
    else
    {
        RobotInterface::version5 = false;
    }

    if(config["version6"])
    {
        RobotInterface::version6 = config["version6"].as<bool>();
        //Keeping learning model problem name same
        //as no change in that due to change to ver6
        LearningModel::problem_name  = "vrep/ver5";
    }
    else
    {
        RobotInterface::version6 = false;
    }

    if(config["version7"])
    {
        RobotInterface::version7 = config["version7"].as<bool>();
        LearningModel::problem_name  = "vrep/ver7";
    }
    else
    {
        RobotInterface::version7 = false;
    }

    if(config["version8"])
	{
		RobotInterface::version8 = config["version8"].as<bool>();
		LearningModel::problem_name  = "vrep/ver8";
	}
	else
	{
		RobotInterface::version8 = false;
	}

    if(config["use_data_step"])
    {
        RobotInterface::use_data_step = config["use_data_step"].as<bool>();
    }
    else
    {
        RobotInterface::use_data_step = false;
    }


     bool test_object_loaded = false;
    test_object_id = 0;
    if(config["test_object_id"])
    {
        test_object_id = config["test_object_id"].as<int>();
        if ((interface_type == 1)|| RobotInterface::use_data_step) //Vrep Data interface
        {
            RobotInterface::objects_to_be_loaded.push_back(test_object_id);
            test_object_loaded = true;
        }

    }

    if(config["get_object_belief"])
    {
        RobotInterface::get_object_belief = config["get_object_belief"].as<bool>();
    }
    else
    {
        RobotInterface::get_object_belief = false;
    }

    if(config["use_regression_models"])
    {
        RobotInterface::use_regression_models = config["use_regression_models"].as<bool>();
    }
    else
    {
        RobotInterface::use_regression_models = false;
    }

    RobotInterface::use_keras_models = Globals::config.use_keras_model;
    RobotInterface::gpuID = Globals::config.GPUid;
    /*if(config["use_keras_models"])
        {
            RobotInterface::use_keras_models = config["use_keras_models"].as<bool>();
        }
        else
        {
            RobotInterface::use_keras_models = false;
        }
	*/
    if(config["auto_load_object"])
    {
        RobotInterface::auto_load_object = config["auto_load_object"].as<bool>();
    }
    else
    {
        RobotInterface::auto_load_object = false;
    }

    if(config["use_pruned_data"])
    {
        RobotInterface::use_pruned_data = config["use_pruned_data"].as<bool>();
    }
    else
    {
        RobotInterface::use_pruned_data = false;
    }

    if(config["use_discretized_data"])
    {
        RobotInterface::use_discretized_data = config["use_discretized_data"].as<bool>();
    }
    else
    {
        RobotInterface::use_discretized_data = false;
    }


    if(config["use_probabilistic_step"])
    {
        RobotInterface::use_probabilistic_step = config["use_probabilistic_step"].as<bool>();
    }
    else
    {
        RobotInterface::use_probabilistic_step = false;
    }
    std::cout << "use probabilistic step is " << RobotInterface::use_probabilistic_step << std::endl;

    if(config["use_classifier_for_belief"])
    {
        RobotInterface::use_classifier_for_belief = config["use_classifier_for_belief"].as<bool>();
    }
    else
    {
        RobotInterface::use_classifier_for_belief = false;
    }


    if(config["check_touch"])
    {
        RobotInterface::check_touch = config["check_touch"].as<bool>();
    }
    else
    {
        RobotInterface::check_touch = true;
    }
    std::cout << "check touch is " << RobotInterface::check_touch << std::endl;


    if(config["use_binary_touch"])
    {
        RobotInterface::use_binary_touch = config["use_binary_touch"].as<bool>();
    }
    else
    {
        RobotInterface::use_binary_touch = false;
    }
    std::cout << "binary touch is " << RobotInterface::use_binary_touch << std::endl;

    if(config["use_wider_workspace"])
    {
        RobotInterface::use_wider_object_workspace = config["use_wider_workspace"].as<bool>();
    }
    else
    {
       RobotInterface::use_wider_object_workspace = false;
    }
    std::cout << "wider workspce is " << RobotInterface::use_wider_object_workspace << std::endl;

    if(config["use_probabilistic_neighbour_step"])
    {
        RobotInterface::use_probabilistic_neighbour_step = config["use_probabilistic_neighbour_step"].as<bool>();
    }
    else
    {
        RobotInterface::use_probabilistic_neighbour_step = false;
    }
    std::cout << "probabilitic neighbour step " << RobotInterface::use_probabilistic_neighbour_step << std::endl;


    if(config["use_discrete_observation_in_step"])
    {
        RobotInterface::use_discrete_observation_in_step = config["use_discrete_observation_in_step"].as<bool>();
    }
    else
    {
        RobotInterface::use_discrete_observation_in_step = false;
    }
    std::cout << "discrete observation in step " << RobotInterface::use_discrete_observation_in_step << std::endl;



    if(config["use_discrete_observation_in_update"])
    {
        RobotInterface::use_discrete_observation_in_update = config["use_discrete_observation_in_update"].as<bool>();
    }
    else
    {
        RobotInterface::use_discrete_observation_in_update = false;
    }
    std::cout << "discrete observation in update " << RobotInterface::use_discrete_observation_in_update << std::endl;


    if(config["belief_object_ids"])
    {
        for(int i = 0; i < config["belief_object_ids"].size(); i++)
        {
            int belief_object_id = config["belief_object_ids"][i].as<int>();
            if ((!test_object_loaded)|| (belief_object_id != test_object_id))
            {
                RobotInterface::objects_to_be_loaded.push_back(belief_object_id);
            }
            belief_object_ids.push_back(belief_object_id);
        }
    }
    if(belief_object_ids.size() == 0)
    {
        belief_object_ids.push_back(test_object_id);
    }



    /*start_state_index = -1;
    if(config["start_state_index"])
    {
        start_state_index = config["start_state_index"].as<int>();
    }*/

    InitializeRobotInterface(interface_type);

    //Set other variables of Robot Interface
    if(config["pick_reward"])
    {
        robotInterface->pick_reward = config["pick_reward"].as<int>();
        reward_max = robotInterface->pick_reward;
    }

    if(config["pick_penalty"])
    {
        robotInterface->pick_penalty = config["pick_penalty"].as<int>();
    }

    if(config["invalid_state_penalty"])
    {
        robotInterface->invalid_state_penalty = config["invalid_state_penalty"].as<int>();
    }

    if(config["clip_number_of_objects"])
    {
        robotInterface->clip_number_of_objects = config["clip_number_of_objects"].as<int>();
    }
    else
    {
        robotInterface->clip_number_of_objects = -1;
    }

    if(config["object_class"])
    {
        robotInterface->object_class_value = config["object_class"].as<int>();
    }
    else
    {
        robotInterface->object_class_value = -1;
    }

    if(config["object_classifier_string_name"])
    {
        robotInterface->classifier_string_name = config["object_classifier_string_name"].as<std::string>();
    }
    else
    {
        robotInterface->classifier_string_name = "";
    }

    if(RobotInterface::get_object_belief)
    {
        int weighted_obs_size = belief_object_ids.size();
        if(RobotInterface::use_classifier_for_belief)
        {
            weighted_obs_size = robotInterface->GetWeightedObservationSize();

        }
        LearningModel::problem_name  = LearningModel::problem_name + "/weighted_" + to_string(weighted_obs_size);
    }
    /*
     *
    // This will be loaded from object property file, so not loading here to
    //avoid overriding property file values
    if(config["object_min_z"])
    {
        for(int i = 0; i < config["object_min_z"].size();i++)
        {
            robotInterface->min_z_o.push_back(config["object_min_z"][i].as<double>());
        }
    }
    else
    {
        for(int i = 0; i < RobotInterface::object_id_to_filename.size() + 1; i++)
        {
            robotInterface->min_z_o.push_back(robotInterface->default_min_z_o);
        }
    }

    if(config["object_initial_pose_z"])
    {
        for(int i = 0; i < config["object_initial_pose_z"].size();i++)
        {
            robotInterface->initial_object_pose_z.push_back(config["object_initial_pose_z"][i].as<double>());
        }
    }
    else
    {
        for(int i = 0; i < RobotInterface::object_id_to_filename.size() + 1; i++)
        {
            robotInterface->initial_object_pose_z.push_back(robotInterface->default_initial_object_pose_z);
        }
    }
    */
    if(config["separate_close_reward"])
    {
        robotInterface->separate_close_reward = config["separate_close_reward"].as<bool>();
    }

    //Set model variBLES
    if(config["num_belief_particles"])
    {
        num_belief_particles = config["num_belief_particles"].as<int>();
    }



}

void GraspingRealArm::InitializeRobotInterface(int interfaceType) {

    logFileInterface = false;
    if(interfaceType == 0)
     {
        VrepInterface* vrepInterfacePointer = new VrepInterface(start_state_index);
    //
        robotInterface = vrepInterfacePointer;
     }

     if(interfaceType == 1)
     {
         std::cout << "Initializing Vrep Data Interface" << std::endl;

        VrepDataInterface* interfacePointer = new VrepDataInterface(start_state_index);
    //
        robotInterface = interfacePointer;
     }

     if(interfaceType == 2)
     {
        RealArmInterface* interfacePointer = new RealArmInterface(start_state_index);
    //
        robotInterface = interfacePointer;
     }

    if(interfaceType == 3)
    {
        VrepLogFileInterface* interfacePointer = new VrepLogFileInterface(start_state_index);
        robotInterface = interfacePointer;
        logFileInterface = true;
    }

     // Display the belief partilces
    pub_gripper = grasping_display_n.advertise<hyp_despot::State>("gripper_pose", 10);
    pub_belief = grasping_display_n.advertise<hyp_despot::Belief>("object_pose", 10);


}


void GraspingRealArm::SyncParticleState(State& state, OBS_TYPE obs) const{
     GraspingStateRealArm& grasping_state = static_cast<GraspingStateRealArm&> (state);
     GraspingObservation o = obsHashMap[obs];
     robotInterface->SyncParticleState(grasping_state,o);
}

/*
GraspingRealArm::GraspingRealArm(std::string dataFileName, int start_state_index_) {

    start_state_index = start_state_index_;
    //for (int i = 0; i < num_sampled_objects; i++) {
    //    object_id_to_radius.push_back(i + 0.5);
    //}
    //for (int i = num_sampled_objects; i < num_sampled_objects + 5; i++) {
    //    object_id_to_radius.push_back(i+1 - num_sampled_objects);
   // }
    learning_data_file_name = dataFileName;


}
*/


/*vector<HistoryWithReward*> GraspingRealArm::LearningData() const {

    vector<HistoryWithReward*> ans;

    //Load learning data
    YAML::Node config_full = YAML::LoadFile(learning_data_file_name);
    std::cout << "Yaml file loaded" << std::endl;
    int num_simulations = 0;
    for(int i = 0; i < config_full.size();i++)
    {
        if (num_simulations == 20)
        {
           // break;
        }
        //std::cout << "Config " << i << std::endl;
        //std::cout << config_full[i].Type() << std::endl;
        YAML::Node config = YAML::Load(config_full[i].as<string>());
        //std::cout << config.Type() << std::endl;
        if(config["stepInfo"].size() < 90)
        {
            num_simulations++;
            //std::cout << "stepinfo " << i << std::endl;
            //Get initial obs from initial state
            double g_l = config["roundInfo"]["state"]["g_l"].as<double>();
            double g_r = config["roundInfo"]["state"]["g_r"].as<double>();
            double x_o = config["roundInfo"]["state"]["x_o"].as<double>();
            double y_o = config["roundInfo"]["state"]["y_o"].as<double>();
            int o_id = config["roundInfo"]["state"]["o_r"].as<double>() + num_sampled_objects - 1;
            GraspingStateRealArm* grasping_state = new GraspingStateRealArm(x_o,y_o, g_r,g_l, o_id);
            double random_num = Random::RANDOM.NextDouble();
            uint64_t obs = GetObsFromState(*grasping_state, random_num, 0);

            double cummulative_reward = 0;
            HistoryWithReward* h = new HistoryWithReward();
            h->SetInitialObs(obs);
            h->SetInitialState(grasping_state);
            for(int j = 0; j < config["stepInfo"].size(); j++)
            {
                int action = GetActionIdFromString(config["stepInfo"][j]["action"].as<string>());

                //Get obs
                uint64_t obs;
                uint64_t sensor_obs = 0;
                for (int k = 0; k < 22; k++)
                {
                    sensor_obs = sensor_obs + (config["stepInfo"][j]["obs"]["sensor_obs"][k].as<int>()*pow(2,k));

                }
                int gripper_l_obs = config["stepInfo"][j]["obs"]["gripper_l_obs"].as<int>();
                int gripper_r_obs = config["stepInfo"][j]["obs"]["gripper_r_obs"].as<int>();
                int terminal_state_obs = config["stepInfo"][j]["obs"]["terminal_state_obs"].as<int>();
                int x_change_obs = config["stepInfo"][j]["obs"]["x_change_obs"].as<int>();
                int y_change_obs = config["stepInfo"][j]["obs"]["y_change_obs"].as<int>();
                int x_w_obs = config["stepInfo"][j]["obs"]["x_w_obs"].as<int>();
                int y_w_obs = config["stepInfo"][j]["obs"]["y_w_obs"].as<int>();

                if(terminal_state_obs == 0)
                {
                    obs = GetObsValue(sensor_obs, gripper_l_obs, gripper_r_obs, x_w_obs, y_w_obs, x_change_obs, y_change_obs);
                }
                else
                {
                    obs = pow(2, 22)*num_gripper_observations * num_gripper_observations * num_x_i_observations * num_y_i_observations * num_x_i_change_observations * num_y_i_change_observations;
                }

                //Get cummulative reward
                cummulative_reward = cummulative_reward + config["stepInfo"][j]["reward"].as<double>();

                h->Add(action, obs, cummulative_reward);
            }
            ans.push_back(h);
        }

    }
    for(int i = 0; i < ans.size(); i++)
    {
        //ans[i]->Print();
    }
    std::cout << ans.size() << " simulations loaded" << std::endl;
    return ans;

}
*/
template <typename T>
std::string to_string(T const& value) {
    std::stringstream sstr;
    sstr << value;
    return sstr.str();
}



/*double GraspingRealArm::GetDistance(int action1, uint64_t obs1, int action2, uint64_t obs2) const {
    //std::cout << "Calculating distance between (" << action1 <<  "," << obs1 << ") and (" << action2 <<  "," << obs2 << ")" << std::endl;
    //Distance between actions
    double action_distance = 0;
    double max_action_distance = 30;
    int action1_class = floor(action1/15);
    int action2_class = floor(action2/15);
    if(action1_class == action2_class)
    {
        if(action1_class < 4)
        {
            action_distance = 0.001*(pow(2,action1) - pow(2,action2));
            if(action_distance < 0)
            {
                action_distance = -1 * action_distance;
            }
        }
        else
        {
            if(action1 != action2) //Open gripper close gripper
            {
                action_distance = max_action_distance;
            }
        }
    }
    else
    {
        action_distance = max_action_distance;
    }
    //std::cout << "Action distance:" << action_distance << std::endl;

    //Distance between observations
    vector<int> obs1_bits = GetObservationBits(obs1);
    vector<int> obs2_bits = GetObservationBits(obs2);

    double obs_distance = 0;
    for (int i = 0; i < obs1_bits.size(); i++)
    {
        double d = obs1_bits[i] - obs2_bits[i];
        if(d < 0)
        {
            d = -1*d;
        }
        if(i < 22)
        {
            obs_distance = obs_distance + d;
        }
        else if(i < 24) //gripper width
        {
            obs_distance = obs_distance + (d/10.0);
        }
        else if(i < 26) // world coordinates
        {
            obs_distance = obs_distance + (d/34.0);
        }
        else if(i < 28)
        {
            obs_distance = obs_distance + (d/80);
        }
        else if (i < 29)//0 if both are terminal observations
        {
            if(d == 0 )
            {
                if(obs1_bits[i] == 1) // bothe are terminal

                {obs_distance = 0;
                }
            }
            else //one is terminal one is not
            {
                obs_distance = 100; //Temporary hack as terminal observation has all other observations 0. /should be remived after getting corrected simulations
            }
        }
    }







    return action_distance + (10*obs_distance);

}
*/

GraspingRealArm::~GraspingRealArm() {
}






bool GraspingRealArm::StepActual(State& state, double random_num, int action,
        double& reward, ObservationClass& obs) const {
    //std::cout << "." << std::endl;
    GraspingStateRealArm& grasping_state = static_cast<GraspingStateRealArm&> (state);
    //GraspingObservation& grasping_obs = static_cast<GraspingObservation&> (obs);

    GraspingObservation grasping_obs;

    //ros::Rate loop_rate(10);
    bool ans = robotInterface->StepActual(grasping_state, random_num, action, reward, grasping_obs);



    //Update observation class hash and store in hashMap
    std::ostringstream obs_string;
    PrintObs(grasping_obs, obs_string);
    uint64_t hashValue = obsHash(obs_string.str());
    obs.SetIntObs(hashValue);
    obsHashMap[hashValue] = grasping_obs;

    return ans;



}
/* Deterministic simulative model.*/
bool GraspingRealArm::Step(State& state, double random_num, int action,
        double& reward, ObservationClass& obs) const {

    //std::cout << " Starting step \n";
    double step_start_t = get_time_second();
    GraspingStateRealArm& grasping_state = static_cast<GraspingStateRealArm&> (state);
    GraspingObservation grasping_obs;

    bool ans = robotInterface->Step(grasping_state, random_num, action, reward, grasping_obs);
    //std::cout << "Reward " << reward << std::endl;
    //Update observation class hash
    std::ostringstream obs_string;

    if(RobotInterface::use_discrete_observation_in_step)
    {
        robotInterface->PrintDicretizedObs(grasping_obs, action, obs_string);
    }
    else
    {
        PrintObs(grasping_obs, obs_string);
    }
    uint64_t hashValue = obsHash(obs_string.str());
    obs.SetIntObs(hashValue);
    if (store_obs_hash) {
        obsHashMap[hashValue] = grasping_obs;
    }
    //Not storing the hash value in map for all cases as the observation returned is not compared using ObsProb function
    //Need to do so in StepActual

    double step_end_t = get_time_second();
   // std::cout << "Step took " << step_end_t - step_start_t << std::endl;
    //Decide if terminal state is reached

    return ans;

}

int GraspingRealArm::LatentDimensionSize() const
	{
		return robotInterface->LatentDimensionSize();
	}

int GraspingRealArm::KerasInputVectorSize() const
	{
		return robotInterface->KerasInputVectorSize();
	}

int GraspingRealArm::KerasObservationVectorSize() const
		{
			return robotInterface->KerasObservationVectorSize();
		}

void GraspingRealArm::StepKerasParticles(const std::vector<float>& keras_particle_batch, int action, std::vector<float>&random_number_vecctor,
			std::vector<tensorflow::Tensor>& outputs) const
{
	robotInterface->StepKerasParticles(keras_particle_batch,action,random_number_vecctor,outputs);
}


void GraspingRealArm::GetObservationProbability(const std::vector<float>& keras_particle_batch, const std::vector<float>& keras_obs_particle_batch, int action,
		std::vector<float>&random_number_vecctor, std::vector<tensorflow::Tensor>& outputs) const
{
	robotInterface->GetObservationProbability(keras_particle_batch,keras_obs_particle_batch,action,random_number_vecctor,outputs);
}




/* Functions related to beliefs and starting states.*/
double GraspingRealArm::ObsProb(ObservationClass obs, const State& state, int action) const {
    const GraspingStateRealArm& grasping_state = static_cast<const GraspingStateRealArm&> (state);
    std::map<uint64_t,GraspingObservation>::iterator it = obsHashMap.find(obs.GetHash());
    if(it == obsHashMap.end())
    {
        std::cout << "Obs not in hash map. This should not happen" <<  std::endl;
        assert(false);
    }
    GraspingObservation grasping_obs = it->second;

    return robotInterface->ObsProb(grasping_obs, grasping_state, action);
}


Belief* GraspingRealArm::InitialBelief(const State* start, std::string type) const
{
    std::cout << "Here" << std::endl;
    return new GraspingMicoParticleBelief(InitialBeliefParticles(start,type), this, NULL, false);
}


std::vector<State*> GraspingRealArm::InitialBeliefParticles(const State* start, std::string type) const
 {
    //std::cout << "In initial belief" << std::endl;
    std::vector<State*> particles;
    int num_particles = 0;
    std::pair <std::map<int,double>,std::vector<double> > temp_pair;
    temp_pair = robotInterface->GetBeliefObjectProbability(belief_object_ids);
    belief_object_weights = temp_pair.first;
    prior_vision_observation = temp_pair.second;
    //Gaussian belief for gaussian start state
    if (type == "GAUSSIAN" || type == "GAUSSIAN_WITH_STATE_IN" ||
           type == "UNIFORM" || type == "UNIFORM_WITH_STATE_IN" )
    {

        for(int i = 0; i < num_belief_particles; i++)
        {
            for(int j = 0; j < belief_object_ids.size(); j++)
            {

                if (robotInterface->clip_number_of_objects == -1 || belief_object_weights[belief_object_ids[j]] > 0)
                {
                    GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(start));
                    grasping_state->object_id = belief_object_ids[j];
                    if(!robotInterface->graspObjectsDynamicModelLoaded[grasping_state->object_id])
                    {
                        robotInterface->loadObjectDynamicModel(grasping_state->object_id);
                    }
                    robotInterface->GetDefaultStartState(*grasping_state);

                    while(true)
                    {
                        if (type == "GAUSSIAN" || type == "GAUSSIAN_WITH_STATE_IN" )
                        {
                        	if(RobotInterface::version8)
                        	{
                        		robotInterface->GenerateGaussianParticleFromState_V8(*grasping_state, type);
                        	}
                        	else
                        	{
                        		robotInterface->GenerateGaussianParticleFromState(*grasping_state, type);
                        	}
                        }
                        if (type == "UNIFORM" || type == "UNIFORM_WITH_STATE_IN")
                        {
                        	if(RobotInterface::version8)
                        	{
                        		robotInterface->GenerateUniformParticleFromState_V8(*grasping_state, type);
                        	}
                        	else
                        	{
                        		robotInterface->GenerateUniformParticleFromState(*grasping_state, type);
                        	}
                        }

                        if (robotInterface->IsValidState(*grasping_state))
                        {
                            break;
                        }
                    }

                    grasping_state->weight = belief_object_weights[grasping_state->object_id];
                    particles.push_back(grasping_state);
                    num_particles = num_particles + 1;
                }
            }
        }
    }

    /*if (type == "UNIFORM" || type == "UNIFORM_WITH_STATE_IN")
    {



        for(int k = 0; k < belief_object_ids.size(); k++)
        {

            for(int i = 0; i < 17; i++)
            {
                for(int j = 0; j < 17; j++)
                {
                    GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(start));
                    grasping_state->object_id = belief_object_ids[k];
                    robotInterface->GetDefaultStartState(*grasping_state);
                    grasping_state->object_pose.pose.position.y = robotInterface->initial_object_y -0.04 + (j*0.005);
                    grasping_state->object_pose.pose.position.x = robotInterface->initial_object_x -0.04 + (i*0.005);
                    particles.push_back(grasping_state);
                    num_particles = num_particles + 1;
                }
            }
        }
    }*/

    if (type == "SINGLE_PARTICLE" || type == "GAUSSIAN_WITH_STATE_IN"
            || type == "DEFAULT" || type ==  "UNIFORM_WITH_STATE_IN" )
    {
    //Single Particle Belief
        GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(start));
        if(std::find(belief_object_ids.begin(),
                    belief_object_ids.end(),
                    grasping_state->object_id ) == belief_object_ids.end()) //Object id not in belief
        {
            //Load a belief object id 0 particle
            grasping_state->object_id = belief_object_ids[0];
        }
        if(!robotInterface->graspObjectsDynamicModelLoaded[grasping_state->object_id])
        {
            robotInterface->loadObjectDynamicModel(grasping_state->object_id);
        }
        //double object_x = grasping_state->object_pose.pose.position.x;
        //double object_y = grasping_state->object_pose.pose.position.y;
        if(type == "DEFAULT")
        {
        	robotInterface->GetDefaultStartState(*grasping_state);
        }
        /*if(type!= "DEFAULT")
        {
            grasping_state->object_pose.pose.position.x = object_x;
            grasping_state->object_pose.pose.position.y = object_y;
        }*/
        grasping_state->weight = belief_object_weights[grasping_state->object_id];
        particles.push_back(grasping_state);
        num_particles = num_particles + 1;
    }


    /*
     //belief around position obtained from vision
     GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(start));
    //grasping_state->object_pose.pose.position.y = 0.1516;
    particles.push_back(grasping_state);
    num_particles = num_particles + 1;
    double particle_distance = 0.002;
    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 50; j++)
        {
            double x_add = particle_distance*(i+1);
            if(i>=5)
            {
                x_add = -1*particle_distance*(i+1-5);
            }
            double y_add = particle_distance*(j+1);
            if(j>=25)
            {
                y_add = -1*particle_distance*(j+1-25);
            }


            GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(start));
            grasping_state->object_pose.pose.position.y = 0.1516;
            grasping_state->object_pose.pose.position.x = grasping_state->object_pose.pose.position.x + x_add;
            grasping_state->object_pose.pose.position.y = grasping_state->object_pose.pose.position.y + y_add;
            if(robotInterface->IsValidState(*grasping_state))
            {
               particles.push_back(grasping_state);
               num_particles = num_particles + 1;
            }

        }
    }
    */
    /*
    //Belief for data interface
    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 10; j++)
        {


            GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(start));
            grasping_state->object_pose.pose.position.y = robotInterface->min_y_o + (j*(robotInterface->max_y_o - robotInterface->min_y_o)/9.0);
            grasping_state->object_pose.pose.position.x = robotInterface->min_x_o + (i*(robotInterface->max_x_o - robotInterface->min_x_o)/9.0);
            if(robotInterface->IsValidState(*grasping_state))
            {
               particles.push_back(grasping_state);
               num_particles = num_particles + 1;
            }

        }
    }
    */
    std::cout << "Num particles : " << num_particles << std::endl;
    double total_weight = State::Weight(particles);
    for(int i = 0; i < num_particles; i++)
    {
        particles[i]->weight = particles[i]->weight/total_weight;
    }
    //std::cout << "Num particles : " << num_particles << std::endl;
    return particles;
}


State* GraspingRealArm::CreateStartState(std::string type) const {

    if(initial_state.object_id == -1)
    {
        initial_state.object_id = test_object_id;
        //std::cout << "Creating staet state" << std::endl;
        //Object id is changed by VrepInterface. test object id is retained only by VrepDataInterface
        robotInterface->CreateStartState(initial_state, type);

    }

    GraspingStateRealArm* grasping_state = static_cast<GraspingStateRealArm*>(Copy(&initial_state));


    //cup_display.DrawRviz();
    return  grasping_state;

};


void GraspingRealArm::PrintAction(int action, std::ostream& out) const {
    robotInterface->PrintAction(action, out);
}

void GraspingRealArm::PrintBelief(const Belief& belief, std::ostream& out) const {
    out << "Printing Belief";
}

void GraspingRealArm::PrintObs(ObservationClass& obs, std::ostream& out) const {
  //std::cout << "Before printing observation" << std::endl;
    GraspingObservation grasping_obs;
  std::map<uint64_t,GraspingObservation>::iterator it = obsHashMap.find(obs.GetHash());
    if(it == obsHashMap.end())
    {
        grasping_obs= static_cast<GraspingObservation&> (obs);
    }
    else
    {
        grasping_obs = it->second;
    }

  robotInterface->PrintObs(grasping_obs, out);
}


void GraspingRealArm::PrintObs(const State& state, ObservationClass& obs, std::ostream& out) const {

    PrintObs(obs,out);
    PrintState(state, out);
}

void GraspingRealArm::PrintState(const State& state, std::ostream& out) const {
    const GraspingStateRealArm& grasping_state = static_cast<const GraspingStateRealArm&> (state);
    robotInterface->PrintState(grasping_state, out);
    if(&out != &std::cout || RobotInterface::version8)
    {
        out << grasping_state.object_id << "\n";
    }

}

void GraspingRealArm::PrintParticles(const std::vector<State*> particles, std::ostream& out) const
{
	for(int i = 0; i < particles.size(); i++)
	{
		const GraspingStateRealArm& grasping_state = static_cast<const GraspingStateRealArm&> (*(particles[i]));
		 out << grasping_state.object_id << "|";
		robotInterface->PrintState(grasping_state, out);
	}
}

// Textual display
void GraspingRealArm::DisplayBeliefs(ParticleBelief* belief,
        std::ostream& ostr) const
{

    // sample NUM_PARTICLE_DISPLAY number of particles from the belief set for
    // diaplay purpose
    hyp_despot::Belief msg;
    if(belief->particles().size() > 0)
    {
        msg.numPars = belief->particles().size();

        for(int i = 0; i < belief->particles().size(); i++)
        {
            const GraspingStateRealArm& grasping_state = static_cast<const GraspingStateRealArm&> (*(belief->particles()[i]));
             msg.belief.push_back(grasping_state.object_pose.pose.position.x);
             msg.belief.push_back(grasping_state.object_pose.pose.position.y);
             msg.belief.push_back(grasping_state.weight);
             msg.belief.push_back(grasping_state.object_id);
        }

    }
    else
        msg.numPars = 0;

    std::cout << "Published belief\n";
    ros::Rate loop_rate(10);
    while(pub_belief.getNumSubscribers() == 0)
    {
        loop_rate.sleep();
    }
    pub_belief.publish(msg);
}
void GraspingRealArm::DisplayState(const State& state, std::ostream& ostr) const
{
    const GraspingStateRealArm& grasping_state = static_cast<const GraspingStateRealArm&> (state);
    if(grasping_state.touch[0])
        std::cout << "left finger in touch" << std::endl;
    if(grasping_state.touch[1])
        std::cout << "right finger in touch" << std::endl;
    if(grasping_state.gripper_status == 0)
        std::cout << "gripper open" << std::endl;
    else if(grasping_state.gripper_status == 1)
        std::cout << "gripper closed but not stable" << std::endl;
    else if(grasping_state.gripper_status == 2)
        std::cout << "gripper closed and stable" << std::endl;
    hyp_despot::State msg;
    msg.gripper_pose = grasping_state.gripper_pose;
    msg.object_pose = grasping_state.object_pose;
    if(grasping_state.gripper_status == 1)
        msg.observation = OBS_NSTABLE;
    else if(grasping_state.gripper_status == 2)
        msg.observation = OBS_STABLE;
    else
        msg.observation = grasping_state.touch[0] * 2 + grasping_state.touch[1];
    std::cout << "Published state\n";
    ros::Rate loop_rate(10);
    while(pub_belief.getNumSubscribers() == 0)
    {
        loop_rate.sleep();
    }
    pub_gripper.publish(msg);
}


class SimpleCloseAndPickPolicy : public DefaultPolicy {
public:

    SimpleCloseAndPickPolicy(const DSPOMDP* model, ParticleLowerBound* bound)
    : DefaultPolicy(model, bound) {
    }

    int Action(const std::vector<State*>& particles,
            RandomStreams& streams, History& history) const {
        //std::cout << "Taking action in CAP" << std::endl;

        if (history.Size()) {
            if (history.LastAction() == 8) {
            	//std::cout << "Last action is 8" << std::endl;
                return 10; //10 is Pick 8 is close gripper
            }
        }
        //std::cout << "Last action is not 8 " << std::endl;
        return 8;
    }
};
/*
class GraspingObjectExplorationPolicy : public Policy {
protected:
	const GraspingRealArm* graspingRealArm_;
public:

    GraspingObjectExplorationPolicy(const DSPOMDP* model)
    : Policy(model),
        graspingRealArm_(static_cast<const GraspingRealArm*>(model)){
    }

    int Action(const vector<State*>& particles,
            RandomStreams& streams, History& history) const {
        // 44 is Move up in y with maximum value
        //std::cout << "Taking action" << std::endl;
        GraspingStateRealArm *mostLikelyState = new GraspingStateRealArm(0,0,0,0,0);
        double max_weight = 0;
        for(int i = 0; i < particles.size(); i++)
        {
            if(particles[i]->weight > max_weight)
            {
                max_weight = particles[i]->weight;
            }
        }
        int object_id[graspingRealArm_->num_sampled_objects];
        for(int  i = 0; i < graspingRealArm_->num_sampled_objects; i++)
        {
            object_id[i] = 0;
        }
        for(int i = 0; i < particles.size(); i++)
        {
            if(particles[i]->weight >= max_weight - 0.0001 )
            {
                GraspingStateRealArm *grasping_state = static_cast<GraspingStateRealArm*> (particles[i]);
                mostLikelyState->x = mostLikelyState->x + (grasping_state->x* particles[i]->weight);
                mostLikelyState->y = mostLikelyState->y + (grasping_state->y* particles[i]->weight);
                mostLikelyState->x_i = mostLikelyState->x_i + (grasping_state->x_i* particles[i]->weight);
                mostLikelyState->y_i = mostLikelyState->y_i + (grasping_state->y_i* particles[i]->weight);
                mostLikelyState->l = mostLikelyState->l + (grasping_state->l* particles[i]->weight);
                mostLikelyState->r = mostLikelyState->r + (grasping_state->r* particles[i]->weight);
                object_id[grasping_state->object_id]++;
            }


        }
        int max_votes = 0;
        for(int  i = 0; i < graspingRealArm_->num_sampled_objects; i++)
        {

            if (object_id[i] > max_votes)
            {
                mostLikelyState->object_id = i;
                max_votes = object_id[i];
            }
        }

        if(mostLikelyState->y > -15)
        {
            if(mostLikelyState->y > 0)
            {
                if(mostLikelyState->x < 0 && mostLikelyState->x > graspingRealArm_->min_y_i)
                {
                    return graspingRealArm_->A_DECREASE_X + 14;
                }
                if(mostLikelyState->x > 0 && mostLikelyState->x < graspingRealArm_->max_y_i)
                {
                    return graspingRealArm_->A_INCREASE_X + 14;
                }
                return graspingRealArm_->A_DECREASE_Y + 14;
            }
            else
            {
                if(mostLikelyState->x + mostLikelyState->r < 0 || mostLikelyState->x - mostLikelyState->l > 0)
                {
                    return graspingRealArm_->A_DECREASE_Y + 14;
                }
                else
                {
                    return graspingRealArm_->A_CLOSE;
                }

            }
        }
        else
        {
            if(mostLikelyState->x > -0.001 && mostLikelyState->x < 0.001)
            {
                return graspingRealArm_->A_INCREASE_Y + 14;
            }
            else
            {
                double dist = mostLikelyState->x;
                if(mostLikelyState->x < 0)
                {
                     dist = 0 - mostLikelyState->x;
                }
                int steps = floor(log(dist/0.001)/log(2)) ;
                if(steps > 14) steps = 14;
                if(mostLikelyState->x < 0)
                {
                    return graspingRealArm_->A_INCREASE_X + steps;
                }
                else
                {
                    return graspingRealArm_->A_DECREASE_X + steps;
                }
            }
        }

        return 44;
    }
};
*/
ScenarioLowerBound* GraspingRealArm::CreateScenarioLowerBound(std::string name, std::string particle_bound_name ) const {
    if (name == "TRIVIAL" || name == "DEFAULT") {
        //the default lower bound from lower_bound.h
        return new TrivialParticleLowerBound(this);
    } else if (name == "CAP") {
        //set EAST as the default action for each state
        return new SimpleCloseAndPickPolicy(this, new TrivialParticleLowerBound(this));
    }
    //else if (name == "OE") {
        //set EAST as the default action for each state
        //scenario_lower_bound_ = new GraspingObjectExplorationPolicy(this);
    //}
    else {

        std::cerr << "GraspingRealArm:: Unsupported lower bound algorithm: " << name << std::endl;
        exit(0);
        return NULL;
    }
}

class SimpleScenarioUpperBound: public ParticleUpperBound
{
protected:
	const GraspingRealArm* graspingRealArm_;
public:
	SimpleScenarioUpperBound(const DSPOMDP* model)
	:	ParticleUpperBound(),
		graspingRealArm_(static_cast<const GraspingRealArm*>(model))
	{

	}
	// ~GraspCupSmartParticleUpperBound();
	double Value(const State& state) const
	{

		// changed
		const GraspingStateRealArm& cstate = static_cast<const GraspingStateRealArm&>(state);
		double toClose_x = abs(cstate.object_pose.pose.position.x - cstate.gripper_pose.pose.position.x); //- 0.245;
                toClose_x = toClose_x > 0 ? toClose_x : 0;
                double toClose_y = abs(cstate.object_pose.pose.position.y - cstate.gripper_pose.pose.position.y);// - 0.036;
                toClose_y = toClose_y > 0 ? toClose_y : 0;
                return graspingRealArm_->GetMaxReward() + (-0.5) + (-1) * (std::ceil(toClose_x/0.08) + std::ceil(toClose_y / 0.08));
	}
};

ScenarioUpperBound* GraspingRealArm::CreateScenarioUpperBound(std::string name,
		std::string particle_bound_name) const
{
	if(name == "TRIVIAL" )
	{
		return new TrivialParticleUpperBound(this);
	}
	else if(name == "DEFAULT")
	{
		return new SimpleScenarioUpperBound(this);
	}
	else
	{
		std::cerr << "Unsupported lower bound algorithm: " << name << std::endl;
		exit(0);
	}
}

//Command for despot. change solver to btdespotalphast for despot-alpha
//../../../build/devel/lib/hyp_despot/despot_without_display -m config_files/low_friction_table/vrep_scene_ver8/multiObjectType/headphones_train3_reward100_penalty10/56_headphones_final-11-Nov-2015-14-14-02_instance0.yaml -v 3 --USEKERAS 1 -s 10 --solver=DESPOT -n 5 --nobs 10 -t 10 --number=27 -l CAP --GPUID 2 -u TRIVIAL --belief=SINGLE_PARTICLE > test_despot.log 2>&1
//../../../build/devel/lib/hyp_despot/despot_without_display -m config_files/low_friction_table/vrep_scene_ver8/multiObjectType/headphones_train3_reward100_penalty10/56_headphones_final-11-Nov-2015-14-14-02_instance0.yaml -v 3 --USEKERAS 1 -s 50 --solver=BTDESPOTALPHAST -n 80 --nobs 10 -t 20 --number=18 -l CAP --GPUID 2 -u TRIVIAL --belief=UNIFORM_WITH_STATE_IN > test.log 2>&1
