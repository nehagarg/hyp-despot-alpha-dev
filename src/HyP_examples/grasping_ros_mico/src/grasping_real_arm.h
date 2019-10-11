/*
 * File:   grasping.h
 * Author: neha
 *
 * Created on September 3, 2014, 10:45 AM
 */

#ifndef GRASPING_REAL_ARM_H
#define	GRASPING_REAL_ARM_H

#include <despot/interface/pomdp.h>
#include "history_with_reward.h"
#include "LearningModel.h"


#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"


#include "VrepInterface.h"
#include "RealArmInterface.h"
#include "GraspingStateRealArm.h"
#include "GraspingObservation.h"
#include "VrepLogFileInterface.h"
#include "grasping_v4_particle_belief.h"




class GraspingRealArm;

class GraspingMicoParticleBelief : public GraspingParticleBelief {
public:
    GraspingMicoParticleBelief(std::vector<State*> particles, const DSPOMDP* model,
	Belief* prior, bool split);
   void Update(int action, OBS_TYPE obs);

   const GraspingRealArm* grasping_model_;

};


class GraspingRealArm : public LearningModel {
public:

    GraspingRealArm(const GraspingRealArm& orig);
    GraspingRealArm(int start_state_index_, int interfaceType = 0);
    GraspingRealArm(int start_state_index_, VrepInterface* roboInterface_);
    //GraspingRealArm(std::string dataFileName, int start_state_index_);
    GraspingRealArm(std::string modelParamFileName, int start_state_index_);


    virtual ~GraspingRealArm();

    int NumActions() const {
        return A_PICK + 1;
    };

     /* Deterministic simulative model.*/
    bool Step(State& state, double random_num, int action,
        double& reward, uint64_t& obs) const {
        //std::cout << "Step: This should not have been printed";
        ObservationClass observation;
        bool isTerminal = Step(state, random_num, action, reward, observation);
        obs = observation.GetHash();
        //assert(false);
        return isTerminal;}
    ;
    bool StepActual(State& state, double random_num, int action,
        double& reward, uint64_t& obs) const { //Function for actual step in simulation or real robot
                //std::cout << "Step: This should not have been printed";
        ObservationClass observation;
        bool isTerminal = StepActual(state, random_num, action, reward, observation);
        obs = observation.GetHash();
        //assert(false);
        return isTerminal;

    }
     bool Step(State& state, double random_num, int action,
        double& reward, ObservationClass& obs) const;
    bool StepActual(State& state, double random_num, int action,
        double& reward, ObservationClass& obs) const; //Function for actual step in simulation or real robot

    /* Functions related to beliefs and starting states.*/
    double ObsProb(uint64_t obs, const State& state, int action) const {
        //std::cout << "ObsProb: This should not have been printed";
        ObservationClass observation;
        observation.SetIntObs(obs);
        return ObsProb(observation, state, action);
        //return false;
    };
    double ObsProb(ObservationClass obs, const State& state, int action) const;
    Belief* InitialBelief(const State* start, std::string type = "DEFAULT") const;
    State* CreateStartState(std::string type = "DEFAULT") const;

    /* Bound-related functions.*/
    double GetMaxReward() const { return reward_max;}
    ValuedAction GetMinRewardAction() const {
        return ValuedAction(A_OPEN, -1);


    };

    ValuedAction GetBestAction() const {
            return ValuedAction(A_OPEN, -1);


        };
    ScenarioLowerBound* CreateScenarioLowerBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

    ScenarioUpperBound* CreateScenarioUpperBound(std::string name = "DEFAULT", std::string particle_bound_name = "DEFAULT") const;

    /* Memory management.*/
    State* Allocate(int state_id, double weight) const {
        //num_active_particles ++;
        GraspingStateRealArm* state = memory_pool_.Allocate();
        state->state_id = state_id;
        state->weight = weight;
        return state;
    };
    State* Copy(const State* particle) const {
        //num_active_particles ++;
       GraspingStateRealArm* state = memory_pool_.Allocate();
        *state = *static_cast<const GraspingStateRealArm*>(particle);
        state->SetAllocated();
        return state;
    };
    void Free(State* particle) const {
        //num_active_particles --;
        memory_pool_.Free(static_cast<GraspingStateRealArm*>(particle));
    };

    int NumActiveParticles() const {
	return memory_pool_.num_allocated();
    }

    /**printing functions*/

    void PrintState(const State& state, std::ostream& out = std::cout) const;

    void PrintAction(int action, std::ostream& out = std::cout) const;

    void PrintObs(const State& state, ObservationClass& obs, std::ostream& out = std::cout) const;
    void PrintObs(ObservationClass& obs, std::ostream& out = std::cout) const;
    void PrintObs(const State& state, uint64_t obs, std::ostream& out = std::cout) const {
        ObservationClass observation;
        observation.SetIntObs(obs);
        PrintObs(state,observation,out);
    }
    void PrintBelief(const Belief& belief, std::ostream& out = std::cout) const;
    void PrintParticles(const std::vector<State*> particles, std::ostream& out = std::cout) const;
    void DisplayBeliefs(ParticleBelief* belief, std::ostream& ostr) const;
    void DisplayState(const State& state, std::ostream& ostr) const;
    void InitializeRobotInterface(int interface_type) ;
    void SyncParticleState(State& state, OBS_TYPE obs) const;
    int  num_sampled_objects = 27;
    std::string learning_data_file_name;

    mutable MemoryPool<GraspingStateRealArm> memory_pool_;
    double reward_max = 20;
    double step_cost = -1;
    int start_state_index = -1;
    int num_belief_particles = 1000;
    //std::string learned_model_name = "";
    //int automatic_switching_method = 0; // 0  for threshold switching 1 for switching wirh both correct and wrong prediction 2 for switching with only correct prediction
    //std::string svm_model_prefix = "";

    RobotInterface* robotInterface;

    std::hash<std::string> obsHash;
    mutable std::map<uint64_t, GraspingObservation> obsHashMap;
    mutable GraspingStateRealArm initial_state;
    mutable std::map<int, double> belief_object_weights;
    mutable std::vector<double> prior_vision_observation;
    std::vector<int> belief_object_ids;
    int test_object_id;
    bool logFileInterface;

    //vector<HistoryWithReward*> LearningData() const;
    //ObservationClass GetInitialObs() const;
    //int GetActionIdFromString(string dataline ) const;
    std::vector<State*> InitialBeliefParticles(const State* start, std::string type="DEFAULT") const;

    mutable ros::NodeHandle grasping_display_n;
    // data used for this graspcup example
    mutable ros::Publisher pub_belief;
    mutable ros::Publisher pub_gripper;


    //Learning model
    std::vector<HistoryWithReward*> LearningData() const {
        std::vector<HistoryWithReward*> ans;
        return ans;}
    //Not used anymore should be removed
    uint64_t GetInitialObs() const {return 0;} ; //Not used anymore should be removed
    double GetDistance(int action1, uint64_t obs1, int action2, uint64_t obs2) const {return 0;} //Not used anymore should be removed

    void PrintObs(uint64_t obs, std::ostream& out = std::cout) const {
        ObservationClass observation;
        observation.SetIntObs(obs);
        PrintObs(observation,out);
    };
    void GenerateAdaboostTestFile(uint64_t obs, History h) const {}//Not used anymore should be removed
    int GetStartStateIndex() const {
        return start_state_index;
    }

    void getInputSequenceForLearnedModelFromObs(GraspingObservation o, std::ostream& oss) const
    {
        double x_ref = robotInterface->min_x_i;
        double y_ref = robotInterface->min_y_i + (0.01*7);
        int weight_belief_values = 0;
        if(LearningModel::problem_name.find("weighted") !=std::string::npos)
        {
            weight_belief_values = prior_vision_observation.size();
        }
        int inc = 1;
        if(LearningModel::problem_name.find("vrep/ver5") !=std::string::npos
            || LearningModel::problem_name.find("vrep/ver7") !=std::string::npos)
        {
            inc = 2;
        }
        for(int j = 0; j < 2; j++)
        {
            oss << o.touch_sensor_reading[j] << ",";
        }
        oss << (o.gripper_pose.pose.position.x -x_ref) << ",";
        oss << (o.gripper_pose.pose.position.y -y_ref) << ",";

        for(int j = 0; j < 4; j=j+inc)
        {
            char c = ',';
            if (j+inc >=4){
                if(weight_belief_values == 0 && LearningModel::problem_name.find("vrep/ver7") ==std::string::npos)
                        {
                            c = '*';
                        }
            }
            oss << o.finger_joint_state[j] << c;
        }
        if(LearningModel::problem_name.find("vrep/ver7") !=std::string::npos)
        {
            char c = ',';
            if(weight_belief_values == 0)
            {
                c = '*';
            }
            oss << o.vision_movement << c;
        }
        for(int j = 0; j < weight_belief_values;j++)
            {
                char c = ',';
                if(j== weight_belief_values-1)
                {
                    c = '*';
                }
                oss << prior_vision_observation[j] << c;
            }
    }
    void GetInputSequenceForLearnedmodel(History h, std::ostream& oss) const
    {
        int weight_belief_values = 0;



        if(LearningModel::problem_name.find("weighted") !=std::string::npos)
        {
            weight_belief_values = prior_vision_observation.size();

            if( h.Size() == 0)
            {
                //Create initial observation
                oss << "$" << ",";
                GraspingObservation o;
                o.getObsFromState(initial_state);
                getInputSequenceForLearnedModelFromObs(o,oss);
            }
        }

        int inc = 1;
        if(LearningModel::problem_name.find("vrep/ver5") !=std::string::npos
           || LearningModel::problem_name.find("vrep/ver7") !=std::string::npos)
        {
            inc = 2;
        }

        for(int i = 0; i < h.Size(); i++)
            {
                oss << h.Action(i) << ",";
                GraspingObservation o = obsHashMap.at(h.Observation(i));
                getInputSequenceForLearnedModelFromObs(o,oss);

            }

        if(inc ==1 )
        {
            oss << NumActions() << ",-1,-1,-1,-1,-1,-1,-1,-1 " ;
        }
        else
        {
            oss << NumActions() << ",-1,-1,-1,-1,-1,-1" ;
            if (LearningModel::problem_name.find("vrep/ver7") !=std::string::npos)
            {
                oss << ",-1";
            }
            for(int j = 0; j < weight_belief_values; j++)
            {
                oss << ",-1";
            }
            oss<<" ";
        }
    }

    ValuedAction GetNextActionFromUser(History h) const {
        if (logFileInterface)
        {
           return ((VrepLogFileInterface*)robotInterface)->NextAction(h);
        }
        else
        {
            return LearningModel::GetNextActionFromUser(h);
        }
    }

    //=================Keras Despot Functions=====================================================
	/*virtual int LatentDimensionSize() const;

	virtual int KerasInputVectorSize() const;


	virtual int KerasObservationVectorSize() const;


	virtual void StepKerasParticles(const std::vector<float>& keras_particle_batch, int action, std::vector<float>&random_number_vecctor,
			std::vector<tensorflow::Tensor>& outputs) const;


	virtual void GetObservationProbability(const std::vector<float>& keras_particle_batch, const std::vector<float>& keras_obs_particle_batch, int action,
			std::vector<float>&random_number_vecctor, std::vector<tensorflow::Tensor>& outputs) const;

*/
	//=================Hyp Despot Functions With Dummy Implementation=============================
        virtual Dvc_State* AllocGPUParticles(int numParticles, MEMORY_MODE mode,  Dvc_State*** particles_all_a = NULL ) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
            return NULL;
        };
        virtual void DeleteGPUParticles( MEMORY_MODE mode, Dvc_State** particles_all_a = NULL) const
        {
           std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        };
        virtual void CopyGPUParticlesFromParent(Dvc_State* des, Dvc_State* src, int src_offset, int* IDs,
                                                        int num_particles, bool interleave,
                                                        Dvc_RandomStreams* streams, int stream_pos,
                                                        void* CUDAstream = NULL, int shift = 0) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void ReadParticlesBackToCPU(std::vector<State*>& particles , const Dvc_State* parent_particles,
                                                    bool deepcopy) const {
                  std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
                }
        virtual Dvc_State* CopyParticlesToGPU(Dvc_State* dvc_particles, const std::vector<State*>& particles , bool deep_copy) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
            return NULL;
        }
        virtual void CopyParticleIDsToGPU(int* dvc_IDs, const std::vector<int>& particleIDs, void* CUDAstream = NULL) const
        {
            std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void InitGPUModel()
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void InitGPUUpperBound(std::string name,std::string particle_bound_name) const
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void InitGPULowerBound(std::string name,std::string particle_bound_name) const {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void DeleteGPUModel()
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void DeleteGPUUpperBound(std::string name, std::string particle_bound_name) {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void DeleteGPULowerBound(std::string name, std::string particle_bound_name) {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual void CreateMemoryPool() const
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }
        virtual void DestroyMemoryPool(MEMORY_MODE mode) const
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
        }

        virtual int ParallelismInStep() const
        {
             std::cout << "Caution! Function " << __FUNCTION__ << " haven't been implemented" << std::endl;
             return 0;
        }


};



#endif	/* GRASPING_REAL_ARM_H */
