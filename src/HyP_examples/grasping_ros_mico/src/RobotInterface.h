/* 
 * File:   RobotInterface.h
 * Author: neha
 *
 * Created on May 5, 2015, 2:11 PM
 */

#ifndef ROBOTINTERFACE_H
#define	ROBOTINTERFACE_H

#include "GraspingStateRealArm.h"
#include "GraspingObservation.h"
#include "simulation_data_reader.h"
#include "KerasModels.h"
#include <cmath> //To make abs work for double
#include "Python.h"

#include "GraspObject.h"
#include "ActionSpecification.h"
#include "Quaternion.h"
class RobotInterface {
public:
    RobotInterface();
    
    RobotInterface(const RobotInterface& orig);
    virtual ~RobotInterface();
    
    void PrintState(const GraspingStateRealArm& state, std::ostream& out = std::cout) const;

    void PrintAction(int action, std::ostream& out = std::cout) const;
    
    void PrintObs(const GraspingStateRealArm& state, GraspingObservation& obs, std::ostream& out = std::cout) const;
    void PrintObs(GraspingObservation& obs, std::ostream& out = std::cout) const;
    void PrintDicretizedObs(GraspingObservation& obs, int action, std::ostream& out = std::cout) const;
    
    virtual void CreateStartState(GraspingStateRealArm& initial_state, std::string type = "DEFAULT") const = 0;
    virtual std::pair <std::map<int,double>,std::vector<double> > GetBeliefObjectProbability(std::vector<int> belief_object_ids) const;
    double ObsProb(GraspingObservation grasping_obs, const GraspingStateRealArm& grasping_state, int action) const;
    bool Step(GraspingStateRealArm& state, double random_num, int action,
        double& reward, GraspingObservation& obs, bool debug = false) const;
    virtual bool StepActual(GraspingStateRealArm& state, double random_num, int action,
        double& reward, GraspingObservation& obs) const = 0;
    virtual bool IsValidState(GraspingStateRealArm grasping_state) const = 0;
    
    void GenerateGaussianParticleFromState(GraspingStateRealArm& initial_state, std::string type = "DEFAULT") const;
    void GenerateUniformParticleFromState(GraspingStateRealArm& initial_state, std::string type = "DEFAULT") const;
    void GenerateGaussianParticleFromState_V8(GraspingStateRealArm& initial_state, std::string type = "DEFAULT") const;
	void GenerateUniformParticleFromState_V8(GraspingStateRealArm& initial_state, std::string type = "DEFAULT") const;
    
    void GetDefaultStartState(GraspingStateRealArm& initial_state) const;
    void SyncParticleState(GraspingStateRealArm& state, GraspingObservation grasping_obs);
    bool CheckTouch(double current_sensor_values[], int on_bits[], int size = 2) const;
    static double abs(double x) {
        if (x < 0) 
        {
            return -1*x;
        }
        else
        {
            return x;
        }
    }
    /*enum { //action for amazon shelf
        A_INCREASE_X = 0 ,
        A_DECREASE_X = 4 ,
        A_INCREASE_Y = 8 ,
        A_DECREASE_Y = 12,
        A_CLOSE = 16 ,
        A_OPEN = 17,
        A_PICK = 18
    };
    */
    

    
    //static const int NUMBER_OF_OBJECTS = 100; 
    
    //For data collected
    double min_x_i ; //range for gripper movement
    double max_x_i ;  // range for gripper movement
    double min_y_i ; // range for gripper movement
    double max_y_i ; // range for gripper movement 
    double gripper_in_x_i ; //for amazon shelf , threshold after which gripper is inside shelf
    //double gripper_out_y_diff = 0.02; //for amazon shelf
    double gripper_out_y_diff ;    //for open table
    
    
    //double min_x_o_low_friction_table = 0.4319;
    //double min_x_o = 0.4586; //for table with high friction//range for object location
    //double max_x_o = 0.5517;  // range for object location
    //double min_y_o = 0.0829; // range for object location
    //double max_y_o = 0.2295; // range for object location
    
   // double min_z_o = 1.6900 ;//below this means object has fallen down //for amazon shelf
    //mutable std::vector<double> min_z_o; //= 1.1200 ; //for objects on table
    //mutable std::vector<double> initial_object_pose_z; // = 1.1248; //1.7066; //for amazon shelf
    //double default_min_z_o_low_friction_table = 1.0950;
    //double default_initial_object_pose_z_low_friction_table = 1.0998;
    //double default_min_z_o = 1.1200 ; //for objects on high friction table
    //double default_initial_object_pose_z = 1.1248; // for objects on high friction table //1.7066; //for amazon shelf
    //double max_x_o_difference = 0.01; //Not being used anywhere
    
    double pick_z_diff ; //Gripper becomes unstable at 0.12
    double pick_x_val ;
    double pick_y_val ;
    double pick_z_diff_2; //Move up after coming to original pick position so that gripper does not become unstable
    double pick_x_val_2 ;
    
    
    double initial_gripper_pose_z_low_friction_table ;
    double initial_gripper_pose_z_low_friction_table_version6 ;
    double initial_gripper_pose_z ; //for objects on high friction table  //1.73337; // for amazon shelf
    //double initial_object_x_low_friction_table = 0.4919;
    //double initial_object_x = 0.498689; //for high friction table
    //double initial_object_y = 0.148582;
    int initial_gripper_pose_index_x ;
    int initial_gripper_pose_index_y ;
    
    //Gripper orientation
    double initial_gripper_pose_xx ;
    double initial_gripper_pose_yy ;
    double initial_gripper_pose_zz ;
    double initial_gripper_pose_ww ;
    
    double initial_gripper_pose_xx_ver6  ;
    double initial_gripper_pose_yy_ver6  ;
    double initial_gripper_pose_zz_ver6  ;
    double initial_gripper_pose_ww_ver6 ;
    
    double vrep_touch_threshold ;
    double pick_reward ;
    double pick_penalty ;
    double invalid_state_penalty;
    bool separate_close_reward ;
    std::string classifier_string_name;
    int clip_number_of_objects;
    int object_class_value; //used only by real arm interface as object classification is not working well
    static bool low_friction_table;
    static bool version5;
    static bool version6;
    static bool version7;
    static bool version8;
    static bool use_data_step;
    static bool get_object_belief;
    static bool use_classifier_for_belief;
    static bool use_regression_models;
    static bool use_keras_models;
    static int gpuID;
    static bool auto_load_object;
    static bool use_pruned_data;
    static bool use_discretized_data;
    static bool check_touch;
    static bool use_probabilistic_step; //Use gaussian distribution 0f 5mm in step function
    static bool use_binary_touch; //Use binary touch to update obs prob
    static bool use_wider_object_workspace;
    static bool use_probabilistic_neighbour_step;
    static bool use_discrete_observation_in_step;
    static bool use_discrete_observation_in_update;
    double epsilon = 0.01; //Smallest step value //Reset during gathering data 
    //double epsilon_multiplier = 2; //for step increments in amazon shelf
    double epsilon_multiplier = 8; //for open table
    static std::vector<int> objects_to_be_loaded;
    static std::vector<std::string> object_id_to_filename;
    //std::string object_property_dir = "g3db_object_labels";
    //std::string object_pointcloud_dir = "point_clouds";
    
    double touch_sensor_mean[48];
    double touch_sensor_std[48];
    double touch_sensor_max[48];
    double touch_sensor_mean_closed_without_object[48];
    double touch_sensor_mean_closed_with_object[48];
    
    double touch_sensor_mean_ver5[2];
    double touch_sensor_mean_closed_without_object_ver5[2];
    double touch_sensor_mean_closed_with_object_ver5[2];
    
    mutable std::map<int, GraspObject*> graspObjects;
    mutable std::map<int, bool> graspObjectsDynamicModelLoaded;
    
    //mutable std::vector<SimulationData> simulationDataCollectionWithObject[NUMBER_OF_OBJECTS][A_PICK+1];
    //mutable std::vector<int> simulationDataIndexWithObject[NUMBER_OF_OBJECTS][A_PICK+1];
    mutable std::vector<SimulationData> simulationDataCollectionWithoutObject[A_PICK+1];
    mutable std::vector<int> simulationDataIndexWithoutObject[A_PICK+1];
    
    //mutable PyObject* dynamicModels[NUMBER_OF_OBJECTS][A_PICK+1];
    //mutable MultiScikitModels* dynamicModelsC[NUMBER_OF_OBJECTS][A_PICK+1];
    //mutable PyObject* dynamicFunction;
    int num_predictions_for_dynamic_function;
    KerasModels* keras_models;
    
    double get_action_range(int action, int action_type) const ;
    void GetObsFromData(GraspingStateRealArm grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug = false) const; //state is the state reached after performing action
    void GetObsFromDynamicModel(GraspingStateRealArm grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug = false) const; //state is the state reached after performing action

    int GetGripperStatus(GraspingStateRealArm grasping_state) const;
    int GetGripperStatus(double finger_joint_state_readings[], bool closeCalled) const;
    void GetObsUsingDefaultFunction(GraspingStateRealArm grasping_state, GraspingObservation& grasping_obs, bool debug = false) const;
    void GetNextStateAndObsFromData(GraspingStateRealArm current_grasping_state, GraspingStateRealArm& next_grasping_state, GraspingObservation& grasping_obs,double random_num, int action, bool debug = false) const;
    void GetNextStateAndObsFromDynamicModel(GraspingStateRealArm current_grasping_state, GraspingStateRealArm& next_grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug = false) const;
    SimulationData GetNextStateUsingNearestNeighbourSearch(std::vector<SimulationData> :: iterator xy_lower_bound,
        std::vector<SimulationData> :: iterator xy_upper_bound, 
        GraspingStateRealArm current_grasping_state, int action, bool debug) const;
    int GetNextStateProbabilistically(std::vector<SimulationData> :: iterator xy_lower_bound,
        std::vector<SimulationData> :: iterator xy_upper_bound, 
        GraspingStateRealArm current_grasping_state, int action, double random_num, bool debug) const;
    
    void GetNextStateAndObsUsingDefaulFunction(GraspingStateRealArm& next_grasping_state, GraspingObservation& grasping_obs, int action, double random_num,bool debug = false) const;
    void GetReward(GraspingStateRealArm initial_grasping_state, GraspingStateRealArm grasping_state, GraspingObservation grasping_obs, int action, double& reward) const;
    void ConvertObs48ToObs2(double current_sensor_values[], double on_bits[]) const;
    void UpdateNextStateValuesBasedAfterStep(GraspingStateRealArm& grasping_state, GraspingObservation grasping_obs, double reward, int action) const;
    std::map< std::string, std::vector<int> > getSimulationData(int object_id);
    std::vector<int> getSimulationDataFromFile(int object_id, std::string fileName, int readActions, bool checkDefault=true, std::string nonDefaultFilename="") const;
    bool readDataLine(int readActions, int action) const;
    void discretizeData(int object_id);
    void getRegressionModels(int object_id);
    bool isDataEntryValid(double reward, SimulationData simData, int action, int object_id) const;
    bool isDataEntrySameAsDefault(SimulationData simData, int action, int object_id) const;
    GraspObject* getGraspObject(std::string object_name) const;
    void loadObjectDynamicModel(int object_id);
    int GetWeightedObservationSize() const;
    
    virtual void GetRewardBasedOnGraspStability(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs, double& reward) const = 0;
    //virtual bool CheckTouch(double current_sensor_values[], int on_bits[], int size = 2) const = 0;
    virtual bool IsValidPick(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs) const = 0;
    virtual void CheckAndUpdateGripperBounds(GraspingStateRealArm& grasping_state, int action) const = 0;
    virtual void GetDefaultPickState(GraspingStateRealArm& grasping_state, double random_num, int pick_type = 2) const = 0;

    void StepKerasParticles(const std::vector<float>& keras_particle_batch, int action, std::vector<float>&random_number_vecctor,
    			std::vector<tensorflow::Tensor>& outputs) const;


	void GetObservationProbability(const std::vector<float>& keras_particle_batch, const std::vector<float>& keras_obs_particle_batch, int action,
			std::vector<float>&random_number_vecctor, std::vector<tensorflow::Tensor>& outputs) const;

	static int LatentDimensionSize()
		{
			return 2;
		}

	static int KerasInputVectorSize()
	{
		return 9;
	}

	static int KerasObservationVectorSize()
		{
			return 8;
		}
};



inline double Gaussian_Distribution(std::default_random_engine& generator, double mean, double var)
{
	std::normal_distribution<double> distrbution(mean, var);
	return distrbution(generator);
}

inline double Uniform_Distribution(std::default_random_engine& generator, double a, double b)
{
    std::uniform_real_distribution<double> distribution(a,b+0.0001);
    return distribution(generator);
}


#endif	/* ROBOTINTERFACE_H */

