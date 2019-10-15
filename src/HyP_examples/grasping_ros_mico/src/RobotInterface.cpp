/*
 * File:   RobotInterface.cpp
 * Author: neha
 *
 * Created on May 5, 2015, 2:11 PM
 */

#include "RobotInterface.h"
#include <chrono>
#include <python2.7/object.h>
#include <python2.7/tupleobject.h>
#include <python2.7/intobject.h>
#include <python2.7/floatobject.h>
#include <python2.7/listobject.h>
#include "math.h"
#include "grasping_real_arm.h"
#include "Display/parameters.h"


std::vector <int> RobotInterface::objects_to_be_loaded;
std::vector<std::string> RobotInterface::object_id_to_filename;
bool RobotInterface::low_friction_table;
bool RobotInterface::version5;
bool RobotInterface::version6;
bool RobotInterface::version7;
bool RobotInterface::version8;
bool RobotInterface::get_object_belief;
bool RobotInterface::use_data_step; //Start the vrep simulator to get vision observation for objecct class but use data step after that
bool RobotInterface::use_regression_models;
bool RobotInterface::use_keras_models;
int RobotInterface::gpuID;
bool RobotInterface::auto_load_object;
bool RobotInterface::use_pruned_data;
bool RobotInterface::use_discretized_data;
bool RobotInterface::use_probabilistic_step;
bool RobotInterface::use_classifier_for_belief;
bool RobotInterface::check_touch;
bool RobotInterface::use_binary_touch;
bool RobotInterface::use_wider_object_workspace;
bool RobotInterface::use_probabilistic_neighbour_step;
bool RobotInterface::use_discrete_observation_in_step;
bool RobotInterface::use_discrete_observation_in_update;
bool RobotInterface::use_point_five_for_pick;
bool RobotInterface::use_combined_prob_output;

RobotInterface::RobotInterface() {
    min_x_i = 0.3379; //range for gripper movement
    max_x_i = 0.5279;  // range for gripper movement
    min_y_i = 0.0816; // range for gripper movement
    max_y_i = 0.2316; // range for gripper movement
    gripper_in_x_i = 0.3779; //for amazon shelf , threshold after which gripper is inside shelf
    //double gripper_out_y_diff = 0.02; //for amazon shelf
    gripper_out_y_diff = 0.0;    //for open table

    pick_z_diff = 0.09; //Gripper becomes unstable at 0.12
    pick_x_val = 0.2879;
    pick_y_val = 0.1516;
    pick_x_val_2 = 0.2379;
    pick_z_diff_2 = 0.07;

    initial_gripper_pose_z_low_friction_table = 1.10835 - 0.03;
    initial_gripper_pose_z_low_friction_table_version6 = 1.0833;
    initial_gripper_pose_z = 1.10835;

    initial_gripper_pose_index_x = 0;
    initial_gripper_pose_index_y = 7;

    //Gripper orientation
    initial_gripper_pose_xx =  -0.694327;
    initial_gripper_pose_yy = -0.0171483;
    initial_gripper_pose_zz = -0.719;
    initial_gripper_pose_ww = -0.0255881;

    initial_gripper_pose_xx_ver6 = 3.43584e-05 ;
    initial_gripper_pose_yy_ver6 = -0.707165 ;
    initial_gripper_pose_zz_ver6 = 7.39992e-05 ;
    initial_gripper_pose_ww_ver6 = -0.707048;

    vrep_touch_threshold = 0.35;
    pick_reward = 20;
    pick_penalty = -100;
    invalid_state_penalty = -100;
    separate_close_reward = true;

    epsilon = 0.01; //Smallest step value //Reset during gathering data
    //double epsilon_multiplier = 2; //for step increments in amazon shelf
    epsilon_multiplier = 8; //for open table

    num_predictions_for_dynamic_function = 18;

    /*if(use_keras_models)
    {
    	//KerasModels temp_keras_model = KerasModels(A_PICK + 1);
    	//keras_models = &(temp_keras_model);
    	keras_models = new KerasModels(A_PICK + 1);
    }*/
    if(version5 || version6 || version7 || version8)
    {
        touch_sensor_mean_ver5[0] = 0.11;
        touch_sensor_mean_ver5[1] = 0.12;
        touch_sensor_mean_closed_with_object_ver5[0] = 1.58;
        touch_sensor_mean_closed_with_object_ver5[1] = 1.73;
        touch_sensor_mean_closed_without_object_ver5[0] = 0.91;
        touch_sensor_mean_closed_without_object_ver5[1] = 1.01;
    }
    else
    {
        std::ifstream infile;
        infile.open("data/sensor_mean_std_max.txt");
        double sensor_mean, sensor_std , sensor_max;
        int count = 0;
        while (infile >> sensor_mean >> sensor_std >> sensor_max)
        {
            touch_sensor_mean[count] = sensor_mean;
            touch_sensor_std[count] = sensor_std;
            touch_sensor_max[count] = sensor_max;
           // std::cout << count << ": "<< touch_sensor_mean[count] << " " << touch_sensor_std[count] << std::endl;
            count++;

        }
        infile.close();

        //Read touch sensor reading when gripper closed
        infile.open("data/gripper_closed_without_object_touch_values.txt");
        for(int i = 0; i < 48; i++)
        //for(int i = 0; i < 2; i++)
        {
            infile >> touch_sensor_mean_closed_without_object[i];
        }
        infile.close();

        //Read touch sensor reading when gripper closed with
        infile.open("data/gripper_closed_with_object_touch_values.txt");
        for(int i = 0; i < 48; i++)
        //for(int i = 0; i < 2; i++)
        {
            infile >> touch_sensor_mean_closed_with_object[i];
        }
        infile.close();

    }
    if(low_friction_table)
    {
        if(version6 || version7 || version8)
        {
            initial_gripper_pose_z = initial_gripper_pose_z_low_friction_table_version6;
            initial_gripper_pose_xx = initial_gripper_pose_xx_ver6;
            initial_gripper_pose_yy = initial_gripper_pose_yy_ver6;
            initial_gripper_pose_zz = initial_gripper_pose_zz_ver6;
            initial_gripper_pose_ww = initial_gripper_pose_ww_ver6;
        }
        else
        {
           initial_gripper_pose_z = initial_gripper_pose_z_low_friction_table;
        }

    }
    //Load simulation data for belief object
    for(int i = 0; i < objects_to_be_loaded.size(); i++)
    {
        int object_id = objects_to_be_loaded[i];
        std::cout << "Loading object " << object_id << " with filename " << object_id_to_filename[object_id] << std::endl;


        graspObjects[object_id] = getGraspObject(object_id_to_filename[object_id]);


    }

}

RobotInterface::RobotInterface(const RobotInterface& orig) {

}

RobotInterface::~RobotInterface() {
	/*if(use_keras_models)
	{
		delete keras_models;
	}*/
}

void RobotInterface::loadObjectDynamicModel(int object_id) {
    graspObjectsDynamicModelLoaded[object_id] = true;
    std::cout << "Loading object " << object_id << " with filename " << object_id_to_filename[object_id] << std::endl;

    if(use_regression_models)
        {
            getRegressionModels(object_id);
        }
    /*else if(use_keras_models)
    {
    	keras_models->load_keras_models();
    }*/
        else
        {
        	if(RobotInterface::version8)
        	{
        		getSimulationDataFromPythonGeneratedTxtFiles(object_id);
        	}
        	else
        	{
        		getSimulationData( object_id);
        		//std::cout << graspObjects[object_id]->simulationDataCollectionWithObject[0].size() << " entries for action 0" << std::endl;
        		if(use_discretized_data)
        		{
        			discretizeData(object_id);
        		}
        	}
        }
}

GraspObject* RobotInterface::getGraspObject(std::string object_name) const{
    std::string data_dir;
    if(low_friction_table)
        {
            data_dir = "data_low_friction_table_exp";
            if(use_wider_object_workspace)
            {
                data_dir = data_dir + "_wider_object_workspace";
            }
            if(version5)
            {
                data_dir = data_dir + "_ver5";
            }
            if(version6)
            {
                data_dir = data_dir + "_ver6";
            }
            if(version7)
            {
                data_dir = data_dir + "_ver7";
            }
            if(version8)
            {
                data_dir = data_dir + "_ver8";
            }
            
        }
        else
        {
            data_dir = "data_table_exp";
        }

    return new GraspObject(object_name, data_dir, low_friction_table);
}

void RobotInterface::getRegressionModels(int object_id) {
    /*

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('scripts')");
    std::cout << "Getting regression models" << std::endl;
    PyObject *pName;
    pName = PyString_FromString("grasping_dynamic_model");
    std::cout << "Loded python function" << std::endl;
    PyObject *pModule = PyImport_Import(pName);
    std::cout << "Loded python function" << std::endl;
    if(pModule == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"scripts/grasping_dynamic_model\"\n");
        assert(0==1);
    }
    Py_DECREF(pName);

    dynamicFunction = PyObject_GetAttrString(pModule, "get_model_prediction");
    PyObject *load_function = PyObject_GetAttrString(pModule, "load_model");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"load_model\"\n");
    }


    PyObject *pArgs, *pValue;

    std::string regression_model_dir =  object_id_to_filename[object_id];

    for(int i = 0;i<A_PICK+1; i++)
    {
        std::string classifier_type = "DTR";
        if(i==A_PICK)
        {
            classifier_type = "linear";
        }
        pArgs = PyTuple_New(4);
        pValue = PyString_FromString(classifier_type.c_str());
        // pValue reference stolen here:
        PyTuple_SetItem(pArgs, 0, pValue);
        pValue = PyInt_FromLong(i);
        // pValue reference stolen here:
        PyTuple_SetItem(pArgs,1,pValue);
        pValue = PyString_FromString(regression_model_dir.c_str());
        // pValue reference stolen here:
        PyTuple_SetItem(pArgs,2,pValue);
        pValue = PyInt_FromLong(num_predictions_for_dynamic_function);
        // pValue reference stolen here:
        PyTuple_SetItem(pArgs,3,pValue);
        dynamicModels[object_id][i] = PyObject_CallObject(load_function, pArgs);
        Py_DECREF(pArgs);

    }
    Py_DECREF(load_function);
    Py_DECREF(pModule);
    */
    std::string regression_model_dirC =  graspObjects[object_id]->getRegressionModelDir() ;//object_id_to_filename[object_id];
    for(int i = 0;i<A_PICK+1; i++)
    {
        std::string classifier_type_c = "DTR";
        if(i==A_PICK)
        {
            classifier_type_c = "linear";
        }

        graspObjects[object_id]->dynamicModelsC[i] = new MultiScikitModels(regression_model_dirC,
                classifier_type_c,i,num_predictions_for_dynamic_function);

    }

}

void RobotInterface::discretizeData(int object_id) {
    SimulationData tempData;
    for(int i = 0;i<A_PICK+1; i++)
    {
        int num_entries = (graspObjects[object_id]->simulationDataCollectionWithObject[i]).size();
        int margin_data_count = 0;
        for(int j=0; j< num_entries; j++)
        {
           double x1 = graspObjects[object_id]->simulationDataCollectionWithObject[i][j].current_object_pose.pose.position.x - graspObjects[object_id]->simulationDataCollectionWithObject[i][j].current_gripper_pose.pose.position.x;
           double y1 = graspObjects[object_id]->simulationDataCollectionWithObject[i][j].current_object_pose.pose.position.y - graspObjects[object_id]->simulationDataCollectionWithObject[i][j].current_gripper_pose.pose.position.y;
           if(RobotInterface::version8)
           {
        	   double theta_z_degree = graspObjects[object_id]->simulationDataCollectionWithObject[i][j].theta_z_degree_current_object_pose;
        	   (graspObjects[object_id]->discretizedSimulationDataInitState[i][graspObjects[object_id]->getDiscretizationIndex(x1,y1,theta_z_degree)]).push_back(j);

           }
           else
           {
        	   (graspObjects[object_id]->discretizedSimulationDataWithoutAngleInitState[i][graspObjects[object_id]->getDiscretizationIndex(x1,y1)]).push_back(j);
           }
           x1 = graspObjects[object_id]->simulationDataCollectionWithObject[i][j].next_object_pose.pose.position.x - graspObjects[object_id]->simulationDataCollectionWithObject[i][j].next_gripper_pose.pose.position.x;
           y1 = graspObjects[object_id]->simulationDataCollectionWithObject[i][j].next_object_pose.pose.position.y - graspObjects[object_id]->simulationDataCollectionWithObject[i][j].next_gripper_pose.pose.position.y;
           if(RobotInterface::version8)
           {
        	   double theta_z_degree = graspObjects[object_id]->simulationDataCollectionWithObject[i][j].theta_z_degree_next_object_pose;
        	   (graspObjects[object_id]->discretizedSimulationDataNextState[i][graspObjects[object_id]->getDiscretizationIndex(x1,y1, theta_z_degree)]).push_back(j);
           }
           else
           {
        	   (graspObjects[object_id]->discretizedSimulationDataWithoutAngleNextState[i][graspObjects[object_id]->getDiscretizationIndex(x1,y1)]).push_back(j);

           }
           /*
           //Print margin data
           if(i < A_CLOSE)
            {

               tempData = graspObjects[object_id]->simulationDataCollectionWithObject[i][j];
                int action_offset = (i/(A_DECREASE_X - A_INCREASE_X)) * (A_DECREASE_X - A_INCREASE_X);
                double action_range = get_action_range(i, action_offset);
                int on_bits[2];
                if(i < A_DECREASE_X) //action is increase x
                {
                    if((tempData.next_gripper_pose.pose.position.x - tempData.current_gripper_pose.pose.position.x ) < action_range)
                    {
                        if(tempData.next_gripper_pose.pose.position.x > max_x_i)
                        {
                            tempData.PrintSimulationData();
                            margin_data_count++;
                        }
                    }
                }
                else if (i < A_INCREASE_Y) //action is decrease x
                {
                    if((-tempData.next_gripper_pose.pose.position.x + tempData.current_gripper_pose.pose.position.x ) < action_range)
                    {
                        if(tempData.next_gripper_pose.pose.position.x < min_x_i)
                        {
                            tempData.PrintSimulationData();
                            margin_data_count++;
                        }
                    }
                }
                else if (i < A_DECREASE_Y) //action is increase y
                {
                    if((tempData.next_gripper_pose.pose.position.y - tempData.current_gripper_pose.pose.position.y ) < action_range)
                    {
                        if(tempData.next_gripper_pose.pose.position.y > max_y_i)
                        {
                            tempData.PrintSimulationData();
                            margin_data_count++;
                        }
                    }
                }
                else //action is decrease y
                {
                    if((-tempData.next_gripper_pose.pose.position.y + tempData.current_gripper_pose.pose.position.y ) < action_range)
                    {
                        if(tempData.next_gripper_pose.pose.position.y < min_y_i)
                        {
                            tempData.PrintSimulationData();
                            margin_data_count++;
                        }
                    }
                }


            }*/

        }
        //std::cout << margin_data_count << " data at boundary for action " << i << std::endl;


        std::cout<< (graspObjects[object_id]->discretizedSimulationDataInitState[i]).size() << " discretized init state entries" << std::endl;
        std::cout<< (graspObjects[object_id]->discretizedSimulationDataNextState[i]).size() << " discretized next state entries" << std::endl;
        /*for(auto elem: graspObjects[object_id]->discretizedSimulationDataInitState[i])
        {
            std::cout << elem.first.first << "," << elem.first.second << "=>" << elem.second.size() << " entries\n";
        }*/
    }
}

bool RobotInterface::isDataEntrySameAsDefault(SimulationData simData, int action, int object_id) const{
    GraspingStateRealArm initDummyState;
    for(int i = 0; i < 4; i++)
    {
        initDummyState.finger_joint_state[i] = simData.current_finger_joint_state[i];
    }
    initDummyState.gripper_pose = simData.current_gripper_pose;
    initDummyState.object_pose = simData.current_object_pose;
    initDummyState.object_id = object_id;
    GraspingStateRealArm nextState(initDummyState);

    if(action==A_OPEN || action==A_PICK)
    {
        nextState.closeCalled = true;
    }

    GraspingStateRealArm initState(nextState);
    GraspingObservation graspingObs;

    GetNextStateAndObsUsingDefaulFunction(nextState, graspingObs, action, 0.0);

    double reward;
    GetReward(initState, nextState, graspingObs, action, reward);
    UpdateNextStateValuesBasedAfterStep(nextState, graspingObs, reward, action);
    //std::vector<double> nextState_vec = nextState.getStateAsVector();



    GraspingStateRealArm nextSimDataState;

    for(int i = 0; i < 4; i++)
    {
        nextSimDataState.finger_joint_state[i] = simData.next_finger_joint_state[i];
    }
    nextSimDataState.gripper_pose = simData.next_gripper_pose;
    nextSimDataState.object_pose = simData.next_object_pose;
    nextSimDataState.object_id = object_id;
    for(int i = 0; i < 2; i++)
    {
        nextSimDataState.touch_value[i] = simData.touch_sensor_reading[i];
    }

    if(action==A_CLOSE || action == A_PICK)
    {
        nextSimDataState.closeCalled = true;
    }
    if(action == A_OPEN)
    {
        nextSimDataState.closeCalled = false;
    }
    GraspingObservation simDataObs;
    simDataObs.getObsFromState(nextSimDataState);

    GetReward(initState, nextSimDataState, simDataObs, action, reward);
    UpdateNextStateValuesBasedAfterStep(nextSimDataState, simDataObs, reward, action);

    //std::vector<double> nextSimDataState_vec = nextSimDataState.getStateAsVector();
    bool defaultState = true;
    if(action == A_PICK)
    {
        //Only compare pick validity
        bool predictedPick = IsValidPick(nextState, graspingObs);
        bool simDataPick = IsValidPick(nextSimDataState, simDataObs);
        if(predictedPick != simDataPick)
        {
            defaultState = false;
        }
    }
    else
    {
        defaultState = nextState.stateEqual(nextSimDataState);

    }
    if(!defaultState)
    {
        PrintObs(nextState, graspingObs, std::cout);
        simData.PrintSimulationData(std::cout);
        PrintAction(action);
    }
    return defaultState;




}


bool RobotInterface::isDataEntryValid(double reward, SimulationData simData, int action, int object_id) const{
    bool ans = false;
    if(reward != -1000 && reward != -2000 && reward != -3000)
    {


        GraspingStateRealArm initDummyState;
        for(int i = 0; i < 4; i++)
        {
            initDummyState.finger_joint_state[i] = simData.current_finger_joint_state[i];
        }
        initDummyState.gripper_pose = simData.current_gripper_pose;
        initDummyState.object_pose = simData.current_object_pose;
        initDummyState.object_id = object_id;
        GraspingStateRealArm nextState(initDummyState);

        if(action==A_OPEN || action==A_PICK)
        {
            nextState.closeCalled = true;
        }
        if(VrepDataInterface::IsValidStateStatic(nextState, graspObjects[object_id],
                min_x_i, max_x_i, min_y_i, max_y_i, gripper_out_y_diff))
        {
            ans = true;
        }

        /*if ((simData.current_object_pose.pose.position.z - default_initial_object_pose_z)
                < max_x_o_difference)
        {
            ans = true;
        }
        else
        {
            std::cout << "Data entry invalid because of faulty object pose: " << action << std::endl;
            simData.PrintSimulationData();
        }*/
    }
    return ans;

}

bool RobotInterface::readDataLine(int readActions, int action) const {
    if(readActions == 1)
    {
        return action == A_OPEN;
    }
    if(readActions == 2)
    {
        return action == A_PICK;
    }
    if(readActions == 0)
    {
        return action != A_OPEN;
    }
    if(readActions == 3)
    {
        return action <= A_CLOSE;
    }
    return true;
}

std::vector<int> RobotInterface::getSimulationDataFromFile(int object_id, std::string simulationFileName, int readActions, bool checkDefault, std::string nonDefaultFilename) const{

    std::vector<int> ans ;
    //Read simualtion data with object

    SimulationDataReader simDataReader;
    if(RobotInterface::version7)
    {
        simDataReader = SimulationDataReader(7);
    }
    else if(RobotInterface::version8)
    {
        simDataReader = SimulationDataReader(8);
    }
    else
    {
        simDataReader = SimulationDataReader();
    }
    std::ifstream simulationDataFile;
    std::ofstream myfile;
    simulationDataFile.open(simulationFileName);
    if(!simulationDataFile.good())
    {
        std::cout << simulationFileName << "is not good" << std::endl;
        return ans;
    }
    if(checkDefault)
    {
        myfile.open(nonDefaultFilename);
    }
    int i = 0;
    while(!simulationDataFile.eof())
    {
        SimulationData simData; double reward; int action;
        std::string line;
        std::getline(simulationDataFile, line);
        while(line.find("#") == 0)
        {
            std::cout << "Skipping" << line;
           std::getline(simulationDataFile, line);
        }
        std::istringstream iss(line);
        //TODO check for nan
        //simData.current_gripper_pose.pose.position.x = temp_read;
       // simDataReader.parseSimulationDataLine(simulationDataFile, simData, action, reward);
        simDataReader.parseSimulationDataLineTableData(iss, simData, action, reward);
        /*t_count++;
        if(t_count > 10)
        {
            exit(0);
        }*/
        //std::cout << reward << " " << action << "*";
        if (isDataEntryValid(reward, simData, action, object_id) && !line.empty()
                && (line.find("nan")== std::string::npos))  //(reward != -1000 && reward != -2000)
        {
            if(readDataLine(readActions,action))
            {
                if(!checkDefault || !isDataEntrySameAsDefault(simData, action, object_id))
                {
                    //TODO also filter the states where the nexxt state and observation is same as given by defulat state
                    graspObjects[object_id]->simulationDataCollectionWithObject[action].push_back(simData);

                    //graspObjects[object_id]->simulationDataIndexWithObject[action].push_back(graspObjects[object_id]->simulationDataIndexWithObject[action].size());
                    ans.push_back(i);
                    if(checkDefault)
                    {
                        myfile << line << std::endl;
                    }
                }
            }
        }
        i++;
    }
    /*for(int j = 0; j < A_PICK + 1; j++)
    {
      std::cout << "(" << j << "," << graspObjects[object_id]->simulationDataCollectionWithObject[j].size() << ")";
    }*/
    //std::cout << std::endl;
    simulationDataFile.close();
    myfile.close();
    return ans;
}
void RobotInterface::getSimulationDataFromPythonGeneratedTxtFiles(int object_id)
{

	SimulationDataReader simDataReader = SimulationDataReader(8);
	for(int action = 0; action <= A_PICK; action++)
	{
		std::string txt_file_name = GraspObject::with_pca_txt_file_dir +
				"/with_pca_" + graspObjects[object_id]->GetObject_name() + "_"
				+ std::to_string(action) + ".txt";
		simDataReader.parsePythonGeneratedTxtFile(txt_file_name, action, graspObjects[object_id]);
		if(action !=A_PICK)
		{
			if(GraspObject::default_image_pca[action].size() == 0)
			{
				txt_file_name = GraspObject::with_pca_txt_file_dir +
								"/default_image_pca_"
								+ std::to_string(action) + ".txt";
				std::ifstream simulationDataFile;
				simulationDataFile.open(txt_file_name);
				std::string line;
				std::getline(simulationDataFile, line);
				std::istringstream iss(line);
				double image_pca_components;
				for(int i = 0; i < 150; i++)
				{
					iss >> image_pca_components;
					GraspObject::default_image_pca[action].push_back(image_pca_components);
				}
			}
		}

	}
}
std::map<std::string, std::vector<int> > RobotInterface::getSimulationData(int object_id) {

   std::map<std::string, std::vector<int> > ans;
    //Read simualtion data with object
    //SimulationDataReader simDataReader;
    //std::ifstream simulationDataFile;

    std::vector<std::string> sasoFilenames = graspObjects[object_id]->getSASOFilenames(use_pruned_data, use_discretized_data);

    for(int i = 0; i < sasoFilenames.size(); i++)
    {
        int readAction = -1;
        //std::cout << sasoFilenames[i] << " ";
        //simulationDataFile.open("data/simulationData1_allParts.txt");
        if(sasoFilenames[i].find("_allActions.txt")!=std::string::npos)
        {
            //std::string simulationFileName = sasoFilenames[i]+"allActions.txt";
            readAction = 3;
            //std::cout << readAction << std::endl;

        }
        if(sasoFilenames[i].find("_openAction.txt")!=std::string::npos)
        {
            //std::string simulationFileName = sasoFilenames[i]+"allActions.txt";
            readAction = 1;
             //std::cout << readAction << std::endl;

        }
        if(sasoFilenames[i].find("_closeAndPushAction.txt")!=std::string::npos)
        {
            //std::string simulationFileName = sasoFilenames[i]+"allActions.txt";
            readAction = 2;
             //std::cout << readAction << std::endl;

        }
        if(readAction > 0)
        {
            ans[sasoFilenames[i]] = getSimulationDataFromFile(object_id, sasoFilenames[i], readAction, false);
        }
        /*
        std::string simulationFileName = sasoFilenames[i]+"allActions.txt";
        ans[simulationFileName] = getSimulationDataFromFile(object_id, simulationFileName, 3, false);

        simulationFileName = sasoFilenames[i]+"openAction.txt";
        ans[simulationFileName] = getSimulationDataFromFile(object_id, simulationFileName, 1, false);
        //simulationDataFile.open("data/simulationData_1_openAction.txt");

        simulationFileName = sasoFilenames[i]+"closeAndPushAction.txt";
        ans[simulationFileName] = getSimulationDataFromFile(object_id, simulationFileName, 2, false);
        //simulationDataFile.open("data/simulationData_1_openAction.txt");
         */
    }
    for(int j = 0; j < A_PICK + 1; j++)
    {
      std::cout << "(" << j << "," << graspObjects[object_id]->simulationDataCollectionWithObject[j].size() << ")";
    }
    return ans;

}


bool RobotInterface::Step(GraspingStateRealArm& grasping_state, double random_num, int action, double& reward, GraspingObservation& grasping_obs, bool debug) const {
GraspingStateRealArm initial_grasping_state = grasping_state;

/*if(use_keras_models) //Used during belief update. Search uses the batch call
    {
    	std::vector<float> keras_input;
    	keras_input.resize(KerasInputVectorSize());
    	grasping_state.get_keras_input(keras_input);
    	std::vector<tensorflow::Tensor> outputs;
    	std::vector<float> random_num_vector;
    	if(action != A_PICK)
    	{
    		std::default_random_engine generator(random_num);
    		random_num_vector.push_back(Gaussian_Distribution(generator,0, 1 ));
    		random_num_vector.push_back(Gaussian_Distribution(generator,0, 1 ));
    	}
    	else
    	{
    		random_num_vector.push_back((float)random_num);
    		std::cout << "Random_num is " << random_num << std::endl;
    	}
    	StepKerasParticles(keras_input,action, random_num_vector,outputs);
    	auto terminal_vector = outputs[1].flat<float>().data();
    	bool ans = terminal_vector[0] >= 1;
		reward = (outputs[2].flat<float>().data())[0];
		auto stepped_particle_batch = outputs[0].flat<float>().data();
		auto obs_vector = outputs[3].flat<float>().data();
		if(grasping_obs.keras_observation.size() == 0)
		{
			grasping_obs.keras_observation.resize(KerasObservationVectorSize());
		}
		std::copy(obs_vector,obs_vector+ KerasObservationVectorSize(),grasping_obs.keras_observation.begin() );
		//Keras input vector will not be of zero size because it contained intial state vector
		std::copy(stepped_particle_batch, stepped_particle_batch + KerasInputVectorSize(), grasping_state.keras_input_vector.begin());
		//std::cout << "Terminal for action " << action << " " << terminal_vector[0] << " " <<  ans << std::endl;
		return ans;

    }
*/

//debug = true;
    //PrintState(grasping_state, std::cout);
    // PrintAction(action);
   // GraspingStateRealArm nearest_grasping_state = GetNearestGraspingStates(grasping_state,0);
    //Get next state

    //Check if gripper is closed using finger joint values
    int gripper_status = GetGripperStatus(grasping_state);

    //Get next state and observation
    if(action < A_CLOSE)
    {
        if(gripper_status == 0) //gripper is open
        {
            GetNextStateAndObsFromData(grasping_state, grasping_state, grasping_obs, random_num, action, debug);

        }
        else
        {
            //State remains same
            //Giving action as A_CLOSE as that was the last action which resulted in this state
            GetObsFromData(grasping_state, grasping_obs, random_num, A_CLOSE, debug);
        }
    }
    else if (action == A_CLOSE) //Close gripper
    {
        if(gripper_status == 0) //gripper is open
        {
            GetNextStateAndObsFromData(grasping_state, grasping_state, grasping_obs, random_num, action, debug);
            grasping_state.closeCalled = true;
        }
        else
        {
            //State remains same
            GetObsFromData(grasping_state, grasping_obs, random_num, action, debug);
        }
    }
    else if (action == A_OPEN) // Open gripper
    {
        if(gripper_status > 0) // gripper is closed
        {
           GetNextStateAndObsFromData(grasping_state, grasping_state, grasping_obs, random_num, action, debug);
           grasping_state.closeCalled = false;
           int new_gripper_status = GetGripperStatus(grasping_state);
           while(new_gripper_status > 0)
           {
               if(debug) {
                std::cout << "Moving gripper back by 1 cm to let it open" << std::endl;
               }
               if (grasping_state.gripper_pose.pose.position.x < min_x_i + 0.005)
                {
                    grasping_state.gripper_pose.pose.position.x = min_x_i - 0.01;
                    if(debug) {
                    std::cout << "Moving to an invalid state as gripper cannot move further back" << std::endl;
                    }
                    break;
                }
               grasping_state.gripper_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x - 0.01;
               GetNextStateAndObsFromData(grasping_state, grasping_state, grasping_obs, random_num, action, debug);
               new_gripper_status = GetGripperStatus(grasping_state);
           }
        }
        else
        {
           //State remains same.
            //Cannot get observation from data for open action as observation is for open action after close action. Gripper might not open correctly and give wrong observation
            //Using dummy action A_INCREASE_X to get observation as the gripper will be open in this action
            GetObsFromData(grasping_state, grasping_obs, random_num, A_INCREASE_X, debug);

        }
    }
    else if (action == A_PICK) // Pick object
    {
        if(gripper_status > 0)
        {
            GetNextStateAndObsFromData(grasping_state, grasping_state, grasping_obs,  random_num, action, debug);
        }
        else
        {
            GetNextStateAndObsUsingDefaulFunction(grasping_state, grasping_obs, action, random_num, debug);
        }
       /* grasping_state.gripper_pose.pose.position.z = grasping_state.gripper_pose.pose.position.z + pick_z_diff;
        grasping_state.gripper_pose.pose.position.x =  pick_x_val;
        grasping_state.gripper_pose.pose.position.y =  pick_y_val;

        if(gripper_status == 2) //Object is inside gripper and gripper is closed
        {
            //Change position of object too
            //Currently pick always succeeds
            //TODO : Gather cases when pick fails even when object is grasped
            //and compare current state against those cases to decide if pick will succeed
            grasping_state.object_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x + 0.03;
            grasping_state.object_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y;
            grasping_state.object_pose.pose.position.z = grasping_state.gripper_pose.pose.position.z;
        }

        GetObsFromData(grasping_state, grasping_obs, random_num, action);
        */
    }
    else
    {
        std::cout << "Invalid Action " << action << std::endl;
        assert(false);
    }

    //PrintState(grasping_state, std::cout);
   //PrintObs(grasping_obs, std::cout);


    bool validState = IsValidState(grasping_state);


    //Decide Reward
    GetReward(initial_grasping_state, grasping_state, grasping_obs, action, reward);

   // std::cout << "Reward " << reward << std::endl;

    //Update next state parameters dependent on previous state
    UpdateNextStateValuesBasedAfterStep(grasping_state,grasping_obs,reward, action);
     if(action == A_PICK || !validState) //Wither pick called or invalid state reached
    {
        return true;
    }
    return false;
}


double RobotInterface::ObsProb(GraspingObservation grasping_obs, const GraspingStateRealArm& grasping_state, int action) const {
        GraspingObservation grasping_obs_expected;

        /*if(use_keras_models) //Used in particle filter, Search calls the batch function
        {
        	std::vector<float> keras_input;
			keras_input.resize(KerasInputVectorSize());
			grasping_state.copy_keras_input(keras_input); //Cannot use get because it is not const function
			std::vector<tensorflow::Tensor> outputs;
			std::vector<float> random_num_vector;
			std::vector<float> obs_vector = grasping_obs.keras_observation;
        	GetObservationProbability(obs_vector, keras_input, action,
        				random_num_vector, outputs);
        	double prob = (double)((outputs[0].flat<float>().data())[0]);
        	return prob;
        }*/
    GetObsFromData(grasping_state, grasping_obs_expected, despot::Random::RANDOM.NextDouble(), action);

   // PrintObs(grasping_obs_expected);
    double total_distance = 0;
    double finger_weight = 1;
    double sensor_weight = 4;
    double gripper_position_weight = 2;
    double gripper_orientation_weight = 0;
    double vision_sensor_weight = 2;
    double image_pca_weight = 2;
    if(action == A_CLOSE)
    {
        finger_weight = 2;
        vision_sensor_weight = 1;
    }


    double tau = finger_weight + sensor_weight + gripper_position_weight + gripper_orientation_weight;
    if(RobotInterface::version7 || RobotInterface::version8)
    {
        tau = tau + vision_sensor_weight;
    }
    if(RobotInterface::version8)
    {
    	tau = tau + image_pca_weight;
    }
    double pca_distance = 0;
    if(RobotInterface::version8)
    {
    	for(int i = 0; i < grasping_obs.image_pca_values.size(); i++)
    	{
    		pca_distance = pca_distance + (grasping_obs.image_pca_values[i]- grasping_obs_expected.image_pca_values[i]);
    	}
    	pca_distance = pow(pca_distance, 0.5);
    }
    double finger_distance = 0;
    int gripper_status, gripper_status_expected;
    if(RobotInterface::use_discrete_observation_in_update)
    {
        bool closeCalled = action==A_CLOSE;
        gripper_status = GetGripperStatus(grasping_obs.finger_joint_state, closeCalled);
        gripper_status_expected = GetGripperStatus(grasping_obs_expected.finger_joint_state, closeCalled);
    }
    else
    {
        gripper_status = 2;
        gripper_status_expected = 2;
    }
    if(gripper_status !=gripper_status_expected)
    {
        finger_distance = 2;
    }
    else
    {
        if(gripper_status == 2)
        {
            int inc = 1;
            if (version5 || version6 || version7 || version8)
            {
                inc = 2;
            }
            for(int i = 0; i < 4; i=i+inc)
            {
                finger_distance = finger_distance + RobotInterface::abs(grasping_obs.finger_joint_state[i] - grasping_obs_expected.finger_joint_state[i]);
            }
            finger_distance = finger_distance/(4.0/inc);
        }
        else
        {
            finger_distance = 0;
        }
    }

    double sensor_distance = 0;
    int on_bits2[2];
    int on_bits2_expected[2];

    if (RobotInterface::use_binary_touch || RobotInterface::use_discrete_observation_in_update)
    {
       CheckTouch(grasping_obs.touch_sensor_reading, on_bits2);
        CheckTouch(grasping_obs_expected.touch_sensor_reading, on_bits2_expected);
    }
    for(int i = 0; i < 2; i++)
    {
        double distance_1 = grasping_obs.touch_sensor_reading[i] - grasping_obs_expected.touch_sensor_reading[i];
        if (RobotInterface::use_binary_touch || RobotInterface::use_discrete_observation_in_update)
        {
            distance_1 = on_bits2[i] - on_bits2_expected[i];
        }
        sensor_distance = sensor_distance + RobotInterface::abs(distance_1);
    }
    sensor_distance = sensor_distance/2.0;
    double vision_sensor_distance = 0;
    if(RobotInterface::version7 || RobotInterface::version8)
    {
        vision_sensor_distance = RobotInterface::abs(grasping_obs.vision_movement - grasping_obs_expected.vision_movement);
        if (grasping_obs.vision_movement == 2 || grasping_obs_expected.vision_movement == 2)
        {
            vision_sensor_distance = 0.5;
        }
    }

    double gripper_distance = 0;
    gripper_distance = gripper_distance + pow(grasping_obs.gripper_pose.pose.position.x - grasping_obs_expected.gripper_pose.pose.position.x, 2);
    gripper_distance = gripper_distance + pow(grasping_obs.gripper_pose.pose.position.y - grasping_obs_expected.gripper_pose.pose.position.y, 2);
    //gripper_distance = gripper_distance + pow(grasping_obs.gripper_pose.pose.position.z - grasping_obs_expected.gripper_pose.pose.position.z, 2);
    gripper_distance = pow(gripper_distance, 0.5);
    if(RobotInterface::use_discrete_observation_in_update)
    {
        if(gripper_distance > 0.015)
        {
            gripper_distance = 2;
        }
        if(gripper_distance < 0.005)
        {
            gripper_distance = 0;
        }
    }

    double gripper_quaternion_distance;
    if(gripper_orientation_weight > 0)
    {
        double gripper_quaternion_distance = 1 - pow(((grasping_obs.gripper_pose.pose.orientation.x*grasping_obs_expected.gripper_pose.pose.orientation.x)+
                                                  (grasping_obs.gripper_pose.pose.orientation.y*grasping_obs_expected.gripper_pose.pose.orientation.y)+
                                                  (grasping_obs.gripper_pose.pose.orientation.z*grasping_obs_expected.gripper_pose.pose.orientation.z)+
                                                  (grasping_obs.gripper_pose.pose.orientation.w*grasping_obs_expected.gripper_pose.pose.orientation.w)), 2);
    }
    total_distance = (finger_distance*finger_weight) +
                     (sensor_distance*sensor_weight) +
                     (gripper_distance*gripper_position_weight) +
                     (gripper_quaternion_distance*gripper_orientation_weight)+
                      (vision_sensor_weight*vision_sensor_distance) +
                      (image_pca_weight*pca_distance);
    double temperature = 5;
    double prob = pow(2, -1*temperature*total_distance/tau);

    return prob;
}



void RobotInterface::PrintAction(int action, std::ostream& out) const {
    out << "Action is ";
    if (action == A_CLOSE) {
        out << "CLOSE GRIPPER";
    } else if (action == A_OPEN) {
        out << "OPEN GRIPPER";
    }else if (action == A_PICK) {
        out << "PICK";
    }
    else if (action >= A_DECREASE_Y) {
        out << "DECREASE Y by " << get_action_range(action, A_DECREASE_Y);
    } else if (action >= A_INCREASE_Y) {
        out << "INCREASE Y by " << get_action_range(action, A_INCREASE_Y);
    } else if (action >= A_DECREASE_X) {
        out << "DECREASE X by " << get_action_range(action, A_DECREASE_X);
    } else if (action >= A_INCREASE_X) {
        out << "INCREASE X by " << get_action_range(action, A_INCREASE_X);
    }


    out << std::endl;
}

void RobotInterface::PrintObs(const GraspingStateRealArm& state, GraspingObservation& obs, std::ostream& out) const {
    PrintObs(obs,out);
    PrintState(state, out);
}

void RobotInterface::PrintObs(GraspingObservation& grasping_obs, std::ostream& out) const {
 char last_char = '*';
    if(&out == &std::cout)
    {
        last_char = '\n';
    }
    if(grasping_obs.keras_observation.size() > 0)
    {
    	for(int i = 0; i < grasping_obs.keras_observation.size(); i++)
    	{
    		out << grasping_obs.keras_observation[i] << " ";
    	}
    	if(RobotInterface::version8)
		{
		  out << "|"  ;
		  out << grasping_obs.rgb_image_name;
		}
    	out << last_char;
    	return;
    }

    out << grasping_obs.gripper_pose.pose.position.x << " " <<
                        grasping_obs.gripper_pose.pose.position.y << " " <<
                        grasping_obs.gripper_pose.pose.position.z << " " <<
                        grasping_obs.gripper_pose.pose.orientation.x << " " <<
                        grasping_obs.gripper_pose.pose.orientation.y << " " <<
                        grasping_obs.gripper_pose.pose.orientation.z << " " <<
                        grasping_obs.gripper_pose.pose.orientation.w << "|" <<
                        grasping_obs.mico_target_pose.pose.position.x << " " <<
                        grasping_obs.mico_target_pose.pose.position.y << " " <<
                        grasping_obs.mico_target_pose.pose.position.z << " " <<
                        grasping_obs.mico_target_pose.pose.orientation.x << " " <<
                        grasping_obs.mico_target_pose.pose.orientation.y << " " <<
                        grasping_obs.mico_target_pose.pose.orientation.z << " " <<
                        grasping_obs.mico_target_pose.pose.orientation.w << "|" ;
    for(int i = 0; i < 4; i++)
    {
        out << grasping_obs.finger_joint_state[i];
        if(i==3)
        {
            out<<"|";
        }
        else
        {
            out<<" ";
        }
    }
 //out<< std::endl;
  /*  for(int i = 0; i < 48; i++)
    {
        out << grasping_obs.force_values[i].x << " " <<
                grasping_obs.force_values[i].y << " " <<
                grasping_obs.force_values[i].z ;
        if(i==47)
        {
            out<<"|";
        }
        else
        {
            out<<" ";
        }
    }
 //out << std::endl;
 for(int i = 0; i < 48; i++)
    {
        out << grasping_obs.torque_values[i].x << " " <<
                grasping_obs.torque_values[i].y << " " <<
                grasping_obs.torque_values[i].z ;
        if(i==47)
        {
            out<<"*";
        }
        else
        {
            out<<" ";
        }
    }
 */
  for(int i = 0; i < 2; i++)
    {
        out << grasping_obs.touch_sensor_reading[i] ;
        if(i==1)
        {
            if(RobotInterface::version7 || RobotInterface::version8)
            {
                out<<"|";
            }
            else
            {
                out<<last_char;
            }
        }
        else
        {
            out<<" ";
        }
    }
     if(RobotInterface::version7 || RobotInterface::version8)
     {
        out << grasping_obs.vision_movement ;
        if(RobotInterface::version8)
        {
          out << "|"  ;
          out << grasping_obs.rgb_image_name;
          out << "|"  ;
          for(int i = 0; i < grasping_obs.image_pca_values.size(); i++)
          {
        	  out << grasping_obs.image_pca_values[i] << " ";
          }
        }
        
        
        out << last_char ;
       
     }
}

void RobotInterface::PrintState(const GraspingStateRealArm& grasping_state, std::ostream& out) const {
    char last_char = '*';
    if(&out == &std::cout)
    {
        last_char = '\n';
    }

    if(grasping_state.keras_input_vector.size() > 0)
        {
        	for(int i = 0; i < grasping_state.keras_input_vector.size(); i++)
        	{
        		out << grasping_state.keras_input_vector[i] << " ";
        	}
        	out << last_char;
        	return;
        }
    out << grasping_state.gripper_pose.pose.position.x << " " <<
                        grasping_state.gripper_pose.pose.position.y << " " <<
                        grasping_state.gripper_pose.pose.position.z << " " <<
                        grasping_state.gripper_pose.pose.orientation.x << " " <<
                        grasping_state.gripper_pose.pose.orientation.y << " " <<
                        grasping_state.gripper_pose.pose.orientation.z << " " <<
                        grasping_state.gripper_pose.pose.orientation.w << "|" <<
                        grasping_state.object_pose.pose.position.x << " " <<
                        grasping_state.object_pose.pose.position.y << " " <<
                        grasping_state.object_pose.pose.position.z << " " <<
                        grasping_state.object_pose.pose.orientation.x << " " <<
                        grasping_state.object_pose.pose.orientation.y << " " <<
                        grasping_state.object_pose.pose.orientation.z << " " <<
                        grasping_state.object_pose.pose.orientation.w << "|" ;
    for(int i = 0; i < 4; i++)
    {
        out << grasping_state.finger_joint_state[i];
        if(i==3)
        {
            out<<last_char;
        }
        else
        {
            out<<" ";
        }
    }



   // out << "Object id:"<< grasping_state.object_id ;
    //out << " x y z change (" << grasping_state.x_change << "," << grasping_state.y_change << "," << grasping_state.z_change << ") " ;
    //out << "Object Pose" << grasping_state.object_pose.pose.position.x;
    //out << " Gripper Pose" << grasping_state.gripper_pose.pose.position.x;
    //out << std::endl;
}

void RobotInterface::GenerateGaussianParticleFromState(GraspingStateRealArm& initial_state, std::string type) const {

    while(true)
    {
                // the engine for generator samples from a distribution
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            initial_state.object_pose.pose.position.x = Gaussian_Distribution(generator,graspObjects[initial_state.object_id]->initial_object_x, 0.03 );
            initial_state.object_pose.pose.position.y = Gaussian_Distribution(generator,graspObjects[initial_state.object_id]->initial_object_y, 0.03 );


            break;
            //Removing redundant code as this is not called after break
            /*
            if((initial_state.object_pose.pose.position.x < initial_object_x + 0.035) &&
                 (initial_state.object_pose.pose.position.x > initial_object_x - 0.035) &&
                 (initial_state.object_pose.pose.position.y < initial_object_y + 0.035) &&
                 (initial_state.object_pose.pose.position.y > initial_object_y - 0.035))
            {
                break;
            }
            */
      }

}

void RobotInterface::GenerateGaussianParticleFromState_V8(GraspingStateRealArm& initial_state, std::string type) const {


                // the engine for generator samples from a distribution
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            initial_state.object_pose.pose.position.x = Gaussian_Distribution(generator,graspObjects[initial_state.object_id]->initial_object_x, 0.02 );
            initial_state.object_pose.pose.position.y = Gaussian_Distribution(generator,graspObjects[initial_state.object_id]->initial_object_y, 0.02 );

            //Using uniform distribution for angle
            double angle_add = Uniform_Distribution(generator,0, 2*3.14);

			Quaternion q(graspObjects[initial_state.object_id]->initial_object_pose_xx,
						 graspObjects[initial_state.object_id]->initial_object_pose_yy,
						 graspObjects[initial_state.object_id]->initial_object_pose_zz,
						 graspObjects[initial_state.object_id]->initial_object_pose_w);
			double roll, pitch,yaw;
			Quaternion::toEulerAngle(q,roll, pitch,yaw);
			double new_yaw = yaw + angle_add;
			Quaternion::toQuaternion(new_yaw,pitch,roll,initial_state.object_pose.pose.orientation.x,
					initial_state.object_pose.pose.orientation.y,
											 initial_state.object_pose.pose.orientation.z,
											 initial_state.object_pose.pose.orientation.w);


}

void RobotInterface::GenerateUniformParticleFromState(GraspingStateRealArm& initial_state, std::string type) const {
// the engine for generator samples from a distribution
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            initial_state.object_pose.pose.position.x = initial_state.object_pose.pose.position.x + Uniform_Distribution(generator,-0.04, 0.04);
            initial_state.object_pose.pose.position.y = initial_state.object_pose.pose.position.y + Uniform_Distribution(generator,-0.04, 0.04 );

}

void RobotInterface::GenerateUniformParticleFromState_V8(GraspingStateRealArm& initial_state, std::string type) const {
// the engine for generator samples from a distribution
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine generator(seed);
            initial_state.object_pose.pose.position.x = initial_state.object_pose.pose.position.x + Uniform_Distribution(generator,-0.02, 0.02);
            initial_state.object_pose.pose.position.y = initial_state.object_pose.pose.position.y + Uniform_Distribution(generator,-0.02, 0.02 );

            double angle_add = Uniform_Distribution(generator,0, 2*3.14);

			Quaternion q(initial_state.object_pose.pose.orientation.x,
						 initial_state.object_pose.pose.orientation.y,
						 initial_state.object_pose.pose.orientation.z,
						 initial_state.object_pose.pose.orientation.w);
			double roll, pitch,yaw;
			Quaternion::toEulerAngle(q,roll, pitch,yaw);
			double new_yaw = yaw + angle_add;
			Quaternion::toQuaternion(new_yaw,pitch,roll,initial_state.object_pose.pose.orientation.x,
					initial_state.object_pose.pose.orientation.y,
											 initial_state.object_pose.pose.orientation.z,
											 initial_state.object_pose.pose.orientation.w);
}


void RobotInterface::GetDefaultStartState(GraspingStateRealArm& initial_state) const {
    int i = initial_gripper_pose_index_x;
    int j = initial_gripper_pose_index_y;
    int object_id = initial_state.object_id;

    initial_state.gripper_pose.pose.position.x = min_x_i + 0.01*i;
    initial_state.gripper_pose.pose.position.y = min_y_i + 0.01*j;
    initial_state.gripper_pose.pose.position.z = initial_gripper_pose_z;
    initial_state.gripper_pose.pose.orientation.x = initial_gripper_pose_xx;
    initial_state.gripper_pose.pose.orientation.y = initial_gripper_pose_yy;
    initial_state.gripper_pose.pose.orientation.z = initial_gripper_pose_zz;
    initial_state.gripper_pose.pose.orientation.w = initial_gripper_pose_ww;
    initial_state.object_pose.pose.position.x = graspObjects[object_id]->initial_object_x;
    initial_state.object_pose.pose.position.y = graspObjects[object_id]->initial_object_y;
    initial_state.object_pose.pose.position.z = graspObjects[object_id]->initial_object_pose_z;
    initial_state.object_pose.pose.orientation.x = graspObjects[object_id]->initial_object_pose_xx;//-0.0327037 ;
    initial_state.object_pose.pose.orientation.y = graspObjects[object_id]->initial_object_pose_yy; //0.0315227;
    initial_state.object_pose.pose.orientation.z = graspObjects[object_id]->initial_object_pose_zz; //-0.712671 ;
    initial_state.object_pose.pose.orientation.w = graspObjects[object_id]->initial_object_pose_w; //0.700027;
    initial_state.finger_joint_state[0] = -2.95639e-05 ;
    initial_state.finger_joint_state[1] = 0.00142145;
    initial_state.finger_joint_state[2] = -1.19209e-06 ;
    initial_state.finger_joint_state[3] = -0.00118446 ;
}

std::pair <std::map<int,double>,std::vector<double> > RobotInterface::GetBeliefObjectProbability(std::vector<int> belief_object_ids) const {
    std::map<int,double> ans ;
    std::vector<double> ans1 ;
    for(int i = 0; i < belief_object_ids.size(); i++)
    {
        ans[belief_object_ids[i]] = 1.0;
    }
    return std::make_pair(ans,ans1);
}

int RobotInterface::GetWeightedObservationSize() const{

    int weighted_obs_size = 7;
    if(classifier_string_name.find("kmeans") != std::string::npos)
    {
        weighted_obs_size = 3;
        if(classifier_string_name.find("label_2") != std::string::npos)
        {
            weighted_obs_size = 8;
        }
        if(classifier_string_name.find("label_3") != std::string::npos)
        {
            weighted_obs_size = 5;
        }
        if(classifier_string_name.find("label_4") != std::string::npos)
        {
            weighted_obs_size = 4;
        }

    }
    return weighted_obs_size;

}

void RobotInterface::SyncParticleState(GraspingStateRealArm& grasping_state, GraspingObservation grasping_obs) {
    //grasping_state.gripper_pose.pose.orientation.x = grasping_obs.gripper_pose.pose.orientation.x;
    //grasping_state.gripper_pose.pose.orientation.y = grasping_obs.gripper_pose.pose.orientation.y;
    //grasping_state.gripper_pose.pose.orientation.z = grasping_obs.gripper_pose.pose.orientation.z;
    //grasping_state.gripper_pose.pose.orientation.w = grasping_obs.gripper_pose.pose.orientation.w;

    //grasping_state.gripper_pose.pose.position.x = grasping_obs.gripper_pose.pose.position.x;
    //grasping_state.gripper_pose.pose.position.y = grasping_obs.gripper_pose.pose.position.y;
    //grasping_state.gripper_pose.pose.position.z = grasping_obs.gripper_pose.pose.position.z;
    /*
    for(int i = 0; i < 4; i++)
    {
        grasping_state.finger_joint_state[i] = grasping_obs.finger_joint_state[i];
    }*/

}



double RobotInterface::get_action_range(int action, int action_type) const {
if ((action - action_type) >= (A_DECREASE_X - A_INCREASE_X)) {
        std::cout << "Action " << action << "out of range of action type " << action_type << std::endl;
        assert(false);
    }
    return epsilon * pow(epsilon_multiplier, action - action_type);
}

bool sort_functor(double x1, double y1, double x2, double y2)
{
     if(x1<x2)
    {
        return true;
    }
    else if(x1 == x2)
    {
        return y1<y2;
    }
    else
    {
        return false;
    }
}

bool sort_by_object_relative_pose_initial_state_x (SimulationData data1,SimulationData data2) {
    double x1 = data1.current_object_pose.pose.position.x - data1.current_gripper_pose.pose.position.x;
    //double y1 = data1.current_object_pose.pose.position.y - data1.current_gripper_pose.pose.position.y;
    double x2 = data2.current_object_pose.pose.position.x - data2.current_gripper_pose.pose.position.x;
    //double y2 = data2.current_object_pose.pose.position.y - data2.current_gripper_pose.pose.position.y;


    return x1 < x2;
}


bool sort_by_object_relative_pose_initial_state_y (SimulationData data1,SimulationData data2) {
    //double x1 = data1.current_object_pose.pose.position.x - data1.current_gripper_pose.pose.position.x;
    double y1 = data1.current_object_pose.pose.position.y - data1.current_gripper_pose.pose.position.y;
    //double x2 = data2.current_object_pose.pose.position.x - data2.current_gripper_pose.pose.position.x;
    double y2 = data2.current_object_pose.pose.position.y - data2.current_gripper_pose.pose.position.y;


    return y1 < y2;
}

bool sort_by_gripper_pose_initial_state_x (SimulationData data1,SimulationData data2) {
    double x1 = data1.current_gripper_pose.pose.position.x;
    double y1 = data1.current_gripper_pose.pose.position.y;
    double x2 = data2.current_gripper_pose.pose.position.x;
    double y2 = data2.current_gripper_pose.pose.position.y;


    return x1 < x2;
}

bool sort_by_gripper_pose_initial_state_y (SimulationData data1,SimulationData data2) {
    double x1 = data1.current_gripper_pose.pose.position.x;
    double y1 = data1.current_gripper_pose.pose.position.y;
    double x2 = data2.current_gripper_pose.pose.position.x;
    double y2 = data2.current_gripper_pose.pose.position.y;


    return y1 < y2;
}

bool sort_by_object_relative_pose_next_state_x (SimulationData data1,SimulationData data2) {
    double x1 = data1.next_object_pose.pose.position.x - data1.next_gripper_pose.pose.position.x;
    //double y1 = data1.current_object_pose.pose.position.y - data1.current_gripper_pose.pose.position.y;
    double x2 = data2.next_object_pose.pose.position.x - data2.next_gripper_pose.pose.position.x;
    //double y2 = data2.current_object_pose.pose.position.y - data2.current_gripper_pose.pose.position.y;


    return x1 < x2;
}

bool sort_by_object_relative_pose_next_state_y (SimulationData data1,SimulationData data2) {
    //double x1 = data1.current_object_pose.pose.position.x - data1.current_gripper_pose.pose.position.x;
    double y1 = data1.next_object_pose.pose.position.y - data1.next_gripper_pose.pose.position.y;
    //double x2 = data2.current_object_pose.pose.position.x - data2.current_gripper_pose.pose.position.x;
    double y2 = data2.next_object_pose.pose.position.y - data2.next_gripper_pose.pose.position.y;


    return y1 < y2;
}

bool sort_by_gripper_pose_next_state_x (SimulationData data1,SimulationData data2) {
    double x1 = data1.next_gripper_pose.pose.position.x;
    double y1 = data1.next_gripper_pose.pose.position.y;
    double x2 = data2.next_gripper_pose.pose.position.x;
    double y2 = data2.next_gripper_pose.pose.position.y;


    return x1 < x2;
}

bool sort_by_gripper_pose_next_state_y (SimulationData data1,SimulationData data2) {
    double x1 = data1.next_gripper_pose.pose.position.x;
    double y1 = data1.next_gripper_pose.pose.position.y;
    double x2 = data2.next_gripper_pose.pose.position.x;
    double y2 = data2.next_gripper_pose.pose.position.y;


    return y1 < y2;
}


void RobotInterface::GetObsFromDynamicModel(GraspingStateRealArm grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug) const {

    grasping_obs.getObsFromState(grasping_state);

}

void RobotInterface::GetObsFromData(GraspingStateRealArm current_grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug) const {
    if(action != A_PICK && IsValidState(current_grasping_state)) //State is not terminal
    {
        if(use_regression_models)
        {
            GetObsFromDynamicModel(current_grasping_state, grasping_obs, random_num, action, debug);
            return;
        }

        int gripper_status = GetGripperStatus(current_grasping_state);
        if(gripper_status > 0)
        {
            action = A_CLOSE;
        }

        bool stateInObjectData = false;
        bool stateInGripperData = false;

        int object_id = current_grasping_state.object_id;
        SimulationData tempData;
        std::vector<SimulationData> tempDataVector;
        std::vector<SimulationData> :: iterator x_lower_bound, x_upper_bound, xy_lower_bound, xy_upper_bound;
        //tempData.next_gripper_pose = current_grasping_state.gripper_pose;
        //tempData.next_object_pose = current_grasping_state.object_pose;

        //sort(simulationDataCollectionWithObject[action].begin(), simulationDataCollectionWithObject[action].end(), sort_by_object_relative_pose_next_state_x);
        //tempData.next_object_pose.pose.position.x = tempData.next_object_pose.pose.position.x - 0.005;
        //x_lower_bound = lower_bound(simulationDataCollectionWithObject[action].begin(), simulationDataCollectionWithObject[action].end(), tempData, sort_by_object_relative_pose_next_state_x);
        //tempData.next_object_pose.pose.position.x = tempData.next_object_pose.pose.position.x + 0.01;
        //x_upper_bound = upper_bound(simulationDataCollectionWithObject[action].begin(), simulationDataCollectionWithObject[action].end(), tempData, sort_by_object_relative_pose_next_state_x);

        //sort(x_lower_bound, x_upper_bound, sort_by_object_relative_pose_next_state_y);
        //tempData.next_object_pose.pose.position.y = tempData.next_object_pose.pose.position.y - 0.005;
        //xy_lower_bound = lower_bound(x_lower_bound, x_upper_bound, tempData, sort_by_object_relative_pose_next_state_y);
        //tempData.next_object_pose.pose.position.y = tempData.next_object_pose.pose.position.y + 0.01;
        //xy_upper_bound = upper_bound(x_lower_bound, x_upper_bound, tempData, sort_by_object_relative_pose_next_state_y);


        if(use_discretized_data)
        {
        	if(RobotInterface::version8)
			{
				graspObjects[object_id]->getSimulationData(current_grasping_state.object_pose, current_grasping_state.gripper_pose, current_grasping_state.get_theta_z_degree(), tempDataVector, action, true);
			}
			else
			{
				tempDataVector = graspObjects[object_id]->getSimulationData(current_grasping_state.object_pose, current_grasping_state.gripper_pose, action, true);
			}
            //tempDataVector = graspObjects[object_id]->getSimulationData(current_grasping_state.object_pose, current_grasping_state.gripper_pose, action, true);
            if(debug)
                {
                    std::cout << "Matched obs particles" << std::endl;
                    for(int i = 0;i < tempDataVector.size(); i++)
                    {
                        tempDataVector[i].PrintSimulationData();
                    }
                        std::cout << "------" << std::endl;
                }

        }
        else
        {
            int len_simulation_data = ((std::vector<SimulationData>)(graspObjects[object_id]->simulationDataCollectionWithObject[action])).size();

            for(int i = 0; i < len_simulation_data; i++)
            {
                double x1 = graspObjects[object_id]->simulationDataCollectionWithObject[action][i].next_object_pose.pose.position.x - graspObjects[object_id]->simulationDataCollectionWithObject[action][i].next_gripper_pose.pose.position.x;
                double x2 = current_grasping_state.object_pose.pose.position.x - current_grasping_state.gripper_pose.pose.position.x;
                if(RobotInterface::abs(x1-x2) <= 0.005)
                {
                    double y1 = graspObjects[object_id]->simulationDataCollectionWithObject[action][i].next_object_pose.pose.position.y - graspObjects[object_id]->simulationDataCollectionWithObject[action][i].next_gripper_pose.pose.position.y;
                    double y2 = current_grasping_state.object_pose.pose.position.y - current_grasping_state.gripper_pose.pose.position.y;
                    if(RobotInterface::abs(y1-y2) <= 0.005)
                    {
                        tempDataVector.push_back(graspObjects[object_id]->simulationDataCollectionWithObject[action][i]);
                    }
                }

            }
        }
        xy_lower_bound = tempDataVector.begin();
        xy_upper_bound = tempDataVector.end();
        // check if x and y exists in simulation data with object
        if(std::distance(xy_lower_bound,xy_upper_bound) > 0)
        {   stateInObjectData = true;

            //Get the closest gripper state
            double min_distance = 100000;

            for(std::vector<SimulationData> :: iterator it = xy_lower_bound; it < xy_upper_bound; it++)
            {
                double temp_distance = 0;
                //Match only gripper pose as relative pose is already matched
                //Match relative pose for observaton, as no further action needs to be taken
                //Matching only pose leads to loer observation prob. for same particle
                double x1 = (*it).next_gripper_pose.pose.position.x - (*it).next_object_pose.pose.position.x;
                double x2 = current_grasping_state.gripper_pose.pose.position.x - current_grasping_state.object_pose.pose.position.x;
                temp_distance = temp_distance + pow(x1 - x2, 2);
                if(temp_distance < min_distance)
                {
                    double y1 = (*it).next_gripper_pose.pose.position.y - (*it).next_object_pose.pose.position.y;
                    double y2 = current_grasping_state.gripper_pose.pose.position.y - current_grasping_state.object_pose.pose.position.y;
                    temp_distance = temp_distance + pow(y1 - y2, 2);
                }
                //Not taking sqaure root to save time
                //temp_distance = pow(temp_distance, 0.5);
                if(temp_distance < min_distance)
                {
                    min_distance = temp_distance;
                    tempData = (*it);
                }
            }


        }
        /*
        else
        {
            sort(simulationDataCollectionWithoutObject[action].begin(), simulationDataCollectionWithoutObject[action].end(), sort_by_gripper_pose_next_state_x);
            tempData.next_gripper_pose.pose.position.x = tempData.next_gripper_pose.pose.position.x - 0.005;
            x_lower_bound = lower_bound(simulationDataCollectionWithoutObject[action].begin(), simulationDataCollectionWithoutObject[action].end(), tempData, sort_by_gripper_pose_next_state_x);
            tempData.next_gripper_pose.pose.position.x = tempData.next_gripper_pose.pose.position.x + 0.01;
            x_upper_bound = upper_bound(simulationDataCollectionWithoutObject[action].begin(), simulationDataCollectionWithoutObject[action].end(), tempData, sort_by_gripper_pose_next_state_x);

            sort(x_lower_bound, x_upper_bound, sort_by_gripper_pose_initial_state_x);
            tempData.next_gripper_pose.pose.position.y = tempData.next_gripper_pose.pose.position.y - 0.005;
            xy_lower_bound = lower_bound(x_lower_bound, x_upper_bound, tempData, sort_by_gripper_pose_next_state_y);
            tempData.next_gripper_pose.pose.position.y = tempData.next_gripper_pose.pose.position.y + 0.01;
            xy_upper_bound = upper_bound(x_lower_bound, x_upper_bound, tempData, sort_by_gripper_pose_next_state_y);

            // check if x and y exists in simulation data without object
            if(std::distance(xy_lower_bound,xy_upper_bound) > 0)
            {   stateInGripperData = true;
                //Get the closest gripper state
                double min_distance = 100000;

                for(std::vector<SimulationData> :: iterator it = xy_lower_bound; it < xy_upper_bound; it++)
                {
                    double temp_distance = 0;
                    if(action < 8 || action > 15)
                    { // if move in x check distance between only x
                        temp_distance = pow(((*it).next_gripper_pose.pose.position.x - current_grasping_state.gripper_pose.pose.position.x), 2);
                    }
                    if (action >= 8)
                    {
                        // if move in y check distance between only y
                        temp_distance = temp_distance + pow(((*it).next_gripper_pose.pose.position.y - current_grasping_state.gripper_pose.pose.position.y), 2);
                    }

                    temp_distance = pow(temp_distance, 0.5);
                    if(temp_distance < min_distance)
                    {
                        min_distance = temp_distance;
                        tempData = (*it);
                    }
                }

            }
        }*/
        if(stateInObjectData || stateInGripperData)
        {

            if(debug)
            {
                std::cout << "Obs being updated from ";
                if (stateInObjectData){
                    std::cout << "object ";
                }
                if (stateInGripperData) {
                    std::cout << "gripper ";
                }
                std::cout << "data\n";
                tempData.PrintSimulationData();
            }
            grasping_obs.gripper_pose = current_grasping_state.gripper_pose;
            grasping_obs.mico_target_pose = tempData.mico_target_pose; //Does not matter for now
            for(int i = 0; i < 4; i++)
            {
                grasping_obs.finger_joint_state[i] = tempData.next_finger_joint_state[i];
            }
            for(int i = 0; i < 2; i++)
            {
                grasping_obs.touch_sensor_reading[i] = tempData.touch_sensor_reading[i];
            }
            grasping_obs.vision_movement = tempData.vision_movement;
            if(RobotInterface::version8)
            {
            	grasping_obs.image_pca_values = tempData.image_pca_components;
            }
            //ConvertObs48ToObs2(tempData.touch_sensor_reading, grasping_obs.touch_sensor_reading);

        }
        else
        {
            GetObsUsingDefaultFunction(current_grasping_state, grasping_obs, action);
        }

    }
    else
    {
        GetObsUsingDefaultFunction(current_grasping_state, grasping_obs, action);
    }
}

//Return value 0 if open //Determined by whther close was called or not
//Return value 1 if closed without object inside it
//Return value 2 if closed with object inside it
int RobotInterface::GetGripperStatus(GraspingStateRealArm grasping_state) const {
    return GetGripperStatus(grasping_state.finger_joint_state, grasping_state.closeCalled);
}

int RobotInterface::GetGripperStatus(double finger_joint_state[], bool closeCalled) const {
    double degree_readings[4];
    for(int i = 0; i < 4; i++)
    {
        degree_readings[i] = finger_joint_state[i]*180/3.14;
    }

    if(version5 || version6 || version7 || version8)
    {
        if(closeCalled)
        {
            if(degree_readings[0] > 59 && degree_readings[2] > 59)
            {
                return 1;
            }

            else
            // if(degree_readings[0] > 1 && degree_readings[2] > 1)
            {
                return 2;
            }
        }
        else
        {
            return 0;
        }

    }
    else
    {
        if(closeCalled)
        {
        if(degree_readings[0] > 22 && //Changed from 20 to 22 looking at the data from 7cm cylinder object
           degree_readings[1] > 85 &&
           degree_readings[2] > 22 && //Changed from 20 to 22 looking at the data from 7cm cylinder object
           degree_readings[3] > 85)
        {//joint1 > 20 joint2 > 85
            return 1;
        }

        //if(//degree_readings[0] > 2 &&
        //   degree_readings[1] > 25 && //Changed from 45 to 25 looking at data
        //   //degree_readings[2] > 2 &&
        //   degree_readings[3] > 25)  //Changed from 45 to 25 looking at data
        else
        {//joint1 > 2 joint2 > 45 return 2
            return 2;
        }
        }
        else
        {
        return 0;
        }
    }

}

void RobotInterface::UpdateNextStateValuesBasedAfterStep(GraspingStateRealArm& grasping_state, GraspingObservation grasping_obs, double reward, int action) const {
    grasping_state.pre_action = action;
    CheckTouch(grasping_obs.touch_sensor_reading, grasping_state.touch);
    grasping_state.touch_value[0] = grasping_obs.touch_sensor_reading[0];
    grasping_state.touch_value[1] = grasping_obs.touch_sensor_reading[1];
    grasping_state.vision_movement = grasping_obs.vision_movement;
    if(grasping_state.closeCalled)
    {
        if(action==A_OPEN)
        {
            grasping_state.closeCalled = false;
        }
    }
    else
    {
        if(action==A_CLOSE)
        {
            grasping_state.closeCalled = true;
        }
    }
    int gripper_status = GetGripperStatus(grasping_state);
    if (gripper_status == 2){
        if(reward < -1) //Pick leads to failure. So object most probably out of gripper
        {
            gripper_status = 1;
        }
    }
    grasping_state.gripper_status = gripper_status;

}

void RobotInterface::GetNextStateAndObsFromDynamicModel(GraspingStateRealArm current_grasping_state, GraspingStateRealArm& next_grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug) const {

    double arg_time_start = despot::get_time_second();
    std::vector<double> gs_vec = current_grasping_state.getStateAsVector();
    /*
    PyObject* x = PyTuple_New(gs_vec.size());
    for(int i = 0; i < gs_vec.size(); i++)
    {
        PyTuple_SetItem(x,i,PyFloat_FromDouble(gs_vec[i]));
    }

    PyObject* pArgs;
    if(action==A_PICK)
    {
        pArgs = PyTuple_New(3);
        PyTuple_SetItem(pArgs,2,PyInt_FromLong(1));
    }
    else
    {
        pArgs = PyTuple_New(2);
    }
    PyTuple_SetItem(pArgs, 0, dynamicModels[current_grasping_state.object_id][action]);
    Py_INCREF(dynamicModels[current_grasping_state.object_id][action]);
    PyTuple_SetItem(pArgs, 1, x);
    std::vector<double> ngs_vec;

    double call_time_start = despot::get_time_second();
    PyObject* y =  PyObject_CallObject(dynamicFunction, pArgs);
    double call_time_end = despot::get_time_second();
    */
    double call_time_start_c = despot::get_time_second();
    std::vector<double> ngs_vec_c = graspObjects[current_grasping_state.object_id]->dynamicModelsC[action]->predict(gs_vec, debug);
    double call_time_end_c = despot::get_time_second();
    /*
    int y_size = PyList_Size(y);

    for(int i = 0; i < y_size; i++)
    {
        ngs_vec.push_back(PyFloat_AsDouble(PyList_GetItem(y,i)));
    }
    std::cout << "[";
    for(int i = 0; i < ngs_vec.size(); i++)
    {
        std::cout<< ngs_vec[i] << "," ;
    }
    std::cout << "]" << std::endl;

    std::cout << "[";
    for(int i = 0; i < ngs_vec_c.size(); i++)
    {
        std::cout<< ngs_vec_c[i] << "," ;
    }
    std::cout << "]" << std::endl;
    */
    if(action==A_PICK)
    {
        assert(ngs_vec_c.size() == 2);
        double pick_success_prob = ngs_vec_c[1];
        /*if(pick_success_prob >= 0.5)
        {
            pick_success_prob = 1.0;
        }*/
      //  std::cout << "Random num is" << random_num << " pick success prob is " << pick_success_prob << std::endl;
        if(pick_success_prob >= random_num)
        {
            //pick success
            GetDefaultPickState(next_grasping_state,1.0,1);
        }
        else
        {
            //pick failure
            GetDefaultPickState(next_grasping_state,0.0,0);
        }


    }
    else
    {
        assert(ngs_vec_c.size() == num_predictions_for_dynamic_function);
        next_grasping_state.getStateFromVector(ngs_vec_c);

    }
    grasping_obs.getObsFromState(next_grasping_state);
    double func_end = despot::get_time_second();
    //std::cout << "Arg time: " << call_time_start_c - arg_time_start << std::endl;
    //std::cout << "Call time: " << call_time_end - call_time_start << std::endl;
    //std::cout << "Call time c: " << call_time_end_c - call_time_start_c << std::endl;
    //std::cout << "Other time: " << func_end - call_time_end_c << std::endl;




}

SimulationData RobotInterface::GetNextStateUsingNearestNeighbourSearch(std::vector<SimulationData> :: iterator xy_lower_bound,
        std::vector<SimulationData> :: iterator xy_upper_bound,
        GraspingStateRealArm current_grasping_state, int action, bool debug) const
{
    SimulationData tempData;
    //Get the closest gripper state
        double min_distance = 100000;

        for(std::vector<SimulationData> :: iterator it = xy_lower_bound; it < xy_upper_bound; it++)
        {
            double temp_distance = 0;
            if(action == A_PICK)
            {
                int inc = 1;
                if (version5 || version6 || version7 || version8)
                {
                    inc = 2;
                }
                for(int i = 0; i < 4; i=i+inc)
                {
                    if(temp_distance < min_distance)
                    {
                        temp_distance = temp_distance + pow(((*it).current_finger_joint_state[i] - current_grasping_state.finger_joint_state[i]), 2);
                    }
                }
            }
            else{
                //Match only current gripper pose as relative pose is already close for all the particles
                //Matching relative pose because a difference of 1 cm in relative pose can make a differnce
                //Should Match current pose to cater to boundary cases
                double x1,x2;
                if(action >= A_INCREASE_Y )
                {
                    x1 = (*it).current_gripper_pose.pose.position.x  - (*it).current_object_pose.pose.position.x;
                    x2 = current_grasping_state.gripper_pose.pose.position.x  - current_grasping_state.object_pose.pose.position.x;
                }
                else
                {
                    //Match current gripper pose for movement in x to cater to boundary cases
                   x1 = (*it).current_gripper_pose.pose.position.x ; // - (*it).current_object_pose.pose.position.x;
                   x2 = current_grasping_state.gripper_pose.pose.position.x ; // - current_grasping_state.object_pose.pose.position.x;

                }
                temp_distance = temp_distance + pow(x1 - x2, 2);
                if(temp_distance < min_distance)
                {
                    double y1,y2;
                    if(action < A_INCREASE_Y || action >=A_CLOSE)
                    {
                     y1 = (*it).current_gripper_pose.pose.position.y - (*it).current_object_pose.pose.position.y;
                     y2 = current_grasping_state.gripper_pose.pose.position.y - current_grasping_state.object_pose.pose.position.y;
                    }
                    else
                    {
                        //Match current gripper pose for movement in y to cater to boundary case
                        y1 = (*it).current_gripper_pose.pose.position.y; //- (*it).current_object_pose.pose.position.y;
                        y2 = current_grasping_state.gripper_pose.pose.position.y ; //- current_grasping_state.object_pose.pose.position.y;

                    }
                     temp_distance = temp_distance + pow(y1 - y2, 2);
                }
            }
            //Not taking square root to save time
            //temp_distance = pow(temp_distance, 0.5);

            if(temp_distance < min_distance)
            {
                min_distance = temp_distance;
                tempData = (*it);
            }
        }

        return tempData;
}




int RobotInterface::GetNextStateProbabilistically(
std::vector<SimulationData>::iterator xy_lower_bound,
        std::vector<SimulationData>::iterator xy_upper_bound,
        GraspingStateRealArm current_grasping_state, int action, double random_num, bool debug) const {

    //Get the closest gripper state
        double total_weight = 0;
        double temperature = 4;
        std::vector<double> weight_vector;
        for(std::vector<SimulationData> :: iterator it = xy_lower_bound; it < xy_upper_bound; it++)
        {
            double temp_distance = 0;
            double temp_weight;
            if(action == A_PICK)
            {
                int inc = 1;
                if (version5 || version6 || version7 || version8)
                {
                    inc = 2;
                }
                for(int i = 0; i < 4; i=i+inc)
                {

                        temp_distance = temp_distance + pow(((*it).current_finger_joint_state[i] - current_grasping_state.finger_joint_state[i]), 2);

                }
            }
            else{
                //Match only current gripper pose as relative pose is already close for all the particles
                //Matching relative pose because a difference of 1 cm in relative pose can make a differnce
                //Should Match current pose to cater to boundary cases
                double x1,x2;
                if(action >= A_INCREASE_Y )
                {
                    x1 = (*it).current_gripper_pose.pose.position.x  - (*it).current_object_pose.pose.position.x;
                    x2 = current_grasping_state.gripper_pose.pose.position.x  - current_grasping_state.object_pose.pose.position.x;
                }
                else
                {
                    //Match current gripper pose for movement in x to cater to boundary cases
                   x1 = (*it).current_gripper_pose.pose.position.x ; // - (*it).current_object_pose.pose.position.x;
                   x2 = current_grasping_state.gripper_pose.pose.position.x ; // - current_grasping_state.object_pose.pose.position.x;

                }
                temp_distance = temp_distance + pow(x1 - x2, 2);

                double y1,y2;
                if(action < A_INCREASE_Y || action >=A_CLOSE)
                {
                 y1 = (*it).current_gripper_pose.pose.position.y - (*it).current_object_pose.pose.position.y;
                 y2 = current_grasping_state.gripper_pose.pose.position.y - current_grasping_state.object_pose.pose.position.y;
                }
                else
                {
                    //Match current gripper pose for movement in y to cater to boundary case
                    y1 = (*it).current_gripper_pose.pose.position.y; //- (*it).current_object_pose.pose.position.y;
                    y2 = current_grasping_state.gripper_pose.pose.position.y ; //- current_grasping_state.object_pose.pose.position.y;

                }
                 temp_distance = temp_distance + pow(y1 - y2, 2);

            }

            temp_distance = pow(temp_distance, 0.5);
            temp_weight = pow(2,-1*temperature*temp_distance);
            total_weight = total_weight + temp_weight;
            weight_vector.push_back(temp_weight);

        }
        double mass = 0;
        double cur;
        for(int i = 0; i < weight_vector.size(); i++)
        {
            cur = weight_vector[i]/total_weight;
            if(random_num < (mass + cur))
            {
                return i;
            }
            else
            {
                mass = mass + cur;
            }
        }

}


void RobotInterface::GetNextStateAndObsFromData(GraspingStateRealArm current_grasping_state, GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, double random_num, int action, bool debug) const {
    if(use_regression_models)
    {
        GetNextStateAndObsFromDynamicModel(current_grasping_state, grasping_state, grasping_obs, random_num, action, debug);
        return;
    }
    //debug = true;
    bool stateInObjectData = false;
    bool stateInGripperData = false;
    int object_id = current_grasping_state.object_id;
    SimulationData tempData;
    std::vector<SimulationData> tempDataVector;
    std::vector<SimulationData> :: iterator x_lower_bound, x_upper_bound, xy_lower_bound, xy_upper_bound;
    //tempData.current_gripper_pose = current_grasping_state.gripper_pose;
    //tempData.current_object_pose = current_grasping_state.object_pose;
    double step_start_t1 = despot::get_time_second();
    if(use_discretized_data)
    {
    	if(RobotInterface::version8)
    	{
    		graspObjects[object_id]->getSimulationData(current_grasping_state.object_pose, current_grasping_state.gripper_pose, current_grasping_state.get_theta_z_degree(), tempDataVector, action, false);
    	}
    	else
    	{
    		tempDataVector = graspObjects[object_id]->getSimulationData(current_grasping_state.object_pose, current_grasping_state.gripper_pose, action, false);
    	}
        if(debug)
        {
            for(int i = 0;i < tempDataVector.size(); i++)
            {
                tempDataVector[i].PrintSimulationData();
            }
        }
    }
    else
    {
        for(int i = 0; i < graspObjects[object_id]->simulationDataCollectionWithObject[action].size(); i++)
        {
            double x1 = graspObjects[object_id]->simulationDataCollectionWithObject[action][i].current_object_pose.pose.position.x - graspObjects[object_id]->simulationDataCollectionWithObject[action][i].current_gripper_pose.pose.position.x;
            double x2 = current_grasping_state.object_pose.pose.position.x - current_grasping_state.gripper_pose.pose.position.x;
            if(RobotInterface::abs(x1-x2) <= 0.005)
            {
                double y1 = graspObjects[object_id]->simulationDataCollectionWithObject[action][i].current_object_pose.pose.position.y - graspObjects[object_id]->simulationDataCollectionWithObject[action][i].current_gripper_pose.pose.position.y;
                double y2 = current_grasping_state.object_pose.pose.position.y - current_grasping_state.gripper_pose.pose.position.y;
                if(RobotInterface::abs(y1-y2) <= 0.005)
                {
                   /* if(debug)
                    {
                        std::cout << "Pushing particle with difference(" << abs(x1-x2) << ", " << abs(y1-y2) << ")" << std::endl;
                        std::cout << "x1 = " << x1 << " x2 = " << x2 << " y1 = " << y1 << " y2 = " << y2 << std::endl;
                        simulationDataCollectionWithObject[object_id][action][i].PrintSimulationData();
                        PrintState(current_grasping_state);
                    }*/
                    tempDataVector.push_back(graspObjects[object_id]->simulationDataCollectionWithObject[action][i]);
                }
            }

        }
    }
    //sort(simulationDataCollectionWithObject[action].begin(), simulationDataCollectionWithObject[action].end(), sort_by_object_relative_pose_initial_state_x);
     double step_start_t1_1 = despot::get_time_second();
     if(debug)
        {
            std::cout << "Temp data size" << tempDataVector.size();
        }
     //tempData.current_object_pose.pose.position.x = tempData.current_object_pose.pose.position.x - 0.005;
    //x_lower_bound = lower_bound(simulationDataCollectionWithObject[action].begin(), simulationDataCollectionWithObject[action].end(), tempData, sort_by_object_relative_pose_initial_state_x);
    //tempData.current_object_pose.pose.position.x = tempData.current_object_pose.pose.position.x + 0.01;
    //x_upper_bound = upper_bound(simulationDataCollectionWithObject[action].begin(), simulationDataCollectionWithObject[action].end(), tempData, sort_by_object_relative_pose_initial_state_x);

    double step_start_t2 = despot::get_time_second();
    //sort(x_lower_bound, x_upper_bound, sort_by_object_relative_pose_initial_state_y);

    //tempData.current_object_pose.pose.position.y = tempData.current_object_pose.pose.position.y - 0.005;
    //xy_lower_bound = lower_bound(x_lower_bound, x_upper_bound, tempData, sort_by_object_relative_pose_initial_state_y);
    //tempData.current_object_pose.pose.position.y = tempData.current_object_pose.pose.position.y + 0.01;
    //xy_upper_bound = upper_bound(x_lower_bound, x_upper_bound, tempData, sort_by_object_relative_pose_initial_state_y);

    double step_start_t3 = despot::get_time_second();
    xy_lower_bound = tempDataVector.begin();
    xy_upper_bound = tempDataVector.end();
    // check if x and y exists in simulation data with object
    if(std::distance(xy_lower_bound,xy_upper_bound) > 1)
    {   stateInObjectData = true;
    //std::cout << "No of objects : " << xy_upper_bound - xy_lower_bound << std::endl;
        if(!use_probabilistic_neighbour_step)
        {
            tempData = GetNextStateUsingNearestNeighbourSearch(xy_lower_bound, xy_upper_bound, current_grasping_state, action, debug);
        }
        else
        {
            int tempDataIndex = GetNextStateProbabilistically(xy_lower_bound, xy_upper_bound, current_grasping_state, action,random_num, debug);
            tempData = tempDataVector[tempDataIndex];
        }
    }

    double step_start_t4 = despot::get_time_second();

    if(stateInObjectData || stateInGripperData)
    {
        if(debug)
        {
            std::cout << "State being updated from ";
            if (stateInObjectData){
                std::cout << "object ";
            }
            if (stateInGripperData) {
                std::cout << "gripper ";
            }
            std::cout << "data\n";
            tempData.PrintSimulationData();
        }
        //Update next state
        //Need to update z for all actions to determine invalid state
        //if(action == A_PICK)
        //{
           grasping_state.gripper_pose.pose.position.z = grasping_state.gripper_pose.pose.position.z + tempData.next_gripper_pose.pose.position.z - tempData.current_gripper_pose.pose.position.z;
           grasping_state.object_pose.pose.position.z = grasping_state.gripper_pose.pose.position.z + tempData.next_object_pose.pose.position.z - tempData.next_gripper_pose.pose.position.z;

        //}
        //Need to update orientation to determine invalid state
        grasping_state.object_pose.pose.orientation.x = tempData.next_object_pose.pose.orientation.x;
        grasping_state.object_pose.pose.orientation.y = tempData.next_object_pose.pose.orientation.y;
        grasping_state.object_pose.pose.orientation.z = tempData.next_object_pose.pose.orientation.z;
        grasping_state.object_pose.pose.orientation.w = tempData.next_object_pose.pose.orientation.w;

        double next_gripper_pose_boundary_margin_x = 0.0;
        double next_gripper_pose_boundary_margin_y = 0.0;

        if(action < A_CLOSE)
        {
            int action_offset = (action/(A_DECREASE_X - A_INCREASE_X)) * (A_DECREASE_X - A_INCREASE_X);
            double action_range = get_action_range(action, action_offset);
            int on_bits[2];
            if(action < A_DECREASE_X) //action is increase x
            {
                if((tempData.next_gripper_pose.pose.position.x - tempData.current_gripper_pose.pose.position.x ) < action_range)
                {
                    if(tempData.next_gripper_pose.pose.position.x > max_x_i)
                    {
                        if(!CheckTouch(tempData.touch_sensor_reading, on_bits))
                        {
                            next_gripper_pose_boundary_margin_x = action_range - (tempData.next_gripper_pose.pose.position.x - tempData.current_gripper_pose.pose.position.x );
                        }
                    }
                }
            }
            else if (action < A_INCREASE_Y) //action is decrease x
            {
                if((-tempData.next_gripper_pose.pose.position.x + tempData.current_gripper_pose.pose.position.x ) < action_range)
                {
                    if(tempData.next_gripper_pose.pose.position.x < min_x_i)
                    {
                        if(!CheckTouch(tempData.touch_sensor_reading, on_bits))
                        {
                            next_gripper_pose_boundary_margin_x = - action_range + (-tempData.next_gripper_pose.pose.position.x + tempData.current_gripper_pose.pose.position.x );
                        }
                    }
                }
            }
            else if (action < A_DECREASE_Y) //action is increase y
            {
                if((tempData.next_gripper_pose.pose.position.y - tempData.current_gripper_pose.pose.position.y ) < action_range)
                {
                    if(tempData.next_gripper_pose.pose.position.y > max_y_i)
                    {
                        if(!CheckTouch(tempData.touch_sensor_reading, on_bits))
                        {
                            next_gripper_pose_boundary_margin_y = action_range - (tempData.next_gripper_pose.pose.position.y - tempData.current_gripper_pose.pose.position.y );
                        }
                    }
                }
            }
            else //action is decrease y
            {
                if((-tempData.next_gripper_pose.pose.position.y + tempData.current_gripper_pose.pose.position.y ) < action_range)
                {
                    if(tempData.next_gripper_pose.pose.position.y < min_y_i)
                    {
                        if(!CheckTouch(tempData.touch_sensor_reading, on_bits))
                        {
                            next_gripper_pose_boundary_margin_y = - action_range + (-tempData.next_gripper_pose.pose.position.y + tempData.current_gripper_pose.pose.position.y );
                        }
                    }
                }
            }
        }



        if(action != A_PICK)
        {
            grasping_state.gripper_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x + next_gripper_pose_boundary_margin_x + tempData.next_gripper_pose.pose.position.x - tempData.current_gripper_pose.pose.position.x;
            grasping_state.gripper_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y + next_gripper_pose_boundary_margin_y + tempData.next_gripper_pose.pose.position.y - tempData.current_gripper_pose.pose.position.y;


            grasping_state.object_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x + tempData.next_object_pose.pose.position.x - (tempData.next_gripper_pose.pose.position.x + next_gripper_pose_boundary_margin_x );
            grasping_state.object_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y + tempData.next_object_pose.pose.position.y - (tempData.next_gripper_pose.pose.position.y + next_gripper_pose_boundary_margin_y );
            if(use_probabilistic_step)
            {
                if (action < A_CLOSE)
                {
                    std::default_random_engine generator(random_num);
                    std::normal_distribution<double> distribution(0,0.0025);
                    grasping_state.gripper_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x + distribution(generator);
                    grasping_state.gripper_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y + distribution(generator);
                }
            }

        }
        else //Move to absolute position for PICK action
        {
        	if(RobotInterface::version8)
        	{
        		if(grasping_state.pick_success.size() == 0)
        		{
        			grasping_state.pick_success.push_back(tempData.pick_success);
        		}
        		else
        		{
        			grasping_state.pick_success[0] = tempData.pick_success;
        		}
        	}
        	else
        	{
        		grasping_state.gripper_pose.pose.position.x = tempData.next_gripper_pose.pose.position.x;
        		grasping_state.gripper_pose.pose.position.y = tempData.next_gripper_pose.pose.position.y;

        		grasping_state.object_pose.pose.position.x = tempData.next_object_pose.pose.position.x;
        		grasping_state.object_pose.pose.position.y = tempData.next_object_pose.pose.position.y;
        	}

        }

        CheckAndUpdateGripperBounds(grasping_state, action);

        if(!RobotInterface::version8 || (RobotInterface::version8 && action != A_PICK))
        {
        	//Update next observation
        	grasping_obs.gripper_pose = grasping_state.gripper_pose;
        	grasping_obs.mico_target_pose = tempData.mico_target_pose; //Needed to check if pick is valid
        	for(int i = 0; i < 4; i++)
        	{
        		grasping_state.finger_joint_state[i] = tempData.next_finger_joint_state[i];
        		grasping_obs.finger_joint_state[i] = tempData.next_finger_joint_state[i];
        	}
        	for(int i = 0; i < 2; i++)
        	{
        		grasping_obs.touch_sensor_reading[i] = tempData.touch_sensor_reading[i];
        	}
        	grasping_obs.vision_movement = tempData.vision_movement;
        	if(RobotInterface::version8)
        	{
        		grasping_obs.image_pca_values = tempData.image_pca_components;
        	}
        	//ConvertObs48ToObs2(tempData.touch_sensor_reading, grasping_obs.touch_sensor_reading);
        }

    }
    else
    {
         if(debug)
        {
            std::cout << "State being updated from default function" << std::endl;
        }
        GetNextStateAndObsUsingDefaulFunction(grasping_state, grasping_obs, action, random_num);
    }
    double step_start_t5 = despot::get_time_second();
    if(debug)
    {
        std::cout << "T112 " << step_start_t1_1 - step_start_t1 << " T212 " << step_start_t2 - step_start_t1_1 <<
            " T23 " << step_start_t3 - step_start_t2 <<
           " T34 " << step_start_t4 - step_start_t3 << " T45 " << step_start_t5 - step_start_t4 << std::endl;
    }
}

void RobotInterface::GetNextStateAndObsUsingDefaulFunction(GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, int action,double random_num, bool debug) const {

    if(action < A_CLOSE)
    {
        int action_offset = (action/(A_DECREASE_X - A_INCREASE_X)) * (A_DECREASE_X - A_INCREASE_X);
        if(action < A_DECREASE_X)
        {
            grasping_state.gripper_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x + get_action_range(action, action_offset);
        }
        else if (action < A_INCREASE_Y)
        {
            grasping_state.gripper_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x - get_action_range(action, action_offset);

        }
        else if (action < A_DECREASE_Y)
        {
            grasping_state.gripper_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y + get_action_range(action,action_offset);

        }
        else if (action < A_CLOSE)
        {
            grasping_state.gripper_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y - get_action_range(action, action_offset);

        }
    }
    else if (action == A_CLOSE)
    {
        grasping_state.closeCalled = true;
        if(version5 || version6 || version7 || version8)
        {
            grasping_state.finger_joint_state[0] = 61.15*3.14/180;
            grasping_state.finger_joint_state[1] = 0*3.14/180;
            grasping_state.finger_joint_state[2] = 61.15*3.14/180;
            grasping_state.finger_joint_state[3] = 0*3.14/180;
        }
        else
        {
            grasping_state.finger_joint_state[0] = 22.5*3.14/180;
            grasping_state.finger_joint_state[1] = 90*3.14/180;
            grasping_state.finger_joint_state[2] = 22.5*3.14/180;
            grasping_state.finger_joint_state[3] = 90*3.14/180;
        }

    }
    else if (action == A_OPEN)
    {
        grasping_state.closeCalled = false;
        grasping_state.finger_joint_state[0] = 0*3.14/180;
        grasping_state.finger_joint_state[1] = 0*3.14/180;
        grasping_state.finger_joint_state[2] = 0*3.14/180;
        grasping_state.finger_joint_state[3] = 0*3.14/180;
    }
    else if(action == A_PICK)
    {
        GetDefaultPickState(grasping_state, 0.0);

    }
    else
    {
        std::cout << "Underfined for this action" << std::endl;
        assert(false);
    }

    CheckAndUpdateGripperBounds(grasping_state, action);


    GetObsUsingDefaultFunction(grasping_state, grasping_obs, action);
}

void RobotInterface::GetObsUsingDefaultFunction(GraspingStateRealArm grasping_state, GraspingObservation& grasping_obs, int action, bool debug) const {
    //Gripper pose
    grasping_obs.gripper_pose = grasping_state.gripper_pose;

    //Mico target pose
    grasping_obs.mico_target_pose = grasping_state.gripper_pose;

    //Finger Joints
    for(int i = 0; i < 4; i++)
    {
        grasping_obs.finger_joint_state[i] = grasping_state.finger_joint_state[i];
    }

    int gripper_status = GetGripperStatus(grasping_state);

    if(version5 || version6 || version7 || version8)
    {
        for(int i = 0; i < 2; i++)
        {
           if(gripper_status == 0)
            {
                grasping_obs.touch_sensor_reading[i] = touch_sensor_mean_ver5[i];
            }
            if(gripper_status == 1)
            {
                grasping_obs.touch_sensor_reading[i] = touch_sensor_mean_closed_without_object_ver5[i];
            }
            if(gripper_status == 2)
            {
                grasping_obs.touch_sensor_reading[i] = touch_sensor_mean_closed_with_object_ver5[i];
            }
        }
    }
    else
    {
        double touch_sensor_reading[48];
        for(int i = 0; i < 48; i++)
        {
            if(gripper_status == 0)
            {
                touch_sensor_reading[i] = touch_sensor_mean[i];
            }
            if(gripper_status == 1)
            {
                touch_sensor_reading[i] = touch_sensor_mean_closed_without_object[i];
            }
            if(gripper_status == 2)
            {
                touch_sensor_reading[i] = touch_sensor_mean_closed_with_object[i];
            }
        }
        ConvertObs48ToObs2(touch_sensor_reading, grasping_obs.touch_sensor_reading);
    }
    grasping_obs.vision_movement = 0;
    if(RobotInterface::version8)
    {
    	if(action!=A_PICK)
    	{
    		grasping_obs.image_pca_values = GraspObject::default_image_pca[action];
    	}
    }
}

void RobotInterface::GetReward(GraspingStateRealArm initial_grasping_state, GraspingStateRealArm grasping_state, GraspingObservation grasping_obs, int action, double& reward) const {
bool validState = IsValidState(grasping_state);
    if(action == A_PICK)
    {
        if(IsValidPick(grasping_state, grasping_obs))
        {
            reward = pick_reward;
        }
        else
        {
            reward = pick_penalty;
        }
    }
    else
    {
        if(validState)
        {
            int initial_gripper_status = GetGripperStatus(initial_grasping_state);
            if((initial_gripper_status == 0 && action == A_OPEN) ||
              (initial_gripper_status !=0 && action <= A_CLOSE)  ||
             (((initial_grasping_state.gripper_pose.pose.position.x <(min_x_i + 0.005) && (action >= A_DECREASE_X && action < A_INCREASE_Y)) ||
              (initial_grasping_state.gripper_pose.pose.position.x >(max_x_i - 0.005)&& (action >= A_INCREASE_X && action < A_DECREASE_X)) ||
              (initial_grasping_state.gripper_pose.pose.position.y < (min_y_i + 0.005)&& (action >= A_DECREASE_Y && action < A_CLOSE) )||
              (initial_grasping_state.gripper_pose.pose.position.y > (max_y_i - 0.005)&& (action >= A_INCREASE_Y && action < A_DECREASE_Y) )))

              )
            {//Disallow open action when gripper is open and move actions when gripper is close
             //Other forbidden actions
                reward = -1*pick_reward;
            }
            else
            {



                int on_bits[2];
                bool touchDetected = CheckTouch(grasping_obs.touch_sensor_reading, on_bits);
                if(touchDetected)
                {


                    if(separate_close_reward && !RobotInterface::version8)
                    {
                        int gripper_status = GetGripperStatus(grasping_state);
                        if(gripper_status == 0) //gripper is open
                        {
                            reward = -0.5;
                            //TODO no reward if touch due to touching the wall
                        }
                        else
                        {//check grasp stability if action was close gripper
                            if (action==A_CLOSE)
                            {
                                GetRewardBasedOnGraspStability(grasping_state, grasping_obs, reward);
                            }
                            else
                            {
                                reward = -1;
                            }
                        }

                    }
                    else
                    {
                        reward = -0.5;
                    }
                    /*if(gripper_status == 1) // Touching without object
                    {
                    reward = -1;
                    }
                    if(gripper_status == 2)
                    {
                    reward = 1;
                    }*/

                }

                else
                {
                    reward = -1;
                }
            }
        }
        else
        {
            reward = invalid_state_penalty;
        }

    }
}



/*void RobotInterface::StepKerasParticles(const std::vector<float>& keras_particle_batch, int action, std::vector<float>&random_number_vecctor,
    			std::vector<tensorflow::Tensor>& outputs) const
{
	keras_models->run_transition_session(keras_particle_batch,action, random_number_vecctor,outputs);
}


void RobotInterface::GetObservationProbability(const std::vector<float>& keras_particle_batch, const std::vector<float>& keras_obs_particle_batch, int action,
			std::vector<float>&random_number_vecctor, std::vector<tensorflow::Tensor>& outputs) const
{
	if(action == A_PICK )
	{
		std::cout << "Caution! Function " << __FUNCTION__ << " getting terminal action " << action << std::endl;
		//return;
	}
	keras_models->run_observation_session(keras_particle_batch,keras_obs_particle_batch,action, random_number_vecctor,outputs);
}*/

//Conert 48 sensor observation to 2 sensor observation on front fingers by taking max
void RobotInterface::ConvertObs48ToObs2(double current_sensor_values[], double on_bits[]) const {


    for(int j = 0; j < 2; j ++)
    {
        on_bits[j] = 0.0;
        for(int i = 12; i < 24; i++)
        {

            if(on_bits[j] < current_sensor_values[i + j*24])
            {
                on_bits[j] = current_sensor_values[i + j*24];
            }
        }
    }
}
bool RobotInterface::CheckTouch(double current_sensor_values[], int on_bits[], int size) const {
//return false;
    bool touchDetected = false;
    for(int i = 0; i < size; i++)
    {
        on_bits[i] = 0;
        //if(current_sensor_values[i] > (touch_sensor_mean[i] + (3*touch_sensor_std[i])))
        if(current_sensor_values[i] > vrep_touch_threshold)
        {
            touchDetected = true;
            on_bits[i] = 1;
        }
    }

    return touchDetected;
}

void RobotInterface::PrintDicretizedObs(GraspingObservation& grasping_obs, int action, std::ostream& out) const {
    double x = round(grasping_obs.gripper_pose.pose.position.x*1000.0)/1000.0;
    double y = round(grasping_obs.gripper_pose.pose.position.y*1000.0)/1000.0;
    out << x << " " << y << " ";
    if(use_binary_touch || RobotInterface::use_discrete_observation_in_step)
    {
        int on_bits2[2];
        out << CheckTouch(grasping_obs.touch_sensor_reading, on_bits2);
        out << on_bits2[0] << " " << on_bits2[1] << " ";
    }
    else
    {
        out << grasping_obs.touch_sensor_reading[0] << " "
            << grasping_obs.touch_sensor_reading[1] << " ";
    }
    if(version7 || version8)
    {
        out << grasping_obs.vision_movement << " ";
    }
    bool closeCalled = action==A_CLOSE;
    int gripper_status = GetGripperStatus(grasping_obs.finger_joint_state, closeCalled);
    if(gripper_status == 0)
    {
        if(version5 || version6 || version7 || version8)
        {
            out << "0 0 ";

        }
        else
        {
            out << "0 0 0 0 ";
        }
    }
    if(gripper_status ==1)
    {


        if(version5 || version6 || version7 || version8)
        {
            out << "1.07 1.07 ";

        }
        else
        {
            out << "0.3925 1.57 0.3925 1.57 ";
        }


    }

    if(gripper_status == 2)
    {
        int inc = 1;
        if (version5 || version6 || version7 || version8)
        {
            inc = 2;
        }
        for(int i = 0; i < 4; i=i+inc)
        {
            //double f = round(grasping_obs.finger_joint_state[i]*100.0)/100.0;
            //out << f << " ";
            out << "0.5 " << " ";
        }

    }





}
