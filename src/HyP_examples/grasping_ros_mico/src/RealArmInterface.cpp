/*
 * File:   RealArmInterface.cpp
 * Author: neha
 *
 * Created on May 6, 2015, 2:06 PM
 */

#include "RealArmInterface.h"

RealArmInterface::RealArmInterface(int start_state_index_) : VrepDataInterface(start_state_index_) {
    micoActionFeedbackClient = grasping_n.serviceClient<hyp_despot::MicoActionFeedback>("mico_action_feedback_server");
    //realArmObs = true;
    //max_x_i = 0.5279 + 0.01;  // range for gripper movement
    //min_y_i = 0.0816 - 0.08; // range for gripper movement
    //max_y_i = 0.2316 + 0.08;

    //Initialize python script for loading object
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('python_scripts')");

    char ** argv;
    //std::cout << "Initialized python 1" << std::endl;
    PySys_SetArgvEx(0, argv, 0); //Required when python script import rospy
    PyObject *pName;
    pName = PyString_FromString("get_initial_object_belief");
    get_belief_module = PyImport_Import(pName);
    if(get_belief_module == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"get_initial_object_belief\"\n");
        assert(0==1);
    }
    Py_DECREF(pName);
}

RealArmInterface::RealArmInterface(const RealArmInterface& orig) {
}

RealArmInterface::~RealArmInterface() {
}

bool RealArmInterface::StepActual(GraspingStateRealArm& state, double random_num, int action, double& reward, GraspingObservation& obs) const {
    /*if (action == A_OPEN)
    {
        action = 0;
    }*/
    GraspingStateRealArm initial_grasping_state = state;
    hyp_despot::MicoActionFeedback micoActionFeedback_srv;
    micoActionFeedback_srv.request.check_touch = RobotInterface::check_touch;
    micoActionFeedback_srv.request.check_vision_movement = RobotInterface::version7;
    if(action < A_CLOSE)
    {
        //std::cout << "gripper statsu is  " << state.gripper_status << std::endl ;
        micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.ACTION_MOVE;
        micoActionFeedback_srv.request.move_x = 0;
        micoActionFeedback_srv.request.move_y = 0;
        if (state.gripper_status == 0)
            {
            int action_offset = (action/(A_DECREASE_X - A_INCREASE_X)) * (A_DECREASE_X - A_INCREASE_X);
            double movement_value = get_action_range(action, action_offset);
            //std::cout << "Movement value is " << movement_value << std::endl;
            if(action_offset == A_INCREASE_X)
            {
                if ((state.gripper_pose.pose.position.x + movement_value) > max_x_i)
                {
                    movement_value = max_x_i - state.gripper_pose.pose.position.x;
                }
                micoActionFeedback_srv.request.move_x = movement_value;
            }
            else if(action_offset == A_DECREASE_X)
            {
                if ((state.gripper_pose.pose.position.x - movement_value) < min_x_i)
                {
                    movement_value = -min_x_i + state.gripper_pose.pose.position.x;
                }
                micoActionFeedback_srv.request.move_x = -1*movement_value;
            }
            else if(action_offset == A_INCREASE_Y)
            {
                if ((state.gripper_pose.pose.position.y + movement_value) > max_y_i)
                {
                    movement_value = max_y_i - state.gripper_pose.pose.position.y;
                }
                micoActionFeedback_srv.request.move_y = movement_value;
            }
            else if(action_offset == A_DECREASE_Y)
            {
                if ((state.gripper_pose.pose.position.y - movement_value) < min_y_i)
                {
                    movement_value = -min_y_i + state.gripper_pose.pose.position.y;
                }
                micoActionFeedback_srv.request.move_y = -1*movement_value;
            }
        }
    }
    else if(action == A_CLOSE)
    {
        micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.ACTION_CLOSE;
        state.closeCalled = true;
    }
    else if(action == A_OPEN)
    {
        micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.ACTION_OPEN;
        state.closeCalled = false;
    }
    else if (action == A_PICK)
    {
        micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.ACTION_PICK;
    }

    if(micoActionFeedbackClient.call(micoActionFeedback_srv))
    {
        state.gripper_pose = micoActionFeedback_srv.response.gripper_pose;
        AdjustRealGripperPoseToSimulatedPose(state.gripper_pose);

        for(int i = 0; i < 2; i++)
        {
            int finger_index = i;
            if(RobotInterface::version6 || RobotInterface::version7)
            {
                finger_index = 1-i;
            }
            obs.touch_sensor_reading[i] = micoActionFeedback_srv.response.touch_sensor_reading[finger_index];
        }
        AdjustTouchSensorToSimulatedTouchSensor(obs.touch_sensor_reading);



        //Finger Joints
        for(int i = 0; i < 4; i=i+2)
        {
            state.finger_joint_state[i] = micoActionFeedback_srv.response.finger_joint_state[i/2];
            obs.finger_joint_state[i] = micoActionFeedback_srv.response.finger_joint_state[i/2];
        }
        AdjustRealFingerJointsToSimulatedJoints(state, obs.touch_sensor_reading);
        for(int i = 0; i < 4; i=i+2)
        {
           obs.finger_joint_state[i] =  state.finger_joint_state[i];
        }
        //AdjustRealFingerJointsToSimulatedJoints(obs.finger_joint_state, obs.touch_sensor_reading);

        if(RobotInterface::version7)
        {
            obs.vision_movement = micoActionFeedback_srv.response.vision_movement;
        }
        obs.gripper_pose = state.gripper_pose;
        obs.mico_target_pose = obs.gripper_pose; //Not being used

    }
    else
    {
        std::cout << "Call for action failed " << std::endl;
            assert(false);
    }

    GetReward(initial_grasping_state, state, obs, action, reward);
    UpdateNextStateValuesBasedAfterStep(state,obs,reward,action);
    bool validState = true; //IsValidState(state); //Not checking valid state for real arm as state is always valid
    //Decide if terminal state is reached
    if(action == A_PICK || !validState) //Wither pick called or invalid state reached
    {
      std::cout << "Please specify reward\n";
      std::cin >> reward;

        return true;
    }
    return false;

    /*if(action == A_CLOSE)
    {
        return true;
    }
    return false;
    */
}

void RealArmInterface::CreateStartState(GraspingStateRealArm& initial_state, std::string type) const {

    //TODO: Set min and max touch values by closing and opening the gripper
    //Fetch touch threshold value from mico action feedback node
    //Get robot pose and finger joints
    //Calling open gripper functon for that
    if(graspObjects.find(initial_state.object_id) == graspObjects.end())
    {
        //This will load object properties
        graspObjects[initial_state.object_id] = getGraspObject(object_id_to_filename[initial_state.object_id]);
    }

    VrepDataInterface::CreateStartState(initial_state, type);
    hyp_despot::MicoActionFeedback micoActionFeedback_srv;
    //Move to pre grasp pos
    micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.INIT_POS;
    micoActionFeedbackClient.call(micoActionFeedback_srv);

    micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.GET_TOUCH_THRESHOLD;
    if(micoActionFeedbackClient.call(micoActionFeedback_srv))
    {
        real_touch_threshold = micoActionFeedback_srv.response.touch_sensor_reading[0];
        real_touch_value_min = 0;
        double real_touch_value_max_calc = (vrep_touch_value_max/vrep_touch_threshold)*real_touch_threshold;
        if (real_touch_value_max_calc > real_touch_value_max)
        {
            real_touch_value_max = real_touch_value_max_calc;
        }
        std::cout << "Touch params: min="<< real_touch_value_min
                  << "thresh="<< real_touch_threshold
                  << "max=" << real_touch_value_max << std::endl;
    }
    micoActionFeedback_srv.request.action = micoActionFeedback_srv.request.ACTION_OPEN;
    if(micoActionFeedbackClient.call(micoActionFeedback_srv))
    {
       initial_state.gripper_pose = micoActionFeedback_srv.response.gripper_pose;
       //min_x offset for point clod movement is 0.39 - 0.355
       real_gripper_offset_x = min_x_i + initial_gripper_pose_index_x*0.01 - initial_state.gripper_pose.pose.position.x;
       real_gripper_offset_y = min_y_i + initial_gripper_pose_index_y*0.01 - initial_state.gripper_pose.pose.position.y;
       real_gripper_offset_z = initial_gripper_pose_z - initial_state.gripper_pose.pose.position.z;
       AdjustRealGripperPoseToSimulatedPose(initial_state.gripper_pose);
       //Finger Joints
        for(int i = 0; i < 4; i=i+2)
        {
            initial_state.finger_joint_state[i] = micoActionFeedback_srv.response.finger_joint_state[i/2];

        }
       double initial_touch[2] = {0.0,0.0};
        AdjustRealFingerJointsToSimulatedJoints(initial_state, initial_touch);

    }

    //Get object pose
    //Ideally should call object detector but currently it is not ready
    //So adding a default object pose . When object pose from kinect is available should compute offeset from defalut pose

    //initial_state.object_pose = initial_state.gripper_pose;
    initial_state.object_pose.pose.position.x = graspObjects[initial_state.object_id]->initial_object_x;
    initial_state.object_pose.pose.position.y = graspObjects[initial_state.object_id]->initial_object_y;
    initial_state.object_pose.pose.position.z = graspObjects[initial_state.object_id]->initial_object_pose_z;
    AdjustRealObjectPoseToSimulatedPose(initial_state.object_pose);
    int proceed;
    std::cout << "Shall I prroceed?[1=yes]\n";
    std::cin >> proceed;
    if(proceed !=1)
    {
        assert(0==1);
    }

}




void RealArmInterface::AdjustRealFingerJointsToSimulatedJoints(GraspingStateRealArm& state, double gripper_obs_values[]) const {
  if(state.closeCalled)
  {
    std::cout << "Touch values " << gripper_obs_values[0] << " " << gripper_obs_values[1] << std::endl;
    if((gripper_obs_values[0]> 2*vrep_touch_threshold && gripper_obs_values[1]> 2*vrep_touch_threshold)
    ||(gripper_obs_values[0]> 3*vrep_touch_threshold)
    ||(gripper_obs_values[1]> 3*vrep_touch_threshold))
    {
      double joint_multiplier = gripper_obs_values[0] > gripper_obs_values[1] ? (gripper_obs_values[0]/(2.0*vrep_touch_threshold)) : (gripper_obs_values[1]/(2.0*vrep_touch_threshold));
      if (joint_multiplier > 5.0)
      {
        joint_multiplier = 5.0;
      }
      std::cout << "Adjusting gripper joint angles because of touch" << std::endl;
      for(int i = 0; i < 4; i=i+2)
      {
        state.finger_joint_state[i] = state.finger_joint_state[i] - (0.1*joint_multiplier); //To differentiate between close without object and close with a thin object
      }
    }
  }

//Adjust the gathered real joint values according to vrep
for(int i = 0; i < 4; i=i+2)
    {
        if (state.finger_joint_state[i]  < 0)
        {
            state.finger_joint_state[i] = 0;
        }

        state.finger_joint_state[i] = state.finger_joint_state[i] - real_finger_joint_min;
        state.finger_joint_state[i] = state.finger_joint_state[i] * (vrep_finger_joint_max - vrep_finger_joint_min);
        state.finger_joint_state[i] = state.finger_joint_state[i] /(real_finger_joint_max - real_finger_joint_min);
        state.finger_joint_state[i] = state.finger_joint_state[i] + vrep_finger_joint_min;

    }

    if(state.closeCalled)
    {
      int gripper_status = GetGripperStatus(state);
      if(gripper_status == 1)
      {
        std::cout << "Artificially generating touch on close. Otherwise it confuses planner and learneer" << std::endl;
        gripper_obs_values[0] = 2*vrep_touch_threshold;
        gripper_obs_values[1] = 2*vrep_touch_threshold;
      }
    }


    /*for(int i = 0; i < 4; i=i+2)
    {
        gripper_joint_values[i+1] = vrep_dummy_finger_joint_min;
        if(gripper_joint_values[i] >= vrep_finger_joint_for_dummy_joint_value_change)
        {
            double add_value = (vrep_dummy_finger_joint_max - vrep_dummy_finger_joint_min);
            add_value = add_value*(gripper_joint_values[i] - vrep_finger_joint_for_dummy_joint_value_change);
            add_value = add_value/(vrep_finger_joint_max-vrep_finger_joint_for_dummy_joint_value_change);
            gripper_joint_values[i+1] = gripper_joint_values[i+1] + add_value;
        }

    }*/
}

void RealArmInterface::AdjustTouchSensorToSimulatedTouchSensor(double gripper_obs_values[]) const {

    //TODO confirm finger order is same
    for(int i = 0; i < 2; i++)
    {
        if(gripper_obs_values[i] < real_touch_value_min)
        {
            gripper_obs_values[i] = real_touch_value_min;
        }
        if(gripper_obs_values[i] > real_touch_value_max)
        {
            gripper_obs_values[i] = real_touch_value_max;
        }
        if(gripper_obs_values[i] <= real_touch_threshold)
        {
            //Interpolate between vrep min and vrep touch threshold
            gripper_obs_values[i] = gripper_obs_values[i] - real_touch_value_min;
            gripper_obs_values[i] = gripper_obs_values[i]*(vrep_touch_threshold - vrep_touch_value_min);
            gripper_obs_values[i] = gripper_obs_values[i]/(real_touch_threshold - real_touch_value_min);
            gripper_obs_values[i] = gripper_obs_values[i] + vrep_touch_value_min;

        }
        else
        {
            //Interpolate between vrep touch threshold and vrep max
            gripper_obs_values[i] = gripper_obs_values[i] - real_touch_threshold;
            gripper_obs_values[i] = gripper_obs_values[i]*(vrep_touch_value_max - vrep_touch_threshold);
            gripper_obs_values[i] = gripper_obs_values[i]/(real_touch_value_max - real_touch_threshold);
            gripper_obs_values[i] = gripper_obs_values[i] + vrep_touch_threshold;

        }
    }

}


void RealArmInterface::AdjustRealGripperPoseToSimulatedPose(geometry_msgs::PoseStamped& gripper_pose) const {
     //Adjust with vrep
    gripper_pose.pose.position.x = gripper_pose.pose.position.x + real_gripper_offset_x;
    gripper_pose.pose.position.y = gripper_pose.pose.position.y + real_gripper_offset_y;
    gripper_pose.pose.position.z = gripper_pose.pose.position.z + real_gripper_offset_z;



    //Get tip pose
    //gripper_pose.pose.position.x = gripper_pose.pose.position.x + tip_wrt_hand_link_x;

   }

void RealArmInterface::AdjustRealObjectPoseToSimulatedPose(geometry_msgs::PoseStamped& object_pose) const {
    //This function will be needed when using kinect to determine object pose
    //object_pose.pose.position.x = object_pose.pose.position.x - real_min_x_o + min_x_o;
    //object_pose.pose.position.y = object_pose.pose.position.y - real_min_y_o + min_y_o;

}


std::pair <std::map<int,double>,std::vector<double> > RealArmInterface::GetBeliefObjectProbability(std::vector<int> belief_object_ids) const {
    if(!RobotInterface::get_object_belief)
    {
        return VrepDataInterface::GetBeliefObjectProbability(belief_object_ids);
    }
    int proceed;
    std::cout << "Getting object class. Place object correctly. \n Shall I prroceed?[1=yes]\n";
    std::cin >> proceed;
    if(proceed !=1)
    {
        assert(0==1);
    }
    std::map<int,double> belief_object_weights;
    std::vector<double> vision_observation;

    std::cout << "Initialized python 2" << std::endl;
    //PyRun_SimpleString("print sys.argv[0]");
    //std::cout << "Initialized python 3" << std::endl;
    //PyRun_SimpleString("import tensorflow as tf");

    //std::cout << "Initialized python" << std::endl;


    PyObject *load_function = PyObject_GetAttrString(get_belief_module, "get_belief_for_real_objects");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_belief_for_objects\"\n");
    }

    PyObject *pArgs, *pValue;
    int num_args = 3;
    if(RobotInterface::use_classifier_for_belief)
    {
        num_args = 6;
    }
    //Change here to real for robot experiment lab
    std::string env_name = "real_rls_object_detection";
    pValue = PyString_FromString(env_name.c_str());


    pArgs = PyTuple_New(num_args);
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 0, pValue);
    PyObject* object_list = PyList_New(belief_object_ids.size());
    for(int i =0;i < belief_object_ids.size(); i++)
    {
        PyList_SetItem(object_list, i, PyString_FromString(object_id_to_filename[belief_object_ids[i]].c_str()));
    }

    PyTuple_SetItem(pArgs, 1, object_list);
    if(RobotInterface::use_classifier_for_belief)
    {
        pValue = PyString_FromString(GraspObject::object_pointcloud_for_classification_dir.c_str());
    }
    else
    {
        pValue = PyString_FromString(GraspObject::object_pointcloud_dir.c_str());
    }
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 2, pValue);

    if(RobotInterface::use_classifier_for_belief)
    {
        pValue = PyInt_FromLong(clip_number_of_objects);
        PyTuple_SetItem(pArgs, 3, pValue);
        std::string classifier_name = classifier_string_name;
        if(object_class_value > -1)
        {

            classifier_name = classifier_name + "_object_class_" + std::to_string(object_class_value);
        }
        pValue = PyString_FromString(classifier_name.c_str());
        PyTuple_SetItem(pArgs, 4, pValue);
        pValue = PyString_FromString(graspObjects[belief_object_ids[0]]->data_dir_name.c_str());
        PyTuple_SetItem(pArgs, 5, pValue);
    }
    PyObject* belief_probs = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(load_function);

    if (belief_probs != NULL) {
        std::cout << "Call to get belief probs succeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to get belief probs failed\n");
        assert(0==1);
    }

    for(int i = 0; i < belief_object_ids.size(); i++)
    {
        PyObject* tmpObj = PyList_GetItem(belief_probs, i);
        belief_object_weights[belief_object_ids[i]] = PyFloat_AsDouble(tmpObj);
        //Py_DECREF(tmpObj);
    }

    if(RobotInterface::use_classifier_for_belief)
    {
        int weighted_obs_size = GetWeightedObservationSize();
        for(int i = belief_object_ids.size(); i < belief_object_ids.size()+weighted_obs_size; i++)
        {
            PyObject* tmpObj = PyList_GetItem(belief_probs, i);
            vision_observation.push_back(PyFloat_AsDouble(tmpObj));
            //Py_DECREF(tmpObj);
        }
    }
    else
    {
        for(int i = 0; i < belief_object_ids.size(); i++)
        {
            vision_observation.push_back(belief_object_weights[belief_object_ids[i]]);
        }
    }
    std::cout << "Vision observation " ;
    for(int i = 0; i < vision_observation.size(); i++)
    {
        std::cout << vision_observation[i] << " ";
    }
    std::cout << std::endl;

    Py_DECREF(belief_probs);
    return std::make_pair(belief_object_weights,vision_observation);

}
