/* 
 * File:   VrepLogFileInterface.cpp
 * Author: neha
 * 
 * Created on October 6, 2017, 11:29 AM
 */

#include "VrepLogFileInterface.h"
#include "Python.h"

VrepLogFileInterface::VrepLogFileInterface(int start_state_index_) : VrepDataInterface(start_state_index_){
    logFileName = "simulation_trace.log";
    std::cout << "Initializing vrep log interface" << std::endl;
    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append('python_scripts')");
    
    //char ** argv;
    //std::cout << "Initialized python 1" << std::endl;
    //PySys_SetArgv(0, argv); //required when pyton script imprt rospy
    std::cout << "Initialized python 2" << std::endl;
    //PyRun_SimpleString("print sys.argv[0]");
    //std::cout << "Initialized python 3" << std::endl;
    //PyRun_SimpleString("import tensorflow as tf");
    
    //std::cout << "Initialized python" << std::endl;
    PyObject *pName;
    pName = PyString_FromString("deepLearning_data_generator");
    PyObject *pModule = PyImport_Import(pName);
    if(pModule == NULL)
    {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"log file parser\"\n");
        assert(0==1);
        return ;
    }
    Py_DECREF(pName);
    
    
    PyObject *load_function = PyObject_GetAttrString(pModule, "get_log_file_parser");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_log_file_parser\"\n");
    }
    
    PyObject *pArgs, *pValue;
    pArgs = PyTuple_New(1);
    pValue = PyString_FromString(logFileName.c_str());
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 0, pValue);

    lfp = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(load_function);
    
    if (lfp != NULL) {
        std::cout << "Call to load parser succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to load parser failed\n");
        assert(0==1);
    }
    
    
    run_function = PyObject_GetAttrString(pModule, "get_next_step_from_log");
    if (!(run_function && PyCallable_Check(run_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function get_next_step_from_log\n");
    }
    
    
}

VrepLogFileInterface::~VrepLogFileInterface() {
}

bool VrepLogFileInterface::StepActual(GraspingStateRealArm& state, double random_num, int action, double& reward, GraspingObservation& obs) const {
    GraspingStateRealArm initial_grasping_state = state;
    //For debugging
    VrepDataInterface::Step(state, random_num, action, reward, obs, true);
    
    //Get actual state from log file
    std::istringstream inputString(next_step_values);
    
    inputString >> state.gripper_pose.pose.position.x;
    inputString >> state.gripper_pose.pose.position.y;
    inputString >> state.gripper_pose.pose.position.z;
    inputString >> state.gripper_pose.pose.orientation.x;
    inputString >> state.gripper_pose.pose.orientation.y;
    inputString >> state.gripper_pose.pose.orientation.z;
    inputString >> state.gripper_pose.pose.orientation.w;

    inputString >> state.object_pose.pose.position.x;
    inputString >> state.object_pose.pose.position.y;
    inputString >> state.object_pose.pose.position.z;
    
    for (int i = 0; i < 4; i++) {
       inputString >> state.finger_joint_state[i];
       obs.finger_joint_state[i] = state.finger_joint_state[i];
               
    }
    obs.gripper_pose = state.gripper_pose;
    obs.mico_target_pose = obs.gripper_pose;
    inputString >> obs.touch_sensor_reading[0];
    inputString >> obs.touch_sensor_reading[1];
    inputString >> obs.vision_movement;
    int dummy_action;
    inputString >> dummy_action;
    
    inputString >> reward;
    bool validState = IsValidState(state);
    
    
   // std::cout << "Reward " << reward << std::endl;
    
    //Update next state parameters dependent on previous state
    UpdateNextStateValuesBasedAfterStep(state,obs,reward, action);
     if(action == A_PICK || !validState) //Wither pick called or invalid state reached
    {
        return true;
    }
    return false;

}

ValuedAction VrepLogFileInterface::NextAction(History h) {
    PyObject *pArgs, *pValue;
    pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, lfp );
    Py_INCREF(lfp);
    pValue = PyInt_FromLong(h.Size());
    PyTuple_SetItem(pArgs, 1, pValue );
    PyObject *ans = PyObject_CallObject(run_function, pArgs);
    
    
    Py_DECREF(pArgs);
    //Call python function to give rnn state and next action
    if (ans != NULL) {
        //std::cout << "Call to run model succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to run model failed\n");
        assert(0==1);
    }
    
    next_step_values = std::string(PyString_AsString(ans));
    Py_DECREF(ans);
    std::istringstream value_stream(next_step_values);
    std::cout << next_step_values << std::endl;
    double x;
    for(int i = 0; i < 17; i++)
    {
        value_stream >> x;
    }
    int action;
    value_stream >> action;
    return ValuedAction(action, 1.0);
    
    
}


