/* 
 * File:   VrepInterface.cpp
 * Author: neha
 * 
 * Created on May 4, 2015, 6:30 PM
 * Modified on 19 dec 2016 for different scene and data gathering for 2 sensors on fingers
 */

#include <unistd.h>

#include "VrepInterface.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "Display/parameters.h"
#include "Python.h"

VrepInterface::VrepInterface(int start_state_index_) : VrepDataInterface(start_state_index_) {
   
        
    //Read joint angles for initial state
    //std::ifstream infile1("data/jointData_0_final.txt");
    std::string joint_file_name = "data_table_exp/jointData.txt";
    if(RobotInterface::low_friction_table)
    {
         joint_file_name = "data_low_friction_table_exp/jointData.txt";
    }
    
    if(RobotInterface::version5 || RobotInterface::version6 || RobotInterface::version7 || RobotInterface::version8)
    {
     
        joint_file_name = "data_low_friction_table_exp";
        if(RobotInterface::version5)
        {
            joint_file_name = joint_file_name + "_ver5";
        }
        if(RobotInterface::version6 || RobotInterface::version7 || RobotInterface::version8)
        {
            joint_file_name = joint_file_name + "_ver6";
        }
        if (start_state_index_ == -10000) //gathering data with epsiln 0.005
        {
            joint_file_name = joint_file_name + "/jointData_0_0-005.txt";
        }
        else
        {
         joint_file_name = joint_file_name + "/jointData.txt";
        }
    }
    std::ifstream infile1(joint_file_name);
    int x_i, x_j;
    while (infile1 >> x_i >> x_j)
    {
        for(int i = 0; i < 10; i++)
        {
            infile1 >> joint_angles_initial_position[x_i][x_j][i];
            
        }
     }
    infile1.close();
    
    
    sim_start_client = grasping_n.serviceClient<vrep_common::simRosStartSimulation>("/vrep/simRosStartSimulation");
    sim_stop_client = grasping_n.serviceClient<vrep_common::simRosStopSimulation>("/vrep/simRosStopSimulation");
    sim_get_object_handle_client = grasping_n.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");     
    sim_get_object_pose_client = grasping_n.serviceClient<vrep_common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");     
    sim_set_object_pose_client = grasping_n.serviceClient<vrep_common::simRosSetObjectPose>("/vrep/simRosSetObjectPose");     
    sim_read_force_sensor_client = grasping_n.serviceClient<vrep_common::simRosReadForceSensor>("/vrep/simRosReadForceSensor"); 
    sim_get_joint_state_client = grasping_n.serviceClient<vrep_common::simRosGetJointState>("/vrep/simRosGetJointState"); 
    sim_set_joint_position_client = grasping_n.serviceClient<vrep_common::simRosSetJointPosition>("/vrep/simRosSetJointPosition");     
    sim_set_integer_signal_client = grasping_n.serviceClient<vrep_common::simRosSetIntegerSignal>("/vrep/simRosSetIntegerSignal"); 
    //sim_get_integer_signal_client = grasping_n.serviceClient<vrep_common::simRosGetIntegerSignal>("/vrep/simRosGetIntegerSignal"); 
    
    
        
    
        
    //Get object handles
    vrep_common::simRosGetObjectHandle object_handle_srv;
    object_handle_srv.request.objectName = "Mico_target";
    if(sim_get_object_handle_client.call(object_handle_srv))
    {
        mico_target_handle = object_handle_srv.response.handle;
    }
    
    object_handle_srv.request.objectName = "Mico_tip";
    if(sim_get_object_handle_client.call(object_handle_srv))
    {
        mico_tip_handle = object_handle_srv.response.handle;
    }
    
    
    object_handle_srv.request.objectName = "Cup";
    if(sim_get_object_handle_client.call(object_handle_srv))
    {
        target_object_handle = object_handle_srv.response.handle;
    }
    
    for(int i = 0; i < 48; i++)
    //for(int i = 0; i < 2; i++)
    {
        std::stringstream ss;
        int finger_id = (2*(i/24)) + 1 ;
        int sensor_id = (((i/12) % 2) + 1)*100 + (i % 12);
       if(sensor_id % 100 == 0)
        {
            sensor_id = sensor_id/100;
        }
        ss << "MicoHand_forceSens" << sensor_id << "_finger" << finger_id ;
        object_handle_srv.request.objectName = ss.str();
        //std::cout << i << " " << object_handle_srv.request.objectName << std::endl;
        if(sim_get_object_handle_client.call(object_handle_srv))
        {
            force_sensor_handles[i] = object_handle_srv.response.handle;
        }
    }
       
    for(int i = 0; i < 4; i++)
    {
        std::stringstream ss;
        int finger_id = (2*(i/2)) + 1 ;
        int joint_id = (i % 2) + 1;
        ss << "MicoHand_joint" << joint_id << "_finger" << finger_id ;
        object_handle_srv.request.objectName = ss.str();
        //std::cout << i << " " << object_handle_srv.request.objectName << std::endl;
        if(sim_get_object_handle_client.call(object_handle_srv))
        {
            finger_joint_handles[i] = object_handle_srv.response.handle;
        }
    }
    
    for(int i = 0; i < 6; i++)
    {
        std::stringstream ss;
        int joint_id = i + 1;
        ss << "Mico_joint" << joint_id ;
        object_handle_srv.request.objectName = ss.str();
        //std::cout << i << " " << object_handle_srv.request.objectName << std::endl;
        if(sim_get_object_handle_client.call(object_handle_srv))
        {
            arm_joint_handles[i] = object_handle_srv.response.handle;
        }
    }
    
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

void VrepInterface::LoadObjectInScene(int object_id, bool force_load) const {
    
    if(target_object_handle != -1)
    {
        
        if(force_load)
        {

            //Set target object handle to -1
            target_object_handle = -1;
            
            //Reload Object
            //The load object python script will automatically remove the object if it is present
            LoadObjectInScene(object_id, force_load);
        }
        else
        {
            std::cout << "Object already loaded. Assuming properties are set too" << std::endl;
        }
    }
    else
    {
        
        if(graspObjects.find(object_id) == graspObjects.end())
        {
            //std::cout << "Loading object properties before loading object" << std::endl;
            //This will load object properties
            graspObjects[object_id] = getGraspObject(object_id_to_filename[object_id]);
        }
        
        //This will add object in scene
        graspObjects[object_id]->loadObject(true);

        
        vrep_common::simRosGetObjectHandle object_handle_srv;
        object_handle_srv.request.objectName = "Cup";
        if(sim_get_object_handle_client.call(object_handle_srv))
        {
            target_object_handle = object_handle_srv.response.handle;
        }
    }
    
}


void VrepInterface::GetNextStateAndObservation(GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, geometry_msgs::PoseStamped mico_target_pose) const {
    
    GetStateFromVrep(grasping_state);
    
    
    //std::cout << "before getting observation" << std::endl;
                    
    //get next observation
                     
    //Gripper pose
    grasping_obs.gripper_pose = grasping_state.gripper_pose;
                    
    //Mico target pose
    grasping_obs.mico_target_pose = mico_target_pose;
    
    //Finger Joints
    for(int i = 0; i < 4; i++)
    {
        grasping_obs.finger_joint_state[i] = grasping_state.finger_joint_state[i];
    }
    // Get touch sensor observation
    GetTouchSensorReadingFromVrep(grasping_obs.touch_sensor_reading);
 
}

void VrepInterface::SetObjectPose(geometry_msgs::PoseStamped object_pose, int handle, int relativeHandle) const {
 
    vrep_common::simRosSetObjectPose set_object_pose_srv;
        set_object_pose_srv.request.handle = handle;
        set_object_pose_srv.request.relativeToObjectHandle = relativeHandle;
        set_object_pose_srv.request.pose = object_pose.pose;
        if(!sim_set_object_pose_client.call(set_object_pose_srv))
        {
            std::cout << "Call to set mico target pose failed " << std::endl;
            assert(0==1);
        }
}


void VrepInterface::SetMicoTargetPose(geometry_msgs::PoseStamped mico_target_pose) const {
        
        vrep_common::simRosSetObjectPose set_object_pose_srv;
        set_object_pose_srv.request.handle = mico_target_handle;
        set_object_pose_srv.request.relativeToObjectHandle = -1;
        set_object_pose_srv.request.pose = mico_target_pose.pose;
        if(!sim_set_object_pose_client.call(set_object_pose_srv))
        {
            std::cout << "Call to set mico target pose failed " << std::endl;
            assert(0==1);
        }
                    
        //Wait for arm to stabilize
        WaitForArmToStabilize();
        /*vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
        set_integer_signal_srv.request.signalName = "checkTip";
        set_integer_signal_srv.request.signalValue = 1;
        if(sim_set_integer_signal_client.call(set_integer_signal_srv))
        {
            std::cout << "Waiting for ros message" << std::endl;
            ros::topic::waitForMessage<std_msgs::String>("/vrep/checkTip", grasping_n);
        }
                
                
        set_integer_signal_srv.request.signalName = "checkTip";
        set_integer_signal_srv.request.signalValue = 0;
        if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
        {
        std::cout << "Call to set integer signal failed " << std::endl;
        assert(false);
        }*/
}

void VrepInterface::SetGripperPose(int i, int j) const {
    //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    SetGripperPose(i,j, mico_target_pose);
        
}

void VrepInterface::SetGripperPose(int i, int j, geometry_msgs::PoseStamped mico_target_pose) const {
    
    //Set mico joint positions
    vrep_common::simRosSetJointPosition joint_state_srv;
    for(int ii = 0; ii < 4; ii++)
    {
        joint_state_srv.request.handle = finger_joint_handles[ii];
        joint_state_srv.request.position = joint_angles_initial_position[i][j][ii];
        if(!sim_set_joint_position_client.call(joint_state_srv))
        {
            std::cout << "Call to set joint position failed" << std::endl;
            assert(0==1);
        }
    }

    for(int ii = 0; ii < 6; ii++)
    {
        joint_state_srv.request.handle = arm_joint_handles[ii];
        joint_state_srv.request.position = joint_angles_initial_position[i][j][ii+4];
        if(!sim_set_joint_position_client.call(joint_state_srv))
        {
            std::cout << "Call to set joint position failed" << std::endl;
            assert(0==1);
        }

    }
    
    //SetMicoTargetPose(mico_target_pose);
    //For some wierd reason this should be set after setting joints
    //Otherwise it gets reset
    mico_target_pose.pose.position.x = min_x_i + 0.01*i;
    mico_target_pose.pose.position.y = min_y_i + 0.01*j;
    //mico_target_pose.pose.position.z = initial_gripper_pose_z;
    //mico_target_pose.pose.orientation.x = -0.694327;
    //mico_target_pose.pose.orientation.y = -0.0171483;
    //mico_target_pose.pose.orientation.z = -0.719 ;
    //mico_target_pose.pose.orientation.w = -0.0255881;

    vrep_common::simRosSetObjectPose set_object_pose_srv;
    set_object_pose_srv.request.handle = mico_target_handle;
    set_object_pose_srv.request.relativeToObjectHandle = -1;
    set_object_pose_srv.request.pose = mico_target_pose.pose;
    if(!sim_set_object_pose_client.call(set_object_pose_srv))
    {
        std::cout << "Call to set mico target pose failed " << std::endl;
        assert(0==1);
    }


    //std::cout<<"Starting simulation" << std::endl;
    vrep_common::simRosStartSimulation start_srv;
    if(sim_start_client.call(start_srv) )
    {
        int sim_start_count = 0;
        while(start_srv.response.result != 1)
        {
            if(sim_start_count >=30)
            {
                std::cout << "ERROR: Could not start simulation after setting gripper pose" << std::endl;
                assert(0==1);
            }
            sleep(5);
            std::cout << "Receiving response" << start_srv.response.result << "while starting" << std::endl;
            sim_start_client.call(start_srv);
            sim_start_count++;           
        }
    }
    else
    {
        std::cout << "Failed to start simualtion" << std::endl;
    }

    //std::cout<<"Started simulation" << std::endl;
    //Wait for arm to stabilize
    WaitForArmToStabilize();
    /*vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
    set_integer_signal_srv.request.signalName = "checkTip";
    set_integer_signal_srv.request.signalValue = 1;
    if(sim_set_integer_signal_client.call(set_integer_signal_srv))
    {
        while(set_integer_signal_srv.response.result != 1)
        {
           std::cout << "Retrying setting signal " << set_integer_signal_srv.response.result << std::endl;
           sim_set_integer_signal_client.call(set_integer_signal_srv);

        }
        std::cout << "Waiting for ros message" << std::endl;
        ros::topic::waitForMessage<std_msgs::String>("/vrep/checkTip", grasping_n);
        std::cout << "Got ros message" << std::endl;
    }


    set_integer_signal_srv.request.signalName = "checkTip";
    set_integer_signal_srv.request.signalValue = 0;
    if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
    {
    std::cout << "Call to set integer signal failed " << std::endl;
    assert(false);
    }*/
    
        
}


void VrepInterface::PickActionInVrep(int action_offset, GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, double& reward) const {
        if(action_offset < A_PICK || action_offset > A_PICK)
    {   
        std::cout << "ERROR wrong action id while performing pick " << action_offset << std::endl; 
        assert(0==1);
    }
        GraspingStateRealArm initial_grasping_state = grasping_state;
    
    //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    //First Lift up the arm
    mico_target_pose.pose.position.z = mico_target_pose.pose.position.z + pick_z_diff;
    SetMicoTargetPose(mico_target_pose);
    
    //Move the arm to the side
    mico_target_pose.pose.position.y = pick_y_val;
    SetMicoTargetPose(mico_target_pose);
    
    // Move the arm back
    mico_target_pose.pose.position.x = pick_x_val;
    SetMicoTargetPose(mico_target_pose);
    
    // Raise it further
    mico_target_pose.pose.position.z = mico_target_pose.pose.position.z + pick_z_diff_2;
    SetMicoTargetPose(mico_target_pose);
    
      //Move the arm back further
    mico_target_pose.pose.position.x = pick_x_val_2;
    SetMicoTargetPose(mico_target_pose);
    
    GetNextStateAndObservation(grasping_state, grasping_obs, mico_target_pose);
    
    //GetReward(initial_grasping_state, grasping_state, grasping_obs, action_offset, reward);
    /*if(IsValidPick(grasping_state, grasping_obs))
    {
        reward = 20;
    }
    else
    {
        reward = -100;
    }*/
    
}


void VrepInterface::OpenCloseGripperInVrep(int action_offset, GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, double& reward) const {
    
    
       //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
         
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    if(action_offset < A_CLOSE || action_offset > A_OPEN)
    {   
        std::cout << "ERROR wrong action id while opening closing gripper " << action_offset << std::endl; 
        assert(0==1);
    }
    
    WaitForStability("closeGripper","/vrep/checkGripper",  action_offset - A_CLOSE + 1, 0);
    //Adding this as opening closing gripper takes longer now
    sleep(5);
    /*vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
    set_integer_signal_srv.request.signalName = "closeGripper";
    set_integer_signal_srv.request.signalValue = action_offset - A_CLOSE + 1;
    if(sim_set_integer_signal_client.call(set_integer_signal_srv))
    {
                
        ros::topic::waitForMessage<std_msgs::String>("/vrep/checkGripper", grasping_n);
    }
    
    set_integer_signal_srv.request.signalValue = 0;
    if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
    {
        std::cout << "Call to set integer check sensor 0 signal failed " << std::endl;
        assert(false);
    }
    */
     
    //get next state
    GetStateFromVrep(grasping_state);
    
    bool stateValid = IsValidState(grasping_state);
    //std::cout << "before getting observation" << std::endl;
                    
    //get next observation
                     
    //Gripper pose
    grasping_obs.gripper_pose = grasping_state.gripper_pose;
      

    
    //Mico target pose
    grasping_obs.mico_target_pose = mico_target_pose;
    
    //Finger Joints
    for(int i = 0; i < 4; i++)
    {
        grasping_obs.finger_joint_state[i] = grasping_state.finger_joint_state[i];
    }
    // Get touch sensor observation
    GetTouchSensorReadingFromVrep(grasping_obs.touch_sensor_reading);
    int on_bits2[2]; 
    bool touchDetected = CheckTouch(grasping_obs.touch_sensor_reading, on_bits2);
    
    
        //Decide reward
    //GetReward(initial_grasping_state, grasping_state, grasping_obs, action_offset, reward);
    /*if(stateValid)
    {
        if(touchDetected)
        {
            //Todo eliminate cases when gripper touches only wall or gripper is closed without object in it
            reward = 1; // A small positive reward for detecting touch
        }
        else
        {
            reward = -1; // Normal penalty for an action
        }
    }
    else
    {
        reward = -100; //Very high negative reward for going to invalid state
    }
    */
   
    
}

/*bool VrepInterface::CheckTouch(double current_sensor_values[], int on_bits[], int size) const {
    //return false;
    bool touchDetected = false;
    for(int i = 0; i < size; i++)
    {
        on_bits[i] = 0;
        //if(current_sensor_values[i] > (touch_sensor_mean[i] + (3*touch_sensor_std[i])))
        if(current_sensor_values[i] > 0.35)
        {
            touchDetected = true;
            on_bits[i] = 1;
        }
    }
    
    return touchDetected;
}
*/





//Return value : shall I stop moving
bool VrepInterface::TakeStepInVrep(int action_offset, int step_no, bool& alreadyTouching, GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, double& reward) const {
    
   // ros::Rate loop_rate(10); 
     //int action_offset = (action/4) * 4;
    
    int gripper_status = GetGripperStatus(grasping_state);
    GraspingStateRealArm initial_grasping_state = grasping_state;
             
    //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    
    double x_val = mico_target_pose.pose.position.x;
    double y_val = mico_target_pose.pose.position.y;
    
    bool take_step = true;
    
    if(step_no == 0) 
    { //Check gripper status only for first step because while generating data gripper is always open 
        //In real simulation gripper can be open initially 
        //but can close midway in decreasex/y action due to previous close action
        //This leads to termination of step midway which is not captured in data and leads to inconsistent belief update
    if(gripper_status > 0)
    {
        //Do not take action if gripper is closed
        std::cout << "Not taking action because gripper closed" << std::endl;
        take_step = false;
    }
    }
    if(action_offset == A_INCREASE_X )
    {
        if(x_val+0.01 > max_x_i+0.005)
        {
            take_step =  false;
        }
        mico_target_pose.pose.position.x = x_val+0.01;
        
    }
    else if(action_offset == A_DECREASE_X)
    {
        if(x_val-0.01 < min_x_i-0.005)
        {
            take_step =  false;
        }
        mico_target_pose.pose.position.x = x_val - 0.01;
    }
    else if (action_offset == A_INCREASE_Y)
    {
        if(y_val+0.01 > max_y_i - gripper_out_y_diff+0.005)
        {
            take_step= false;
        }
        mico_target_pose.pose.position.y = y_val + 0.01;
    }
    else if (action_offset == A_DECREASE_Y)
    {
        if(y_val-0.01 < min_y_i + gripper_out_y_diff-0.005)
        {
            take_step =  false;
        }
        mico_target_pose.pose.position.y = y_val - 0.01;
    }
    else
    {
        std::cout << "ERROR Wrong action offset " << action_offset << std::endl;
        assert(0==1);
    }
     
    //bool alreadyTouching = true;
    if(take_step)
    {
       //Check if already touching
        if(RobotInterface::check_touch)
        {
        if(step_no == 0)
        {

            double initial_touch_reading[2];
            int on_bits1[2];
            GetTouchSensorReadingFromVrep(initial_touch_reading);
            
            alreadyTouching = CheckTouch(initial_touch_reading, on_bits1);
            if(alreadyTouching)
            {
                std::cout<< "Already touching" <<  " ";
                for(int i = 0; i < 2; i++)
                {
                     std::cout << on_bits1[i] << " ";
                }
                std::cout << std::endl;
            }
        }
        if(alreadyTouching)
        {
            std::cout<< "Already touching" <<  " " << std::endl; 
        }
        }
        
        SetMicoTargetPose(mico_target_pose);
    }
    //loop_rate.sleep();
                    
    //std::cout << "before getting state" << std::endl;
    //get next state
    GetNextStateAndObservation(grasping_state, grasping_obs, mico_target_pose);
    
    int on_bits2[2]; 
    bool stateValid = IsValidState(grasping_state);
    bool touchDetected = CheckTouch(grasping_obs.touch_sensor_reading, on_bits2);
    
    
    //Decide reward
    //GetReward(initial_grasping_state, grasping_state, grasping_obs, action_offset, reward);
    /*if(take_step)
    {
        if(stateValid)
        {
            if(touchDetected)
            {
                //Todo eliminate cases when gripper touches only wall or gripper is closed without object in it
                reward = 1; // A small positive reward for detecting touch
            }
            else
            {
                reward = -1; // Normal penalty for an action
            }
        }
        else
        {
            reward = -100; //Very high negative reward for going to invalid state
        }
    }
    else
    {
        reward = -1; //Taking a step resuled in no action as gripper at boundary
    }
    */
    
     //Decide whether to give stop signal 
    
     // if touch toggle is detected stop as actions are move until touch
    bool stop_due_to_touch = false; 
    if(RobotInterface::check_touch)
    {
    if(!alreadyTouching)
    {
    
     stop_due_to_touch = touchDetected;
     if(stop_due_to_touch)
     {
        std::cout<< "Stopping due to touch" <<  " ";
        for(int i = 0; i < 2; i++)
        {
            std::cout << on_bits2[i] << " ";
         }
        std::cout << std::endl;
     }
    }
    }
    
    if(take_step)
    {
        if(stateValid)
        {
            
            return stop_due_to_touch;
        }
        else
        {
            std::cout<< "Stopping due to invalid state" <<  " ";
            return true; //Step should stop as state reached is invalid
        }
    }
    else
    {
        std::cout<< "Stopping because cannot take step" <<  " ";
        return true; // Step should stop as state is not changed and no action can be taken in current state
    }
}

void VrepInterface::GetTouchSensorReadingFromVrep(double touch_sensor_reading[]) const {
        
    double touch_sensor_reading48[48];
        //Send signal to start sensor data collection
        vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
        set_integer_signal_srv.request.signalName = "checkSensors";
        set_integer_signal_srv.request.signalValue = 1;
        if(sim_set_integer_signal_client.call(set_integer_signal_srv))
        {
                
            std_msgs::String::ConstPtr sensorReadingPtr = ros::topic::waitForMessage<std_msgs::String>("/vrep/checkSensors", grasping_n);
            std_msgs::String sensorReadingsString = *sensorReadingPtr.get();
            //std::cout << sensorReadingsString.data << std::endl;
            std::istringstream sensorReadings(sensorReadingsString.data);
            //for(int i = 0; i < 48; i++)
            for(int i = 0; i < 48; i++)
            {
                //std::cout << i << std::endl;
                sensorReadings >> touch_sensor_reading48[i];
            }
            ConvertObs48ToObs2(touch_sensor_reading48, touch_sensor_reading);
            
        }
        else
        {
            std::cout << "Call to set integer signal checkSensors 1 failed " << std::endl;
            assert(0==1);
        }
                
        
        //send signal 
        set_integer_signal_srv.request.signalName = "checkSensors";
        set_integer_signal_srv.request.signalValue = 0;
        if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
        {
        std::cout << "Call to set integer signal checkSensors 0 failed " << std::endl;
        assert(0==1);
        }
        
    //std::cout << std::endl;
    
}


void VrepInterface::GetStateFromVrep(GraspingStateRealArm& grasping_state) const
{
     
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.relativeToObjectHandle = -1;
    
    //Gripper pose
    object_pose_srv.request.handle = mico_tip_handle;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        grasping_state.gripper_pose  = object_pose_srv.response.pose;
    }
    
    //std::cout << "Got gripper pose " << std::endl;
    
    //Object pose
    object_pose_srv.request.handle = target_object_handle;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        grasping_state.object_pose  = object_pose_srv.response.pose;
    }
    
    //Finger joint readings
    vrep_common::simRosGetJointState joint_state_srv;
    for(int i = 0; i < 4; i++)
    {
        joint_state_srv.request.handle = finger_joint_handles[i];
        if(sim_get_joint_state_client.call(joint_state_srv))
        {
            grasping_state.finger_joint_state[i] = joint_state_srv.response.state.position[0];
        }
    }
}


void VrepInterface::GatherGripperStateData(int object_id) const {
    std::ofstream myfile;
    myfile.open ("data/simulationDefaultGripperData.txt");
    GraspingStateRealArm initial_state;
    GraspingStateRealArm* grasping_state = (&initial_state);
    //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    //vrep_common::simRosSetObjectPose set_object_pose_srv;
    vrep_common::simRosStartSimulation start_srv;
    vrep_common::simRosStopSimulation stop_srv;
    //vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
    vrep_common::simRosSetJointPosition joint_state_srv;
    
    int i_loop_max = (int)((max_x_i - min_x_i)/epsilon) + 1; //20
    int i_loop_min = 0; 
    int j_loop_max = (int)((max_y_i - min_y_i)/epsilon) + 1;//16;
    int j_loop_min = 0;//16;
    for(int i = i_loop_min; i < i_loop_max; i++) //loop over x
    {
        
        /* //For amazon shelf
         if(i < 4)
        {
           j_loop_min = 2;
           j_loop_max = 14;
        }
        else
        {
           j_loop_min = 0;
           j_loop_max = 16; 
        }*/
        int j;
        if(i%2 == 0)
        {
            j = j_loop_min;
        }
        else
        {
            j = j_loop_max - 1;
        }
        while(true) //loop over y 
        {
            
            //Set mico joint positions
            
            for(int ii = 0; ii < 4; ii++)
            {
                joint_state_srv.request.handle = finger_joint_handles[ii];
                joint_state_srv.request.position = joint_angles_initial_position[i][j][ii];
                if(!sim_set_joint_position_client.call(joint_state_srv))
                {
                    std::cout << "Call to set joint position failed" << std::endl;
                    assert(0==1);
                }
            }
            for(int ii = 0; ii < 6; ii++)
            {
                joint_state_srv.request.handle = arm_joint_handles[ii];
                joint_state_srv.request.position = joint_angles_initial_position[i][j][ii+4];
                if(!sim_set_joint_position_client.call(joint_state_srv))
                {
                    std::cout << "Call to set joint position failed" << std::endl;
                    assert(0==1);
                }
              
            }
            
            //SetMicoTargetPose(mico_target_pose);
            //For some wierd reason this should be set after setting joints
            //Otherwise it gets reset
            mico_target_pose.pose.position.x = min_x_i + 0.01*i;
            mico_target_pose.pose.position.y = min_y_i + 0.01*j;
            
            
            vrep_common::simRosSetObjectPose set_object_pose_srv;
            set_object_pose_srv.request.handle = mico_target_handle;
            set_object_pose_srv.request.relativeToObjectHandle = -1;
            set_object_pose_srv.request.pose = mico_target_pose.pose;
            if(!sim_set_object_pose_client.call(set_object_pose_srv))
            {
                std::cout << "Call to set mico target pose failed " << std::endl;
                assert(0==1);
            }
           
            //Start simulation
            if(sim_start_client.call(start_srv) )
            {
                while(start_srv.response.result != 1)
                {
                    //std::cout << "Receiving response" << start_srv.response.result << "while starting" << std::endl;
                    sim_start_client.call(start_srv);
                }
            }
            else
            {
                std::cout << "Failed to start simualtion" << std::endl;
            }

            
            //Wait for arm to stabilize
            WaitForArmToStabilize();
            /*vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
            set_integer_signal_srv.request.signalName = "checkTip";
            set_integer_signal_srv.request.signalValue = 1;
            if(sim_set_integer_signal_client.call(set_integer_signal_srv))
            {
                std::cout << "Waiting for ros message" << std::endl;
                ros::topic::waitForMessage<std_msgs::String>("/vrep/checkTip", grasping_n);
            }


            set_integer_signal_srv.request.signalName = "checkTip";
            set_integer_signal_srv.request.signalValue = 0;
            if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
            {
            std::cout << "Call to set integer signal failed " << std::endl;
            assert(false);
            }
             */ 
            
            GraspingObservation grasping_obs;
            GetNextStateAndObservation(*grasping_state, grasping_obs, mico_target_pose);

            
            
            if(sim_stop_client.call(stop_srv) ){
                if(stop_srv.response.result ==0)
                {
                     std::cout << "Simulation already stopped" << std::endl;
                }
            }
            else
            {
                std::cout << "Failed to stop simualtion" << std::endl ;
            }
            
            //Print reverse state with action id 17
            PrintState(*grasping_state, myfile);
            myfile << A_OPEN << "*";
            PrintState(*grasping_state, myfile);
            PrintObs(grasping_obs, myfile);
            myfile<<"-1" << std::endl;
            
            //std::cout << "After stopping simulation " << mico_target_pose.pose.position.y << std::endl;
            if(i%2 == 0)
            {
                j = j + 1 ;
                if(j == j_loop_max)
                {
                    break;
                }
            }
            else
            {
                j = j - 1;
                if(j == j_loop_min - 1)
                {
                    break;
                }
            }
        }
    }
    
    myfile.close();
}
void VrepInterface::GatherJointData(int object_id, double epsi) const {
    std::ofstream myfile;
    //myfile.open ("data_table_exp/jointData_0.txt");
    //myfile.open ("data_low_friction_table_exp/jointData_0.txt");
    //myfile.open ("data_low_friction_table_exp_ver5/jointData_0.txt");
    //myfile.open ("data_low_friction_table_exp_ver5/jointData_0_0-005.txt");
    
    std::string out_file = "data_low_friction_table_exp";
    if(RobotInterface::version5)
    {
        out_file = out_file + "_ver5";
       
    }
    if(RobotInterface::version6)
    {
        out_file = out_file + "_ver6";
       
    }
    if(RobotInterface::version7)
    {
        out_file = out_file + "_ver7";
       
    }
    if(RobotInterface::version8)
    {
        out_file = out_file + "_ver8";
       
    }
    out_file = out_file + "/jointData_0";
    if(epsi == 0.005)
    {
        out_file = out_file + "_0-005";
    }
    out_file = out_file + ".txt";
    myfile.open(out_file);
    //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    //vrep_common::simRosSetObjectPose set_object_pose_srv;
    vrep_common::simRosStartSimulation start_srv;
    vrep_common::simRosStopSimulation stop_srv;
    
    //Start simulation
    if(sim_start_client.call(start_srv) )
    {
        while(start_srv.response.result != 1)
        {
            //std::cout << "Receiving response" << start_srv.response.result << "while starting" << std::endl;
            sim_start_client.call(start_srv);
        }
    }
    else
    {
        std::cout << "Failed to start simualtion" << std::endl;
    }
    

    
    
    int i_loop_max = (int)((max_x_i - min_x_i)/epsi) + 1; //20
    int i_loop_min = 0; 
    int j_loop_max = (int)((max_y_i - min_y_i)/epsi) + 1;//16;
    int j_loop_min = 0;//16;
    for(int i = i_loop_min; i < i_loop_max; i++) //loop over x
    {
        /*
        //Required for amazon shelf
        if(i < 4)
        {
           j_loop_min = 2;
           j_loop_max = 14;
        }
        else
        {
           j_loop_min = 0;
           j_loop_max = 16; 
        }
         */ 
        int j;
        if(i%2 == 0)
        {
            j = j_loop_min;
        }
        else
        {
            j = j_loop_max - 1;
        }
        while(true) //loop over y 
        {
            
           /*
            //Required for amazon shelf
             if(i == 4 && j == j_loop_min)
            {
                mico_target_pose.pose.position.x = min_x_i + 0.01*i;
                mico_target_pose.pose.position.y = min_y_i + 0.01*(j+2);
                SetMicoTargetPose(mico_target_pose);
            }*/
            mico_target_pose.pose.position.x = min_x_i + epsi*i;
            mico_target_pose.pose.position.y = min_y_i + epsi*j;
            SetMicoTargetPose(mico_target_pose);

            WaitForArmToStabilize();
            //Get state and joint information
            myfile << i << " " << j << " ";
            //Finger joint readings
            vrep_common::simRosGetJointState joint_state_srv;
            for(int ii = 0; ii < 4; ii++)
            {
                joint_state_srv.request.handle = finger_joint_handles[ii];
                if(sim_get_joint_state_client.call(joint_state_srv))
                {
                    myfile << joint_state_srv.response.state.position[0] << " ";
                }
            }
            for(int ii = 0; ii < 6; ii++)
            {
                joint_state_srv.request.handle = arm_joint_handles[ii];
                if(sim_get_joint_state_client.call(joint_state_srv))
                {
                    myfile << joint_state_srv.response.state.position[0] << " ";
                }
              
            }
            
            myfile << std::endl;
            

            if(i%2 == 0)
                {
                    j = j + 1 ;
                    if(j == j_loop_max)
                    {
                        break;
                    }
                }
                else
                {
                    j = j - 1;
                    if(j == j_loop_min - 1)
                    {
                        break;
                    }
                }
        }
    }
    
    //Stop simulation
    //std::cout << "Stopping simulation" << std::endl;

    if(sim_stop_client.call(stop_srv) ){
        if(stop_srv.response.result ==0)
        {
             std::cout << "Simulation already stopped" << std::endl;
        }
    }
    else
    {
        std::cout << "Failed to stop simualtion" << std::endl ;
    }
    
    myfile.close();             
    
}

void VrepInterface::GatherDataStep(GraspingStateRealArm* grasping_state, 
        std::ofstream& myfile, int i, int j, int k, int k1,
        geometry_msgs::PoseStamped mico_target_pose, std::string object_id) const {
    //Get initial State
    std::cout << "Getting initial state" << std::endl;
    GetStateFromVrep(*grasping_state);

    bool isValid = IsValidState(*grasping_state);
    bool isReachable = IsReachableState(*grasping_state, mico_target_pose);
    bool isInCollision = (GetCollisionState() == pick_penalty);
    bool isTerminal = false;
    
    if(isValid && isReachable && (!isTerminal) &&(!isInCollision))
    {
        if(k1 >=0)
        {
            double reward_;
            GraspingObservation grasping_obs;
            isTerminal = StepActual(*grasping_state, 0.0, k1, reward_, grasping_obs);
            isValid = IsValidState(*grasping_state);
            isReachable = IsReachableState(*grasping_state, grasping_obs.mico_target_pose);
        }
        if(k == A_OPEN)
        {
            double reward_;
            GraspingObservation grasping_obs;
            isTerminal = StepActual(*grasping_state, 0.0, A_CLOSE, reward_, grasping_obs);
            isValid = IsValidState(*grasping_state);
            isReachable = IsReachableState(*grasping_state, grasping_obs.mico_target_pose);
        }
    }
    
     //Print Initial state
    std::cout << "Before printing initial state" << std::endl;
    myfile << i << " " << j << " ";
    PrintState(*grasping_state, myfile);
    myfile << k << "*";
    std::cout << "Printed initial state" << std::endl;

    //Take action
    double reward = -1;
    GraspingObservation obs;

    if(!isValid)
    {
        reward = -1000; //to signal invalid initial state
    }  
    else
    {
        if(!isReachable)
        {
            reward = -2000;
        }
        else
        {
            if(isInCollision)
            {
                reward = -3000;
            }
            else
            {
                isTerminal = StepActual(*grasping_state, 0.0, k, reward, obs);
                isValid = IsValidState(*grasping_state);
                isReachable = IsReachableState(*grasping_state, obs.mico_target_pose);
            }
        }
    }

    PrintState(*grasping_state, myfile);
    PrintObs(obs, myfile);
    myfile << reward << std::endl;


    if(k == A_CLOSE && (object_id!="-1"))
    {  
        if(isValid && isReachable && (!isTerminal) &&(!isInCollision))
        {
            /*
            //Print reverse state with action id 17
            //Not needed as reverse state is not necessarily achieved after open action
            myfile << i << " " << j << " ";
            PrintState(*grasping_state, myfile);
            myfile << A_OPEN << "*";
            PrintState(initial_state_copy, myfile);
            PrintObs(grasping_obs, myfile);
            myfile<<"-1" << std::endl;
             */
            
            //Print and perform pick action
            myfile << i << " " << j << " ";
            PrintState(*grasping_state, myfile);
            myfile << A_PICK << "*";
            StepActual(*grasping_state, 0.0, A_PICK, reward, obs);
            PrintState(*grasping_state, myfile);
            PrintObs(obs, myfile);
            myfile << reward << std::endl;

        } 
    }
                    
}

void VrepInterface::GatherData(std::string object_id, int action_type, double gap,
        int min_x, int max_x, int min_y, int max_y, int rot_z,
        int object_state_id, bool generate_default) const {
    //return;

    std::ofstream myfile;
    std::string filename;
    std::string file_dir;
    std::string filename_suffix;
    double theta_z;
    if(RobotInterface::version5 || RobotInterface::version6 || RobotInterface::version7 || RobotInterface::version8)
    {
        file_dir =  graspObjects[0]->regression_data_dir_name + "/";
        /*
        if(version5)
        {
            file_dir = "data_low_friction_table_exp_ver5/data_for_regression/";
        }
        if(version6)
        {
            file_dir = "data_low_friction_table_exp_ver6/data_for_regression/";
        }
        if(version7)
        {
            file_dir = "data_low_friction_table_exp_ver7/data_for_regression/";
        }
        */
        filename =  object_id + "/SASOData_";
        if (gap == 0.01)
        {
        //filename = "data_low_friction_table_exp_ver5/SASOData_";

        }
        else{
           //filename = "data_low_friction_table_exp_ver5/SASOData_0-005_"; 
            filename = filename + "0-005_";
        }
    }
    else
    {
        filename = "data_low_friction_table_exp/SASOData_Cylinder_";
    }
    filename = filename +object_id;
    filename = filename + "_";
    filename = filename + std::to_string(object_state_id)+ "_";
    if(min_x+max_x+min_y + max_y > -4)
    {
        filename = filename + std::to_string(min_x) + "-";
        filename = filename + std::to_string(max_x) + "-";
        filename = filename + std::to_string(min_y) + "-";
        filename = filename + std::to_string(max_y) + "_";        
    }
    
    if(rot_z > -1)
    {
        filename = filename + std::to_string(rot_z) + "_";
        theta_z = rot_z*1.0;
        
    }
    
    
    bool allActions = true;
    int k_loop_min_value = 0;
    int k_look_max_value = A_CLOSE+1;
    int k1_loop_min_value = -1;
    int k1_loop_max_value = A_CLOSE;
    if(action_type == 0)
    {
        filename = filename + "allActions.txt";
    }
    if(action_type == 1)
    {
        filename = filename + "openAction.txt";
        k_loop_min_value = A_OPEN;
        k_look_max_value = A_OPEN + 1;
    }
    if(action_type == 2)
    {
        filename = filename + "closeAndPushAction.txt";
        k_loop_min_value = A_CLOSE;
        k_look_max_value = A_CLOSE + 1;
    }
    if(action_type == 4)
    {
        filename = filename + "collisionCheck.txt";
        k_loop_min_value = A_COLLISIONCHECK;
        k_look_max_value = A_COLLISIONCHECK + 1;
        k1_loop_max_value = 0;
    }
    filename_suffix = filename;
    filename = file_dir + filename;
    std::cout << "filename is " << filename << std::endl;
    
    //myfile.open ("data_table_exp/SASOData_Cuboid_7cm_allActions.txt");
    //myfile.open ("data_table_exp/SASOData_Cylinder_85mm_allActions.txt");
   
    GraspingStateRealArm initial_state ;
    initial_state.object_id = 0;
    
    
    if(generate_default)
    {
        std::string filename_pruned;
        /*
        std::string filename_pruned = "data_low_friction_table_exp";
        if(RobotInterface::version5)
        {
            filename_pruned = filename_pruned + "_ver5";
        }
        if(RobotInterface::version6)
        {
            filename_pruned = filename_pruned + "_ver6";
        }
        if(RobotInterface::version7)
        {
            filename_pruned = filename_pruned + "_ver7";
        }
        filename_pruned = filename_pruned + "/pruned_data_files/";
        */
        filename_pruned = graspObjects[0]->regression_data_dir_name + "/pruned_data_files/";
        filename_pruned = filename_pruned + filename_suffix;
        std::vector<int> line_nos = getSimulationDataFromFile(initial_state.object_id, filename, action_type, true, filename_pruned);
        std::string filename_lineno = filename+ ".Usedlinenos";
        myfile.open(filename_lineno);
        for(int i = 0; i < line_nos.size(); i++)
        {
            myfile<< line_nos[i] << std::endl;
        }
        myfile.close();
        
        
        
        return;
    }
    
    //Set Object pose
    start_state_index = object_state_id;
    CreateObjectStartState(initial_state);
    if(rot_z > -1)
    {
        graspObjects[0]->rotateZ(theta_z);
    }
    
    /*if(initial_state.object_id == -1)
    {
        initial_state.object_id = 0;
        if(object_id > 1000)// G3DB object
        {
           initial_state.object_id = 1; 
        }
        
    }*/
    GraspingStateRealArm* grasping_state = &initial_state;
    
    //ros::Rate loop_rate(10);
    
    //Get Current Pose of mico target
    geometry_msgs::PoseStamped mico_target_pose;
    vrep_common::simRosGetObjectPose object_pose_srv;
    object_pose_srv.request.handle = mico_target_handle;
    object_pose_srv.request.relativeToObjectHandle = -1;
    if(sim_get_object_pose_client.call(object_pose_srv))
    {
        mico_target_pose = object_pose_srv.response.pose;
    }
    
    //vrep_common::simRosSetObjectPose set_object_pose_srv;
    vrep_common::simRosStartSimulation start_srv;
    vrep_common::simRosStopSimulation stop_srv;
    //vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
    
    int i_loop_max = (int)((max_x_i - min_x_i)/gap) + 1; //20
    if(max_x > -1 && max_x < i_loop_max) i_loop_max = max_x;
    int i_loop_min = 0;  //20
    if(min_x > -1) i_loop_min = min_x;
    int j_loop_max = (int)((max_y_i - min_y_i)/gap) + 1;//16;
    if  (max_y > -1 && max_y < j_loop_max) j_loop_max = max_y;
    int j_loop_min = 0;//16;
    if(min_y > -1) j_loop_min = min_y;
    int k_loop_max = k_look_max_value;
    int k_loop_min = k_loop_min_value ;
    
    if((i_loop_min < i_loop_max) &&
       (j_loop_min < j_loop_max) &&
       (k1_loop_min_value < k1_loop_max_value) &&
       (k_loop_min < k_loop_max))
    {
        myfile.open (filename);
    }
    else
    {
        std::cout << "Not gathering data" << std::endl;
        return;
    }
    
    
    //int l_loop = 2;
    /*double theta_z = 0.0;
    if(rot_z > -1)
    {
        theta_z = rot_z * 1.0;
    }*/
    
    for(int i = i_loop_min; i < i_loop_max; i++) //loop over x
        {

            /*
            if(i < 4)
            {
               j_loop_min = 2;
               j_loop_max = 14;
            }
            else
            {
               j_loop_min = 0;
               j_loop_max = 16; 
            }*/
            for(int j = j_loop_min; j < j_loop_max; j++) //loop over y 
            {
                //Set mico joint positions
                 //Removing joint position setting as this leads to collisions with g3DB objects
                //Again adding as setting position after simulation leads to displacement of object
                vrep_common::simRosSetJointPosition joint_state_srv;
                for(int ii = 0; ii < 4; ii++)
                {
                    joint_state_srv.request.handle = finger_joint_handles[ii];
                    joint_state_srv.request.position = joint_angles_initial_position[i][j][ii];
                   
                    if(!sim_set_joint_position_client.call(joint_state_srv))
                    {
                        std::cout << "Call to set joint position failed" << std::endl;
                        assert(0==1);
                    }
                }
                for(int ii = 0; ii < 6; ii++)
                {
                    joint_state_srv.request.handle = arm_joint_handles[ii];
                    joint_state_srv.request.position = joint_angles_initial_position[i][j][ii+4];
                    
                    if(!sim_set_joint_position_client.call(joint_state_srv))
                    {
                        std::cout << "Call to set joint position failed" << std::endl;
                        assert(0==1);
                    }

                }
                

                //SetMicoTargetPose(mico_target_pose);
                //For some wierd reason this should be set after setting joints
                //Otherwise it gets reset
                mico_target_pose.pose.position.x = min_x_i + gap*i;
                mico_target_pose.pose.position.y = min_y_i + gap*j;


                vrep_common::simRosSetObjectPose set_object_pose_srv;
                set_object_pose_srv.request.handle = mico_target_handle;
                set_object_pose_srv.request.relativeToObjectHandle = -1;
                set_object_pose_srv.request.pose = mico_target_pose.pose;
                if(!sim_set_object_pose_client.call(set_object_pose_srv))
                {
                    std::cout << "Call to set mico target pose failed " << std::endl;
                    assert(0==1);
                }                 
               
                
                for(int k1 = k1_loop_min_value; k1 < k1_loop_max_value; k1++)
                {
                    for(int k = k_loop_min; k < k_loop_max; k++) //loop over actions
                    {

                         //std::cout<<"Starting simulation" << std::endl;
                         if(sim_start_client.call(start_srv) )
                         {
                             while(start_srv.response.result != 1)
                             {
                                 std::cout << "Receiving response" << start_srv.response.result << "while starting" << std::endl;
                                 sim_start_client.call(start_srv);
                             }
                         }
                         else
                         {
                             std::cout << "Failed to start simualtion" << std::endl;
                         }

                         //Wait for arm to stabilize
                         WaitForArmToStabilize();

                         //mico_target_pose.pose.position.x = min_x_i + 0.01*i;
                         //mico_target_pose.pose.position.y = min_y_i + 0.01*j;
                         //SetMicoTargetPose(mico_target_pose);
                         //loop_rate.sleep();
                         //loop_rate.sleep();
                         
                         grasping_state->closeCalled = false;
                         GatherDataStep(grasping_state, myfile, i,j,k,k1 ,mico_target_pose, object_id);

                         //Stop simulation
                         //std::cout << "Stopping simulation" << std::endl;

                         if(sim_stop_client.call(stop_srv) ){
                             if(stop_srv.response.result ==0)
                             {
                                  std::cout << "Simulation already stopped" << std::endl;
                             }
                         }
                         else
                         {
                             std::cout << "Failed to stop simualtion" << std::endl ;
                         }



                    }
                }
            }
        }
    
    myfile.close();
}


void VrepInterface::WaitForArmToStabilize() const {
    WaitForStability("checkTip", "/vrep/checkTip", 1, 0 );
}

void VrepInterface::WaitForStability(std::string signal_name, std::string topic_name, int signal_on_value, int signal_off_value) const {

        vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
        set_integer_signal_srv.request.signalName = signal_name;
        set_integer_signal_srv.request.signalValue = signal_on_value;
        
        //vrep_common::simRosGetIntegerSignal get_integer_signal_srv;
        //get_integer_signal_srv.request.signalName = signal_name;
        std_msgs::StringConstPtr msg = NULL;
        //while(true)
        //{
        //    if(msg == NULL){
                
                
                if(sim_set_integer_signal_client.call(set_integer_signal_srv))
                {
                    while(set_integer_signal_srv.response.result != 1)
                    {
                        std::cout << "Retrying setting signal " << set_integer_signal_srv.response.result << std::endl;
                        sim_set_integer_signal_client.call(set_integer_signal_srv);

                    }
                }     
                    
                    std::cout << "Waiting for ros message on topic " << topic_name << std::endl;
                    msg = ros::topic::waitForMessage<std_msgs::String>(topic_name, grasping_n, ros::Duration(600));
                     std::cout << "Got ros message" << std::endl;
         
                    
                    set_integer_signal_srv.request.signalName = signal_name;
                    set_integer_signal_srv.request.signalValue = signal_off_value;
                    if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
                    {
                        std::cout << "Call to set integer signal "<< signal_name << " " << signal_off_value << " failed " << std::endl;         
                        assert(0==1);
                    }
              //  }
                     
              //  else
               // {
                //    std::cout << "Call to set integer signal "<< signal_name << " " << signal_on_value << " failed " << std::endl;
                //    assert(false);
               // }

   
               
          //  }
          //  else
          //  {
                    
             //   std::cout << "Got ros message" << std::endl;
          //      break;
           // }
        //}
            
            
        
      
        
}
/* bool VrepInterface::IsValidState(GraspingStateRealArm grasping_state) const {
    bool isValid = true;
    //Check gripper is in its range
    if(grasping_state.gripper_pose.pose.position.x < min_x_i - 0.005 ||
       grasping_state.gripper_pose.pose.position.x > max_x_i + 0.005||
       grasping_state.gripper_pose.pose.position.y < min_y_i - 0.005 ||
       grasping_state.gripper_pose.pose.position.y > max_y_i + 0.005)
    {
        return false;
    }
    
//    if(grasping_state.gripper_pose.pose.position.x < gripper_in_x_i)
    if(grasping_state.gripper_pose.pose.position.x < max_x_i + 0.005)
    {
        if(grasping_state.gripper_pose.pose.position.y < min_y_i + gripper_out_y_diff - 0.005 ||
         grasping_state.gripper_pose.pose.position.y > max_y_i - gripper_out_y_diff + 0.005)
        {
            return false;
        }  
    }
    
    
        
    //Check object is in its range
    if(grasping_state.object_pose.pose.position.x < min_x_o ||
       grasping_state.object_pose.pose.position.x > max_x_o ||
       grasping_state.object_pose.pose.position.y < min_y_o ||
       grasping_state.object_pose.pose.position.y > max_y_o ||
       grasping_state.object_pose.pose.position.z < min_z_o) // Object has fallen
    {
        return false;
    }
    return isValid;
}
*/
/*bool VrepInterface::IsValidPick(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs) const {
 bool isValidPick = true;
    
    //if object and tip are far from each other set false
    double distance = 0;
    distance = distance + pow(grasping_state.gripper_pose.pose.position.x - grasping_state.object_pose.pose.position.x, 2);
    distance = distance + pow(grasping_state.gripper_pose.pose.position.y - grasping_state.object_pose.pose.position.y, 2);
    distance = distance + pow(grasping_state.gripper_pose.pose.position.z - grasping_state.object_pose.pose.position.z, 2);
    distance = pow(distance, 0.5);
    if(distance > 0.08)
    {
        isValidPick= false;
    }
            
    
    // if target and tip are far from each other set false
    distance = 0;
    distance = distance + pow(grasping_state.gripper_pose.pose.position.x - grasping_obs.mico_target_pose.pose.position.x, 2);
    distance = distance + pow(grasping_state.gripper_pose.pose.position.y - grasping_obs.mico_target_pose.pose.position.y, 2);
    distance = distance + pow(grasping_state.gripper_pose.pose.position.z - grasping_obs.mico_target_pose.pose.position.z, 2);
    distance = pow(distance, 0.5);
    if(distance > 0.03)
    {
        isValidPick = false;
    }
      
    
    return isValidPick;
}*/

/*void VrepInterface::GetDefaultPickState(GraspingStateRealArm& grasping_state) const {
        grasping_state.gripper_pose.pose.position.z = grasping_state.gripper_pose.pose.position.z + pick_z_diff;
        grasping_state.gripper_pose.pose.position.x =  pick_x_val; 
        grasping_state.gripper_pose.pose.position.y =  pick_y_val;
        int gripper_status = GetGripperStatus(grasping_state.finger_joint_state);
        if(gripper_status == 2) //Object is inside gripper and gripper is closed
        {

            grasping_state.object_pose.pose.position.x = grasping_state.gripper_pose.pose.position.x + 0.03;
            grasping_state.object_pose.pose.position.y = grasping_state.gripper_pose.pose.position.y;
            grasping_state.object_pose.pose.position.z = grasping_state.gripper_pose.pose.position.z;
        }
}*/

/*void VrepInterface::CheckAndUpdateGripperBounds(GraspingStateRealArm& grasping_state, int action) const {
    if(action != A_PICK)
    {
    if(grasping_state.gripper_pose.pose.position.x < min_x_i)
    {
        grasping_state.gripper_pose.pose.position.x= min_x_i;
    }
    
    if(grasping_state.gripper_pose.pose.position.x > max_x_i)
    {
        grasping_state.gripper_pose.pose.position.x= max_x_i;
    }
    
    //if(grasping_state.gripper_pose.pose.position.x < gripper_in_x_i)
    if(grasping_state.gripper_pose.pose.position.x <= max_x_i)
    {
        if(grasping_state.gripper_pose.pose.position.y > max_y_i - gripper_out_y_diff)
        {
            grasping_state.gripper_pose.pose.position.y= max_y_i - gripper_out_y_diff;
        }
        if(grasping_state.gripper_pose.pose.position.y < min_y_i + gripper_out_y_diff)
        {
            grasping_state.gripper_pose.pose.position.y= min_y_i + gripper_out_y_diff;
        }
        
    }
    else
    {
        if(grasping_state.gripper_pose.pose.position.y > max_y_i)
        {
             grasping_state.gripper_pose.pose.position.y= max_y_i;
        }
        if(grasping_state.gripper_pose.pose.position.y < min_y_i)
        {
             grasping_state.gripper_pose.pose.position.y= min_y_i;
        }
    }
    }
}
 */


int VrepInterface::GetCollisionState() const {
    std::cout<< "Calling script function" << std::endl;
    ros::ServiceClient sim_call_script_function_client = grasping_n.serviceClient<vrep_common::simRosCallScriptFunction>("/vrep/simRosCallScriptFunction");
    vrep_common::simRosCallScriptFunction script_function_srv;
    script_function_srv.request.functionNameAtObjectName = "rosCollisionFunction@highTable";
    script_function_srv.request.scriptHandleOrType = 1;
    //script_function_srv.request.inputInts = {0};
    //script_function_srv.request.inputFloats= {0};
    //script_function_srv.request.inputStrings= {""};
    //script_function_srv.request.inputBuffer= "";
    if(sim_call_script_function_client.call(script_function_srv))
    {
        if ( script_function_srv.response.outputInts[0] == 0)
        {
            return pick_reward;
        }
        else
        {
            return pick_penalty;
        }
    }
    else
    {
        std::cout<< "Call to script function failed" << std::endl;
        assert(0==1);
        return -1000;
    }
    
}

std::string VrepInterface::GetAndSaveVisionImageName(int object_id) const {
    PyObject* ans;
    std::string ans_str;
    std::string raw_feedback_dir = GraspObject::raw_vision_feedback_dir + "/" + graspObjects[object_id]->GetObject_name();
    PyObject *load_function = PyObject_GetAttrString(get_belief_module, "save_current_rgb_image");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"save_current_rgb_image\"\n");
    }
    PyObject *pArgs, *pValue;
    pArgs = PyTuple_New(1);
    pValue = PyString_FromString(raw_feedback_dir.c_str());
    PyTuple_SetItem(pArgs, 0, pValue);
    ans = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(load_function);
    Py_DECREF(pArgs);
    if (ans != NULL) {
        std::cout << "Call to save_current_rgb_image succeeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to save_current_rgb_image failed\n");
        assert(0==1);
    }
    
    ans_str = PyString_AsString(ans);
    Py_DECREF(ans);
    return ans_str;
}


PyObject* VrepInterface::GetPointCloudAboveGripperPlane(double min_x) const {
    PyObject* ans;
    
    PyObject *load_function = PyObject_GetAttrString(get_belief_module, "get_current_point_cloud_for_movement");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_current_point_cloud_for_movement\"\n");
    }
    PyObject *pArgs, *pValue;
    pArgs = PyTuple_New(1);
    pValue = PyFloat_FromDouble(min_x);
    PyTuple_SetItem(pArgs, 0, pValue);
    ans = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(load_function);
    Py_DECREF(pArgs);
    if (ans != NULL) {
        std::cout << "Call to get point cloud succeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to get point clod failed\n");
        assert(0==1);
    }
    
    return ans;
    
}

int VrepInterface::CheckPointCloudMovement(PyObject* starting_point_cloud, PyObject* step_point_cloud) const {
    PyObject *load_function = PyObject_GetAttrString(get_belief_module, "has_object_moved");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"has_object_moved\"\n");
    }
    
    PyObject *pArgs;
    int num_args = 2;
    
    pArgs = PyTuple_New(num_args);
    
    PyTuple_SetItem(pArgs, 0, starting_point_cloud);
    //Set Item steals reference but we need starting point cloud later
    Py_INCREF(starting_point_cloud);
    PyTuple_SetItem(pArgs, 1, step_point_cloud);
    
    PyObject* ans = PyObject_CallObject(load_function, pArgs);
    Py_DECREF(pArgs);
    Py_DECREF(load_function);
    
    if (ans != NULL) {
        std::cout << "Call to has_object_moved succeded \n";
    }
    else {

        PyErr_Print();
        fprintf(stderr,"Call to has_object_moved failed\n");
        assert(0==1);
    }
    int vision_movement = PyInt_AsLong(ans);
    Py_DECREF(ans);
    return vision_movement;
}


bool VrepInterface::StepActual(GraspingStateRealArm& grasping_state, double random_num, int action, double& reward, GraspingObservation& grasping_obs) const {
    if (action == A_COLLISIONCHECK)
    {
        reward = GetCollisionState();
        return false;
    }
    if(RobotInterface::use_data_step)
    {
        return VrepDataInterface::StepActual(grasping_state, random_num, action, reward, grasping_obs);
    }
    GraspingStateRealArm initial_grasping_state = grasping_state;
    
    PyObject* starting_point_cloud;
    if(RobotInterface::version7 || RobotInterface::version8)
    {
        starting_point_cloud = GetPointCloudAboveGripperPlane(grasping_state.gripper_pose.pose.position.x -0.03);
    }
    std::cout << "Action is " << action ;
    if (action < A_CLOSE) { 
        int action_offset = (action/(A_DECREASE_X - A_INCREASE_X)) * (A_DECREASE_X - A_INCREASE_X);
        std::cout << "step ";
        bool alreadyTouching = true;
        for(int i = 0; i <pow(epsilon_multiplier, action-action_offset); i++)
        //    while(true)
        {
            std::cout << i << std::endl;
            bool stopMoving = TakeStepInVrep(action_offset, i, alreadyTouching, grasping_state, grasping_obs, reward);                 
            if(RobotInterface::version7 || RobotInterface::version8)
            {
                PyObject* step_point_cloud = GetPointCloudAboveGripperPlane(grasping_state.gripper_pose.pose.position.x - 0.03);
                grasping_state.vision_movement = CheckPointCloudMovement(starting_point_cloud,step_point_cloud);
                grasping_obs.vision_movement = grasping_state.vision_movement;
                //Py_DECREF(step_point_cloud); //Reference decreased with args in CheckPointCloudMovement
                
            }
            if(stopMoving || grasping_state.vision_movement == 1)
            {
                if(grasping_state.vision_movement == 1)
                {
                    std::cout << "Stopping because object movement detected \n" ;
                }
                break;
            }
                    
                    
        }   
       
    }
    else if (action < A_PICK){ 
        std::cout << std::endl;
        OpenCloseGripperInVrep(action, grasping_state, grasping_obs, reward);
        //TODO Test : Add decrease x action to make sure gripper is always open on open action
        //Do not do it while gathering data or may be do it as it will lead to open action always resulting in open gripper
        if(action == A_CLOSE)
        {
            grasping_state.closeCalled = true;
        }
        int i = 0;
        if(action == A_OPEN)
        {
            grasping_state.closeCalled = false;
            bool touching = true;
            int new_gripper_status = GetGripperStatus(grasping_state);
            while(new_gripper_status > 0)
            {    
                i = i+1;
                if (i > 5)
                {
                    //Sometimes this gets stuck in a loop as gripper doesnt open fully. So try only 5 times
                    break;
                }
                if (grasping_state.gripper_pose.pose.position.x < min_x_i + 0.005)
                {
                    grasping_state.gripper_pose.pose.position.x = min_x_i - 0.01;
                    break;
                }
                TakeStepInVrep(A_DECREASE_X, 1, touching, grasping_state, grasping_obs, reward);                 
                OpenCloseGripperInVrep(action, grasping_state, grasping_obs, reward);
                new_gripper_status = GetGripperStatus(grasping_state);
                
            }
        }
        if(RobotInterface::version7 || RobotInterface::version8)
            {
                PyObject* step_point_cloud = GetPointCloudAboveGripperPlane(grasping_state.gripper_pose.pose.position.x - 0.03);
                grasping_state.vision_movement = CheckPointCloudMovement(starting_point_cloud,step_point_cloud);
                grasping_obs.vision_movement = grasping_state.vision_movement;
                //Py_DECREF(step_point_cloud); //Reference decreased with args in CheckPointCloudMovement

            }
        
    }
    else if (action == A_PICK) { //Pick
        std::cout << std::endl;
        PickActionInVrep(action, grasping_state, grasping_obs, reward);
        //return true;
    }
    
    if(RobotInterface::version7 || RobotInterface::version8)
    {
        Py_DECREF(starting_point_cloud);
    }
    if(RobotInterface::version8)
    {
        if(action < A_PICK)
        {
            //Get vision observation
            grasping_obs.rgb_image_name = GetAndSaveVisionImageName(grasping_state.object_id);
        }
    }
    GetReward(initial_grasping_state, grasping_state, grasping_obs, action, reward);
    UpdateNextStateValuesBasedAfterStep(grasping_state,grasping_obs,reward,action);
    bool validState = IsValidState(grasping_state);
    //Decide if terminal state is reached
    if(action == A_PICK || !validState) //Wither pick called or invalid state reached
    {
        return true;
    }
    return false;
}

void VrepInterface::CreateObjectStartState(GraspingStateRealArm& initial_state, std::string type) const {
        VrepDataInterface::CreateStartState(initial_state, type);
        
        //Get object pose from vrep and update its x and y coordnates from initial state
        double object_x = initial_state.object_pose.pose.position.x;
        double object_y = initial_state.object_pose.pose.position.y;
        
         vrep_common::simRosGetObjectPose object_pose_srv;
        object_pose_srv.request.relativeToObjectHandle = -1;
        object_pose_srv.request.handle = target_object_handle;
        if(sim_get_object_pose_client.call(object_pose_srv))
        {
            initial_state.object_pose  = object_pose_srv.response.pose;
        }
        
        //Set only x and y values. Rest should be same as the current values otherwise object becomes unstable
        initial_state.object_pose.pose.position.x = object_x;
        initial_state.object_pose.pose.position.y = object_y;
        
        SetObjectPose(initial_state.object_pose, target_object_handle, -1);
            
}

void VrepInterface::CreateStartState(GraspingStateRealArm& initial_state, std::string type) const {
        
        
    
            //Stop Simulation if its already running
    int stop_count = 0;
    vrep_common::simRosStopSimulation stop_srv;
    if(sim_stop_client.call(stop_srv) ){
        while(stop_srv.response.result !=0)
        {
            sim_stop_client.call(stop_srv);
            sleep(1);
             std::cout << "Stopping simulation" << std::endl;
             stop_count++;
             if(stop_count > 10)
             {
                 break;
             }
        }
    }

    if(!sim_stop_client.call(stop_srv) )
    {
        std::cout << "ERROR: Failed to stop simualtion" << std::endl ;
        assert(0==1);
    }
       /* //Start simulation as starting simulation after setting griiper pose leads to unstable non pure shape
        vrep_common::simRosStartSimulation start_srv;
        if(sim_start_client.call(start_srv) )
        {
            while(start_srv.response.result != 1)
            {
             std::cout << "Receiving response" << start_srv.response.result << "while starting" << std::endl;
                sim_start_client.call(start_srv);
            }
        }
        else
        {
            std::cout << "Failed to start simualtion" << std::endl;
        }
        */
        
        if(RobotInterface::auto_load_object)
        {
            LoadObjectInScene(initial_state.object_id, true);
        }
        int i = 0;
        int j = 7;
        
        std::cout << "Creating Start state" << std::endl;
        //Set gripper pose in front of bin
        //Simulation started with this command
        SetGripperPose(i,j);
        std::cout << "Gripper pose set" << std::endl;
        
        
        CreateObjectStartState(initial_state,type);
        
        
        //std::cout << "Getting initial state" << std::endl;
        GetStateFromVrep(initial_state);
        if(!IsValidState(initial_state))
        {
            std::cout << "ERROR failed Invalid initial state" << std::endl;
        }
        //std::cout<< "Got initial state" << std::endl;
        
}


bool VrepInterface::IsReachableState(GraspingStateRealArm grasping_state, geometry_msgs::PoseStamped mico_target_pose) const {
    bool isReachable = true;
    //if object and tip are far from each other set false
    double distance = 0;
    distance = distance + pow(grasping_state.gripper_pose.pose.position.x - mico_target_pose.pose.position.x, 2);
    distance = distance + pow(grasping_state.gripper_pose.pose.position.y - mico_target_pose.pose.position.y, 2);
    distance = distance + pow(grasping_state.gripper_pose.pose.position.z - mico_target_pose.pose.position.z, 2);
    distance = pow(distance, 0.5);
    if(distance > 0.02)
    {
        isReachable = false;
    }
    
    return isReachable;
}
/*void VrepInterface::GetRewardBasedOnGraspStability(GraspingStateRealArm grasping_state, GraspingObservation grasping_obs, double& reward) const {
    double pick_reward;
    Step(grasping_state,Random::RANDOM.NextDouble(), A_PICK, pick_reward, grasping_obs);
    if(pick_reward == 20)
    {
        reward = -0.5;
    }
    else
    {
        reward = -1.5;
    }
}
 */


std::pair <std::map<int,double>,std::vector<double> > VrepInterface::GetBeliefObjectProbability(std::vector<int> belief_object_ids) const {
    if(!RobotInterface::get_object_belief)
    {
        return VrepDataInterface::GetBeliefObjectProbability(belief_object_ids);
    }
    std::map<int,double> belief_object_weights;
    std::vector<double> vision_observation;

    std::cout << "Initialized python 2" << std::endl;
    //PyRun_SimpleString("print sys.argv[0]");
    //std::cout << "Initialized python 3" << std::endl;
    //PyRun_SimpleString("import tensorflow as tf");
    
    //std::cout << "Initialized python" << std::endl;
     
    
    PyObject *load_function = PyObject_GetAttrString(get_belief_module, "get_belief_for_objects");
    if (!(load_function && PyCallable_Check(load_function)))
    {
        if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"get_belief_for_objects\"\n");
    }
    
    PyObject *pArgs, *pValue;
    int num_args = 2;
    if(RobotInterface::use_classifier_for_belief)
    {
        num_args = 5;
    }
    pArgs = PyTuple_New(num_args);
    PyObject* object_list = PyList_New(belief_object_ids.size());
    for(int i =0;i < belief_object_ids.size(); i++)
    {
        PyList_SetItem(object_list, i, PyString_FromString(object_id_to_filename[belief_object_ids[i]].c_str()));
    }
    
    PyTuple_SetItem(pArgs, 0, object_list);
    if(RobotInterface::use_classifier_for_belief)
    {
        pValue = PyString_FromString(GraspObject::object_pointcloud_for_classification_dir.c_str());
    }
    else
    {
        pValue = PyString_FromString(GraspObject::object_pointcloud_dir.c_str());
    }
    /* pValue reference stolen here: */
    PyTuple_SetItem(pArgs, 1, pValue);

    if(RobotInterface::use_classifier_for_belief)
    {
        pValue = PyInt_FromLong(clip_number_of_objects);
        PyTuple_SetItem(pArgs, 2, pValue);
        pValue = PyString_FromString(classifier_string_name.c_str());
        PyTuple_SetItem(pArgs, 3, pValue);
        pValue = PyString_FromString(graspObjects[belief_object_ids[0]]->data_dir_name.c_str());
        PyTuple_SetItem(pArgs, 4, pValue);
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


VrepInterface::VrepInterface(const VrepInterface& orig) {
}

VrepInterface::~VrepInterface() {
}

