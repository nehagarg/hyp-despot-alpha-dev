/* 
 * File:   VrepInterface.cpp
 * Author: neha
 * 
 * Created on May 4, 2015, 6:30 PM
 */

#include "VrepInterface.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

VrepInterface::VrepInterface() {
   
        
    //Read joint angles for initial state
    std::ifstream infile1("data/jointData_0_final.txt");
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

void VrepInterface::SetMicoTargetPose(geometry_msgs::PoseStamped mico_target_pose) const {
        
        vrep_common::simRosSetObjectPose set_object_pose_srv;
        set_object_pose_srv.request.handle = mico_target_handle;
        set_object_pose_srv.request.relativeToObjectHandle = -1;
        set_object_pose_srv.request.pose = mico_target_pose.pose;
        if(!sim_set_object_pose_client.call(set_object_pose_srv))
        {
            std::cout << "Call to set mico target pose failed " << std::endl;
            assert(false);
        }
                    
        //Wait for arm to stabilize
        vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
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
            assert(false);
        }
    }
    for(int ii = 0; ii < 6; ii++)
    {
        joint_state_srv.request.handle = arm_joint_handles[ii];
        joint_state_srv.request.position = joint_angles_initial_position[i][j][ii+4];
        if(!sim_set_joint_position_client.call(joint_state_srv))
        {
            std::cout << "Call to set joint position failed" << std::endl;
            assert(false);
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
        assert(false);
    }


    //std::cout<<"Starting simulation" << std::endl;
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

    //Wait for arm to stabilize
    vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
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
    }
        
}


void VrepInterface::PickActionInVrep(int action_offset, GraspingStateRealArm& grasping_state, GraspingObservation& grasping_obs, double& reward) const {
        if(action_offset < 18 || action_offset > 18)
    {   
        std::cout << "wrong action id while performing pick " << action_offset << std::endl; 
        assert(false);
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
    
    if(action_offset < 16 || action_offset > 17)
    {   
        std::cout << "wrong action id while opening closing gripper " << action_offset << std::endl; 
        assert(false);
    }
    vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
    set_integer_signal_srv.request.signalName = "closeGripper";
    set_integer_signal_srv.request.signalValue = action_offset - 15;
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
    
    int gripper_status = GetGripperStatus(grasping_state.finger_joint_state);
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
    if(gripper_status > 0)
    {
        //Do not take action if gripper is closed
        take_step = false;
    }
    
    if(action_offset == 0 )
    {
        if(x_val+0.01 > max_x_i+0.005)
        {
            take_step =  false;
        }
        mico_target_pose.pose.position.x = x_val+0.01;
        
    }
    else if(action_offset == 4)
    {
        if(x_val-0.01 < min_x_i-0.005)
        {
            take_step =  false;
        }
        mico_target_pose.pose.position.x = x_val - 0.01;
    }
    else if (action_offset == 8)
    {
        if(y_val+0.01 > max_y_i - gripper_out_y_diff+0.005)
        {
            take_step= false;
        }
        mico_target_pose.pose.position.y = y_val + 0.01;
    }
    else if (action_offset == 12)
    {
        if(y_val-0.01 < min_y_i + gripper_out_y_diff-0.005)
        {
            take_step =  false;
        }
        mico_target_pose.pose.position.y = y_val - 0.01;
    }
    else
    {
        std::cout << "Wrong action offset " << action_offset << std::endl;
        assert(false);
    }
     
    //bool alreadyTouching = true;
    if(take_step)
    {
       //Check if already touching
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
     
    if(take_step)
    {
        if(stateValid)
        {
            return stop_due_to_touch;
        }
        else
        {
            return true; //Step should stop as state reached is invalid
        }
    }
    else
    {
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
            assert(false);
        }
                
        
        //send signal 
        set_integer_signal_srv.request.signalName = "checkSensors";
        set_integer_signal_srv.request.signalValue = 0;
        if(!sim_set_integer_signal_client.call(set_integer_signal_srv))
        {
        std::cout << "Call to set integer signal checkSensors 0 failed " << std::endl;
        assert(false);
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
    
    int i_loop_max = 20; //20
    int i_loop_min = 0; 
    int j_loop_max = 16;//16;
    int j_loop_min = 0;//16;
    for(int i = i_loop_min; i < i_loop_max; i++) //loop over x
    {
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
                    assert(false);
                }
            }
            for(int ii = 0; ii < 6; ii++)
            {
                joint_state_srv.request.handle = arm_joint_handles[ii];
                joint_state_srv.request.position = joint_angles_initial_position[i][j][ii+4];
                if(!sim_set_joint_position_client.call(joint_state_srv))
                {
                    std::cout << "Call to set joint position failed" << std::endl;
                    assert(false);
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
                assert(false);
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
            vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
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
void VrepInterface::GatherJointData(int object_id) const {
    std::ofstream myfile;
    myfile.open ("data/jointData_0.txt");

    
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
    

    
    
    int i_loop_max = 20; //20
    int i_loop_min = 0; 
    int j_loop_max = 16;//16;
    int j_loop_min = 0;//16;
    for(int i = i_loop_min; i < i_loop_max; i++) //loop over x
    {
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
            
            if(i == 4 && j == j_loop_min)
            {
                mico_target_pose.pose.position.x = min_x_i + 0.01*i;
                mico_target_pose.pose.position.y = min_y_i + 0.01*(j+2);
                SetMicoTargetPose(mico_target_pose);
            }
            mico_target_pose.pose.position.x = min_x_i + 0.01*i;
            mico_target_pose.pose.position.y = min_y_i + 0.01*j;
            SetMicoTargetPose(mico_target_pose);

            
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

void VrepInterface::GatherData(int object_id) const {
    //return;
    std::ofstream myfile;
    myfile.open ("data_test/simulationData.txt");
    GraspingStateRealArm initial_state ;
    if(initial_state.object_id == -1)
    {
        initial_state.object_id = object_id;
        
    }
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
    
    int i_loop_max = 20; //20
    int i_loop_min = 0;  //20
    int j_loop_max = 14;//16;
    int j_loop_min = 2;//16;
    int k_loop_max = 17; 
    int k_loop_min = 0; 
    //int l_loop = 2;
   
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
                vrep_common::simRosSetJointPosition joint_state_srv;
                for(int ii = 0; ii < 4; ii++)
                {
                    joint_state_srv.request.handle = finger_joint_handles[ii];
                    joint_state_srv.request.position = joint_angles_initial_position[i][j][ii];
                    if(!sim_set_joint_position_client.call(joint_state_srv))
                    {
                        std::cout << "Call to set joint position failed" << std::endl;
                        assert(false);
                    }
                }
                for(int ii = 0; ii < 6; ii++)
                {
                    joint_state_srv.request.handle = arm_joint_handles[ii];
                    joint_state_srv.request.position = joint_angles_initial_position[i][j][ii+4];
                    if(!sim_set_joint_position_client.call(joint_state_srv))
                    {
                        std::cout << "Call to set joint position failed" << std::endl;
                        assert(false);
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
                    assert(false);
                }
 
               
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
                    vrep_common::simRosSetIntegerSignal set_integer_signal_srv;
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
                    }
                    //mico_target_pose.pose.position.x = min_x_i + 0.01*i;
                    //mico_target_pose.pose.position.y = min_y_i + 0.01*j;
                    //SetMicoTargetPose(mico_target_pose);
                    //loop_rate.sleep();
                    //loop_rate.sleep();
                
                    //Get initial State
                    std::cout << "Getting initial state" << std::endl;
                    GetStateFromVrep(*grasping_state);
                    
                    bool isValid = IsValidState(*grasping_state);
                    bool isReachable = IsReachableState(*grasping_state, mico_target_pose);
                    
                    GraspingStateRealArm initial_state_copy;
                    GraspingObservation grasping_obs;
                    if(k == A_CLOSE && object_id>0)
                    {
                        initial_state_copy = *grasping_state;
                        //Gripper pose
                        grasping_obs.gripper_pose = initial_state_copy.gripper_pose;

                        //Mico target pose
                        grasping_obs.mico_target_pose = mico_target_pose;

                        //Finger Joints
                        for(int ii = 0; ii < 4; ii++)
                        {
                            grasping_obs.finger_joint_state[ii] = initial_state_copy.finger_joint_state[ii];
                        }
                        // Get touch sensor observation
                        GetTouchSensorReadingFromVrep(grasping_obs.touch_sensor_reading);
                    }
                    
                   
                    if(k == A_OPEN)
                    {
                        double reward_;
                        StepActual(*grasping_state, 0.0, A_CLOSE, reward_, grasping_obs);
                    }
                     //Print Initial state
                    std::cout << "Before printing initial state" << std::endl;
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
                            
                            bool isTerminal = StepActual(*grasping_state, 0.0, k, reward, obs);
                        }
                    }
                
                    PrintState(*grasping_state, myfile);
                    PrintObs(obs, myfile);
                    myfile << reward << std::endl;
                    
                    
                    if(k == A_CLOSE && object_id>0)
                    {  
                        if(isValid && isReachable)
                        {
                            //Print reverse state with action id 17
                            PrintState(*grasping_state, myfile);
                            myfile << A_OPEN << "*";
                            PrintState(initial_state_copy, myfile);
                            PrintObs(grasping_obs, myfile);
                            myfile<<"-1" << std::endl;
                            
                            //Print and perform pick action
                            PrintState(*grasping_state, myfile);
                            myfile << A_PICK << "*";
                            StepActual(*grasping_state, 0.0, A_PICK, reward, obs);
                            PrintState(*grasping_state, myfile);
                            PrintObs(obs, myfile);
                            myfile << reward << std::endl;
                            
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
                 
 
        
                }
            }
        }
    
    myfile.close();
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
bool VrepInterface::StepActual(GraspingStateRealArm& grasping_state, double random_num, int action, double& reward, GraspingObservation& grasping_obs) const {
    GraspingStateRealArm initial_grasping_state = grasping_state;
    if (action < 16) { 
        int action_offset = (action/4) * 4;
        std::cout << "Action is " << action << "step ";
        bool alreadyTouching = true;
        for(int i = 0; i <pow(2, action-action_offset); i++)
        //    while(true)
        {
            std::cout << i << std::endl;
            bool stopMoving = TakeStepInVrep(action_offset, i, alreadyTouching, grasping_state, grasping_obs, reward);                 
            if(stopMoving)
            {
                break;
            }
                    
                    
        }   
       
    }
    else if (action < 18){ 
        
        OpenCloseGripperInVrep(action, grasping_state, grasping_obs, reward);
        
    }
    else if (action == 18) { //Pick
        PickActionInVrep(action, grasping_state, grasping_obs, reward);
        //return true;
    }
    

    GetReward(initial_grasping_state, grasping_state, grasping_obs, action, reward);
    UpdateNextStateValuesBasedAfterStep(grasping_state,grasping_obs,reward,action);
    bool validState = IsValidState(grasping_state);
    //Decide if terminal state is reached
    if(action == 18 || !validState) //Wither pick called or invalid state reached
    {
        return true;
    }
    return false;
}

void VrepInterface::CreateStartState(GraspingStateRealArm& initial_state, std::string type) const {
        
        
        int i = 0;
        int j = 7;
        
        std::cout << "Creating Start state" << std::endl;
        //Set gripper pose in front of bin
        SetGripperPose(i,j);
        
        //std::cout << "Getting initial state" << std::endl;
        GetStateFromVrep(initial_state);
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

VrepInterface::VrepInterface(const VrepInterface& orig) {
}

VrepInterface::~VrepInterface() {
}

